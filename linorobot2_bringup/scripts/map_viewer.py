#!/usr/bin/env python3
"""
Live SLAM map viewer over the network -- no rviz needed.

Subscribes to /map (nav_msgs/OccupancyGrid) and the robot pose (/pose from
slam_toolbox while mapping, /amcl_pose from AMCL while navigating), renders a
PNG, and serves a small auto-refreshing web page. Open it in any browser:

  - On the same LAN:        http://192.168.1.59:8000
  - Through an SSH tunnel:   ssh -L 8000:localhost:8000 jetson1@192.168.1.59
                            then open http://localhost:8000 on your laptop

Hover over the map to read map-frame coordinates in meters; click anywhere to
get a ready-to-paste `ros2 action send_goal` command for that spot. The robot
is drawn at its actual footprint size (550mm x 550mm) with a heading line.

The map PNG is encoded once per /map message and cached; the robot footprint
is drawn client-side on a canvas overlay from /meta.json, so pose updates are
cheap and frequent while the heavy image work only happens when the map
actually changes.

robot.launch.py starts this automatically (map_viewer:=false to disable).
To run it standalone on the Jetson (after sourcing ROS):
  source /opt/ros/jazzy/setup.bash
  source ~/linorobot2_ws/install/setup.bash
  python3 ~/Desktop/linorobot2/linorobot2_bringup/scripts/map_viewer.py
"""

import io
import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
from PIL import Image, ImageDraw

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseWithCovarianceStamped

PORT = 8000
UPSCALE = 4          # pixels per map cell, makes the image easier to see
MAP_POLL_MS = 500    # browser meta poll; map image reloads only when it changed
ROBOT_HALF = 0.275   # meters: half of the 550mm x 550mm square footprint


class MapRenderer(Node):
    def __init__(self, map_file=None):
        super().__init__('map_viewer')
        self._lock = threading.Lock()
        self._png = self._placeholder_png()
        self._meta = None   # (res, ox, oy, w, h)
        self._pose = None   # (x, y, yaw) in map frame
        self._seq = 0       # bumps on every new map so the browser knows to reload
        self._slam_pose_time = 0.0
        self._plan = None       # [[x,y], ...] downsampled current global plan
        self._goal = None       # (x, y, yaw) = last pose of the current plan
        self._plan_time = 0.0   # monotonic time of the last /plan message

        # Preload a saved map (map_saver_cli yaml+pgm) so the page has content
        # with no SLAM or map_server running. A live /map still overrides it.
        if map_file:
            self._load_map_file(map_file)

        # /map is published with TRANSIENT_LOCAL (latched) durability.
        map_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, '/map', self._on_map, map_qos)
        # slam_toolbox publishes /pose while mapping; AMCL publishes /amcl_pose
        # while navigating. SLAM wins when both are alive (a stray AMCL running
        # during mapping publishes garbage poses); AMCL is only used when no
        # SLAM pose has arrived recently.
        self.create_subscription(
            PoseWithCovarianceStamped, '/pose',
            lambda m: self._on_pose(m, slam=True), 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose',
            lambda m: self._on_pose(m, slam=False), 10)
        # Nav2's planner_server publishes the current global plan on /plan
        # (republished ~1 Hz while a goal is active). Rendered on the page as
        # a path line with a blue footprint at the destination.
        self.create_subscription(Path, '/plan', self._on_plan, 10)
        # RViz-less "2D Pose Estimate": the web page can publish /initialpose
        # so AMCL can be (re)localized from the browser.
        self._initpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.get_logger().info(f'Map viewer up. Browse to http://<jetson-ip>:{PORT}')

    def publish_initial_pose(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        # Same uncertainty RViz uses for its 2D Pose Estimate tool.
        msg.pose.covariance[0] = 0.25    # x
        msg.pose.covariance[7] = 0.25    # y
        msg.pose.covariance[35] = 0.0685  # yaw
        self._initpose_pub.publish(msg)
        self.get_logger().info(f'initial pose set: x={x:.2f} y={y:.2f} yaw={yaw:.2f}')

    def _on_pose(self, msg, slam):
        now = time.monotonic()
        with self._lock:
            if slam:
                self._slam_pose_time = now
            elif now - self._slam_pose_time < 10.0:
                return  # SLAM is alive; ignore AMCL
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self._lock:
            self._pose = (p.x, p.y, yaw)

    def _on_plan(self, msg):
        if not msg.poses:
            return
        # Downsample to <=150 points: plenty for drawing, tiny meta.json.
        step = max(1, len(msg.poses) // 150)
        pts = [[round(p.pose.position.x, 3), round(p.pose.position.y, 3)]
               for p in msg.poses[::step]]
        last = msg.poses[-1].pose
        pts.append([round(last.position.x, 3), round(last.position.y, 3)])
        q = last.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self._lock:
            self._plan = pts
            self._goal = (round(last.position.x, 3),
                          round(last.position.y, 3), round(yaw, 3))
            self._plan_time = time.monotonic()

    def _on_map(self, msg):
        w, h = msg.info.width, msg.info.height
        if w == 0 or h == 0:
            return
        res = msg.info.resolution
        ox, oy = msg.info.origin.position.x, msg.info.origin.position.y

        data = np.array(msg.data, dtype=np.int16).reshape(h, w)
        # Color map: unknown(-1)=gray, free(0)=white, occupied(100)=black.
        img = np.full((h, w), 205, dtype=np.uint8)        # unknown
        known = data >= 0
        img[known] = (255 * (1.0 - data[known] / 100.0)).astype(np.uint8)

        # OccupancyGrid origin is bottom-left; image origin is top-left -> flip rows.
        img = np.flipud(img)
        pil = Image.fromarray(img, mode='L').convert('RGB')
        pil = pil.resize((w * UPSCALE, h * UPSCALE), Image.NEAREST)

        # Encode ONCE per map message (compress_level=1: speed over size, it's
        # a LAN) and serve the cached bytes. The robot overlay is drawn in the
        # browser, so requests never trigger image work here.
        buf = io.BytesIO()
        pil.save(buf, format='PNG', compress_level=1)
        with self._lock:
            self._png = buf.getvalue()
            self._meta = (res, ox, oy, w, h)
            self._seq += 1

    def _load_map_file(self, yaml_path):
        import os
        import yaml as pyyaml
        with open(yaml_path) as f:
            info = pyyaml.safe_load(f)
        img_path = info['image']
        if not os.path.isabs(img_path):
            img_path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), img_path)
        # The saved pgm is already in image orientation (row 0 = top of map)
        # with the same gray levels the live renderer produces.
        pil = Image.open(img_path).convert('L').convert('RGB')
        w, h = pil.size
        pil = pil.resize((w * UPSCALE, h * UPSCALE), Image.NEAREST)
        buf = io.BytesIO()
        pil.save(buf, format='PNG', compress_level=1)
        ox, oy = float(info['origin'][0]), float(info['origin'][1])
        with self._lock:
            self._png = buf.getvalue()
            self._meta = (float(info['resolution']), ox, oy, w, h)
            self._seq += 1
        self.get_logger().info(f'Loaded saved map {yaml_path} ({w}x{h} cells)')

    def png(self):
        with self._lock:
            return self._png

    def meta_json(self):
        with self._lock:
            if self._meta is None:
                return b'{}'
            res, ox, oy, w, h = self._meta
            pose = None
            if self._pose is not None:
                pose = [round(self._pose[0], 3), round(self._pose[1], 3),
                        round(self._pose[2], 3)]
            # Path/goal go stale 15 s after the last /plan (goal reached or
            # aborted -> nothing republishes -> overlay disappears).
            plan, goal = None, None
            if self._plan and time.monotonic() - self._plan_time < 15.0:
                plan, goal = self._plan, list(self._goal)
            return json.dumps({'res': res, 'ox': ox, 'oy': oy, 'w': w, 'h': h,
                               'upscale': UPSCALE, 'half': ROBOT_HALF,
                               'seq': self._seq, 'pose': pose,
                               'plan': plan, 'goal': goal}).encode()

    @staticmethod
    def _placeholder_png():
        pil = Image.new('RGB', (200, 60), (40, 40, 40))
        ImageDraw.Draw(pil).text((10, 25), 'waiting for /map...', fill=(255, 255, 255))
        buf = io.BytesIO()
        pil.save(buf, format='PNG')
        return buf.getvalue()


HTML = """<!doctype html><html><head><meta charset="utf-8">
<title>SLAM map</title>
<style>
 body{background:#222;color:#eee;font-family:sans-serif;text-align:center;margin:0;padding:12px}
 #wrap{position:relative;display:inline-block}
 img{image-rendering:pixelated;max-width:98vw;border:1px solid #444;cursor:crosshair;display:block}
 #ov{position:absolute;left:0;top:0;pointer-events:none}
 #bar{font-family:monospace;font-size:15px;margin:8px;min-height:1.2em}
 #goal{font-family:monospace;font-size:12px;text-align:left;white-space:pre-wrap;
       word-break:break-all;background:#111;border:1px solid #444;border-radius:4px;
       max-width:98vw;margin:8px auto;padding:8px;display:none}
 small{color:#999}
</style></head><body>
<h3>Live SLAM map &mdash; robot in <span style="color:#f33">red</span>,
goal in <span style="color:#39f">blue</span>,
path in <span style="color:#3c6">green</span></h3>
<div id="bar">hover over the map for coordinates</div>
<div id="wrap"><img id="m" src="/map.png"><canvas id="ov"></canvas></div>
<div><button id="posebtn">&#128205; set robot pose</button></div>
<pre id="goal"></pre>
<small>click on the map to generate a nav goal command for that spot &mdash; or press
"set robot pose", click where the robot IS, then click a point it is FACING
(publishes /initialpose for AMCL, like RViz 2D Pose Estimate)</small>
<script>
 let meta = null, seq = -1;
 const img = document.getElementById('m');
 const cv  = document.getElementById('ov');
 const bar = document.getElementById('bar');
 const goal = document.getElementById('goal');

 let pendingGoal = null;   // [x,y] clicked destination, shown until /plan takes over

 function toPx(x, y, r){   // map meters -> display pixels
   const scale = r.width / img.naturalWidth;
   return [(x - meta.ox) / meta.res * meta.upscale * scale,
           (meta.h - (y - meta.oy) / meta.res) * meta.upscale * scale];
 }

 function footprint(ctx, x, y, yaw, color, dashed, r){
   const pxPerM = meta.upscale / meta.res * (r.width / img.naturalWidth);
   const [px, py] = toPx(x, y, r);
   const half = meta.half * pxPerM;
   ctx.save();
   ctx.translate(px, py);
   ctx.rotate(-yaw);                   // map yaw is CCW, canvas y points down
   ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = 2;
   ctx.setLineDash(dashed ? [6, 4] : []);
   ctx.strokeRect(-half, -half, 2*half, 2*half);
   ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(half, 0); ctx.stroke();
   ctx.setLineDash([]);
   ctx.beginPath(); ctx.arc(0, 0, 3, 0, 7); ctx.fill();
   ctx.restore();
 }

 function drawRobot(){
   const r = img.getBoundingClientRect();
   if(cv.width !== Math.round(r.width) || cv.height !== Math.round(r.height)){
     cv.width = Math.round(r.width); cv.height = Math.round(r.height);
   }
   const ctx = cv.getContext('2d');
   ctx.clearRect(0, 0, cv.width, cv.height);
   if(!meta || !meta.w || !img.naturalWidth) return;
   // Planned path (green line), from Nav2's /plan.
   if(meta.plan && meta.plan.length > 1){
     ctx.strokeStyle = '#3c6'; ctx.lineWidth = 2;
     ctx.beginPath();
     meta.plan.forEach((p, i)=>{
       const [px, py] = toPx(p[0], p[1], r);
       i ? ctx.lineTo(px, py) : ctx.moveTo(px, py);
     });
     ctx.stroke();
   }
   // Destination: blue footprint. Solid when Nav2 has an active plan; dashed
   // right after clicking, before the plan arrives.
   if(meta.goal){
     footprint(ctx, meta.goal[0], meta.goal[1], meta.goal[2], '#39f', false, r);
     pendingGoal = null;               // the real goal took over
   } else if(pendingGoal){
     footprint(ctx, pendingGoal[0], pendingGoal[1], 0, '#39f', true, r);
   }
   // Robot itself (red footprint), on top.
   if(meta.pose) footprint(ctx, meta.pose[0], meta.pose[1], meta.pose[2], '#f33', false, r);
 }

 async function tick(){
   try{ meta = await (await fetch('/meta.json')).json(); }catch(e){ return; }
   if(meta.seq !== undefined && meta.seq !== seq){
     seq = meta.seq;
     img.src = '/map.png?' + seq;      // reload image only when the map changed
   }
   drawRobot();
 }
 setInterval(tick, __POLL__);
 tick();
 img.addEventListener('load', drawRobot);
 window.addEventListener('resize', drawRobot);

 function mapCoords(e){
   if(!meta || !meta.w) return null;
   const r = img.getBoundingClientRect();
   const nx = (e.clientX - r.left) / r.width  * img.naturalWidth;
   const ny = (e.clientY - r.top)  / r.height * img.naturalHeight;
   const x = meta.ox + nx / meta.upscale * meta.res;
   const y = meta.oy + (meta.h - ny / meta.upscale) * meta.res;
   return [x, y];
 }
 img.addEventListener('mousemove', e=>{
   const c = mapCoords(e);
   if(!c) return;
   let t = `cursor  x: ${c[0].toFixed(2)} m   y: ${c[1].toFixed(2)} m`;
   if(meta && meta.pose) t += `   |   robot  x: ${meta.pose[0].toFixed(2)}  y: ${meta.pose[1].toFixed(2)}`;
   bar.textContent = t;
 });
 // Pose-set mode: click 1 = where the robot is, click 2 = a point it faces.
 let poseMode = 0;        // 0=off, 1=await position, 2=await heading
 let poseXY = null;
 const posebtn = document.getElementById('posebtn');
 posebtn.addEventListener('click', ()=>{
   poseMode = poseMode ? 0 : 1;
   poseXY = null;
   posebtn.style.background = poseMode ? '#f33' : '';
   bar.textContent = poseMode ? 'click where the robot IS' : 'pose mode off';
 });

 img.addEventListener('click', async e=>{
   const c = mapCoords(e);
   if(!c) return;
   if(poseMode === 1){
     poseXY = c;
     poseMode = 2;
     bar.textContent = `position (${c[0].toFixed(2)}, ${c[1].toFixed(2)}) — now click a point the robot is FACING`;
     return;
   }
   if(poseMode === 2){
     const yaw = Math.atan2(c[1]-poseXY[1], c[0]-poseXY[0]);
     try{
       await fetch(`/set_initialpose?x=${poseXY[0].toFixed(3)}&y=${poseXY[1].toFixed(3)}&yaw=${yaw.toFixed(3)}`);
       bar.textContent = `initial pose sent: x=${poseXY[0].toFixed(2)} y=${poseXY[1].toFixed(2)} yaw=${yaw.toFixed(2)} — watch the footprint snap`;
     }catch(err){ bar.textContent = 'failed to send initial pose'; }
     poseMode = 0; poseXY = null; posebtn.style.background = '';
     return;
   }
   goal.style.display = 'block';
   goal.textContent =
     `ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose ` +
     `"{pose: {header: {frame_id: map}, pose: {position: {x: ${c[0].toFixed(2)}, y: ${c[1].toFixed(2)}}, orientation: {w: 1.0}}}}"`;
   pendingGoal = c;                    // show the blue destination immediately
   drawRobot();
 });
</script></body></html>""".replace('__POLL__', str(MAP_POLL_MS))


def make_handler(node):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, body, ctype):
            self.send_response(200)
            self.send_header('Content-Type', ctype)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path.startswith('/map.png'):
                self._send(node.png(), 'image/png')
            elif self.path.startswith('/meta.json'):
                self._send(node.meta_json(), 'application/json')
            elif self.path.startswith('/set_initialpose'):
                from urllib.parse import urlparse, parse_qs
                q = parse_qs(urlparse(self.path).query)
                try:
                    node.publish_initial_pose(
                        float(q['x'][0]), float(q['y'][0]), float(q['yaw'][0]))
                    self._send(b'ok', 'text/plain')
                except (KeyError, ValueError):
                    self._send(b'bad params', 'text/plain')
            else:
                self._send(HTML.encode(), 'text/html')
    return H


def main():
    import argparse
    ap = argparse.ArgumentParser(description='Web map viewer (port 8000)')
    ap.add_argument('--map', help='saved map .yaml (from map_saver_cli) to preload')
    args, ros_argv = ap.parse_known_args()

    rclpy.init(args=ros_argv)
    node = MapRenderer(map_file=args.map)
    srv = ThreadingHTTPServer(('0.0.0.0', PORT), make_handler(node))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        srv.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
