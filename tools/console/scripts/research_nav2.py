#!/usr/bin/env python3
"""Nav2 community-issues digest builder.

Scrapes ROS Discourse + GitHub issue search, feeds samples to a local Ollama
model, writes tools/console/NAV2_COMMUNITY_ISSUES.md.

Run on any host with Ollama. Streams the model response so it can't wedge on one long
blocking read, flushes progress, and always writes whatever it got.
"""
import json
import os
import subprocess
import sys
import time
import urllib.parse
import urllib.request

MODEL = os.environ.get("NAV2_RESEARCH_MODEL", "gemma4:12b-256k")
OLLAMA = os.environ.get("OLLAMA_URL", "http://127.0.0.1:11434")
# Host running Ollama. Defaults to this machine; set NAV2_RESEARCH_NODE to
# offload to another one.
NODE = os.environ.get("NAV2_RESEARCH_NODE", "localhost")
OUT_PATH = os.path.expanduser(
    "~/code/linorobot2/tools/console/NAV2_COMMUNITY_ISSUES.md"
)


def log(msg):
    print(msg, flush=True)


DISCOURSE_QUERIES = [
    "nav2 controller oscillation",
    "nav2 rotation in place",
    "nav2 doorway narrow inflation",
    "nav2 recovery behavior loop stuck",
    "nav2 mppi controller tuning",
    "nav2 twist stamped enable_stamped_cmd_vel",
    "nav2 amcl drift localization",
    "nav2 raytrace_range obstacle clearing",
]

GITHUB_QUERIES = [
    "repo:ros-navigation/navigation2 oscillation in:title,body",
    "repo:ros-navigation/navigation2 stuck in:title,body",
    "repo:ros-navigation/navigation2 rotation in:title,body",
    "repo:ros-navigation/navigation2 doorway in:title,body",
    "repo:ros-navigation/navigation2 recovery loop in:title,body",
    "repo:ros-navigation/navigation2 MPPI in:title,body",
    "repo:ros-navigation/navigation2 overshoot in:title,body",
]


def fetch_json(url):
    req = urllib.request.Request(
        url, headers={"User-Agent": "Mozilla/5.0 (Linorobot2 Sub-Agent)"}
    )
    try:
        with urllib.request.urlopen(req, timeout=15) as resp:
            return json.loads(resp.read().decode())
    except Exception as e:
        log(f"[WARN] fetch failed {url}: {e}")
        return None


discourse_results = []
log("--> Fetching ROS Discourse discussions...")
for q in DISCOURSE_QUERIES:
    url = f"https://discourse.openrobotics.org/search.json?q={urllib.parse.quote(q)}"
    data = fetch_json(url)
    if data and "posts" in data:
        for p in data["posts"][:3]:
            discourse_results.append({
                "source": "ROS Discourse",
                "query": q,
                "topic_id": p.get("topic_id"),
                "blurb": (p.get("blurb") or "").replace("\n", " "),
                "created_at": p.get("created_at"),
            })
    time.sleep(0.5)

github_results = []
log("--> Fetching GitHub issues from ros-navigation/navigation2...")
for q in GITHUB_QUERIES:
    url = f"https://api.github.com/search/issues?q={urllib.parse.quote(q)}&per_page=4"
    data = fetch_json(url)
    if data and "items" in data:
        for it in data["items"]:
            github_results.append({
                "source": "GitHub Issues",
                "number": it.get("number"),
                "title": it.get("title"),
                "html_url": it.get("html_url"),
                "state": it.get("state"),
                "body_snippet": (it.get("body") or "")[:300].replace("\n", " "),
            })
    time.sleep(0.8)  # unauthenticated GitHub search = 10 req/min

log(f"--> Collected {len(discourse_results)} Discourse posts, {len(github_results)} GitHub issues.")

summary_prompt = f"""You are an expert autonomous robotics systems architect specializing in ROS 2 Navigation 2 (Nav2).
Analyze the collected ROS Discourse posts and GitHub issues regarding Nav2 common issues, bugs, and tuning traps.

Synthesize an exhaustive, authoritative community troubleshooting guide formatted in professional Markdown titled:
# Nav2 Community Issues, Real-World Pitfalls & Production Solutions

Structure the report into these 8 core sections:
1. Kinematic Drift & In-Place Rotation Slip (Differential vs Mecanum, RotationShimController, EKF v_y filtering, raw IMU yaw vs angular velocity)
2. Path Tracking Oscillation & Trajectory Hunting (DWB critic balance, Regulated Pure Pursuit lookahead distance, MPPI windshield wiper wobble)
3. Goal Overshoot & Tolerance Hunting (Approach velocity scaling, lookahead reduction, braking deceleration limits, goal_checker tolerances)
4. Doorway Traversal, Narrow Corridors & Inflation Traps (Circumscribed radius, inflation_radius overlap, cost_scaling_factor steepness)
5. Costmap Ghost Obstacles & Dynamic Obstacle Clearing (raytrace_range > obstacle_max_range rule, voxel decay, lidar reflection noise)
6. Recovery Behavior Death Spirals & BT Aborts (RecoveryNode retry counter bugs #6236/#6427, ProgressChecker timeouts, spin-backup loops)
7. Localization Covariance Divergence & Jumps (AMCL symmetrical ambiguity, beam vs likelihood field, covariance inflation, odometry loss)
8. TwistStamped vs Twist Migration & Multi-Distro Parity (enable_stamped_cmd_vel flag in Humble/Jazzy/Rolling, RMW message drops, TF extrapolation)

For each section:
- Real-World Symptoms & Community Problem Description
- Underlying Root Cause (Kinematics, Physics, Algorithm, or QoS)
- Concrete Recommended Parameter Fixes (with exact YAML snippet)
- Production Best Practice / Architecture Guardrail

Collected Discourse Samples:
{json.dumps(discourse_results[:12], indent=2)}

Collected GitHub Samples:
{json.dumps(github_results[:12], indent=2)}
"""

log(f"--> Querying Ollama model '{MODEL}' at {OLLAMA} (streaming)...")
req_data = json.dumps({
    "model": MODEL,
    "prompt": summary_prompt,
    "stream": True,
    "options": {"temperature": 0.2, "num_predict": 6144, "num_ctx": 16384},
}).encode()
req = urllib.request.Request(
    f"{OLLAMA}/api/generate", data=req_data,
    headers={"Content-Type": "application/json"},
)

report_text = ""
t0 = time.time()
try:
    with urllib.request.urlopen(req, timeout=600) as resp:
        for raw in resp:
            raw = raw.strip()
            if not raw:
                continue
            try:
                chunk = json.loads(raw)
            except json.JSONDecodeError:
                continue
            report_text += chunk.get("response", "")
            if chunk.get("done"):
                break
            if len(report_text) % 2000 < 40:
                log(f"    ...{len(report_text)} chars ({time.time() - t0:.0f}s)")
except Exception as e:
    log(f"[ERROR] Ollama call failed after {time.time() - t0:.0f}s: {e}")

report_text = report_text.strip()
if report_text:
    header = (
        f"<!-- generated by research_nav2 on {NODE} via Ollama {MODEL}, "
        f"{time.strftime('%Y-%m-%d %H:%M:%S%z')}, "
        f"{len(discourse_results)} discourse + {len(github_results)} github samples -->\n\n"
    )
    with open(OUT_PATH, "w") as f:
        f.write(header + report_text + "\n")
    log(f"--> Saved {len(report_text)} chars -> {OUT_PATH}")
    ok = True
else:
    log("[ERROR] empty response; not overwriting the output file")
    ok = False

notify = os.path.expanduser("~/bin/notify-agent")
if os.path.exists(notify):
    subprocess.run([
        notify, f"Sub-Agent ({NODE})",
        f"Nav2 community digest {'DONE ' + str(len(report_text)) + ' chars' if ok else 'FAILED (empty response)'}",
    ], check=False)

sys.exit(0 if ok else 1)
