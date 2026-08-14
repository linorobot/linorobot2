# Create custom Gazebo worlds from images, floor plans, or occupancy grid map

linorobot2 includes a GUI tool for generating Gazebo worlds from real-world sources: a floor plan image or a SLAM-generated map. It produces a Gazebo world that reflects the actual geometry of your physical environment, so you can develop and test ROS2 applications in simulation with full confidence in the obstacle layout before deploying to the Physical Robot.

---

## world_creator

A GUI tool that converts a SLAM-generated map or any floor plan image (PNG, JPG, BMP, etc.) into a ready-to-use Gazebo world. The tool generates the 3D wall mesh, model SDF, and world SDF automatically.

Under the hood, it traces every dark (occupied) pixel in the image and extrudes it into a 3D wall mesh exported as an STL file.

### Running

```bash
ros2 run linorobot2_gazebo world_creator
```

The **Map Image Processor** window opens with a **Controls** sidebar on the left and an **Image View** canvas on the right.

### Workflow

There are two workflows depending on your source material. Use **Load Map** when you have a SLAM-generated map (recommended). Use **Load Image** when you only have a raw floor plan image and need to calibrate the scale and origin manually.

---

#### Workflow A — From a SLAM map (recommended)

**1. Load Map**

Click **Load Map** and select the `.yaml` file produced by SLAM Toolbox (or any `map_server`-compatible YAML). The file dialog opens at `linorobot2_navigation/maps/` by default.

The tool reads the YAML and automatically sets:
- The occupancy grid image
- The resolution (metres/pixel)
- The world coordinate origin
- The world name (pre-filled from the map filename in the Generate dialog)

The image is displayed on the canvas with the origin marker shown.

Skip ahead to **Set Wall Height** and then **Generate World**.

---

#### Workflow B — From a floor plan image (manual calibration)

**1. Load Image**

Click **Load Image** and select your floor plan file (PNG, JPG, JPEG, BMP, GIF, or TIFF).

The file dialog opens at `linorobot2_gazebo/linorobot2_gazebo/images/` by default. Place your floor plan images there for easy access.

The image is displayed on the canvas. X (red, pointing right) and Y (green, pointing up) reference axes are drawn in the bottom-left corner.

---

**2. Set Meters Per Pixel**

This step calibrates the real-world scale of the image.

1. Click **Set Meters Per Pixel**. The status bar prompts you to click two points.
2. Click the first point on the canvas (a red dot appears).
3. Click the second point (another red dot and a connecting line appear).
4. A dialog asks for the real-world distance between the two points in metres. Enter the known distance and click **OK**.

The resolution (metres/pixel) is calculated and shown in the **Map Info** panel.

---

**3. Set Origin**

This step defines where the Gazebo world coordinate origin `(0, 0)` falls on the image.

1. Click **Set Origin**. The status bar prompts you to click a point.
2. Click the desired origin location on the image (e.g. a doorway, room corner, or the robot's starting position).

A small circle with red (X) and green (Y) arrows is drawn at the clicked point. The computed world-frame origin `[x, y, 0.0]` is shown in the **Map Info** panel.

> The origin follows the ROS `map_server` convention: the pixel you click becomes `(0, 0)` in the world frame, with Y pointing up in the image.

---

#### Both workflows continue here

**Set Wall Height**

In the **Wall Height** field (default `0.5` m), enter the desired extrusion height for the walls.

---

**Generate World**

Click **Generate World**. A dialog appears with three fields:

| Field | Description |
|-------|-------------|
| **World Name** | A human-readable name (spaces and CamelCase are converted to `snake_case`). A live preview shows the final name. |
| **Model Directory** | Directory where the model folder will be created. Defaults to `linorobot2_gazebo/models/`. |
| **World SDF Directory** | Directory where the world SDF file will be written. Defaults to `linorobot2_gazebo/worlds/`. |

Click **Generate** (or press Enter). A progress splash is shown while the mesh is being built. When complete, a success dialog confirms the output paths.

### Output Files

For a world named `my_map` the following files are created:

```
linorobot2_gazebo/models/
└── my_map/
    ├── model.config
    ├── my_map.sdf
    └── meshes/
        └── my_map.stl

linorobot2_gazebo/worlds/
└── my_map.sdf
```

### Launching the Generated World

After rebuilding the workspace (or if you used `--symlink-install`), launch the simulation with:

```bash
ros2 launch linorobot2_gazebo gazebo.launch.py world_name:=my_map
```

The `world_name` argument resolves to `linorobot2_gazebo/worlds/<world_name>.sdf`, so pass the base filename without the `.sdf` extension.

To use a world SDF located outside the package, pass the full path instead:

```bash
ros2 launch linorobot2_gazebo gazebo.launch.py world_path:=/absolute/path/to/my_map.sdf
```

### Typical Workflow

1. Drive your Physical Robot and build a map with SLAM Toolbox (see [Mapping](07_mapping.md)).
2. Save the map to `linorobot2_navigation/maps/`.
3. Open `world_creator`, click **Load Map**, select the saved map YAML, set the wall height, and generate the world.
4. Rebuild the workspace and launch the world in Gazebo.
5. Develop and test your Nav2 application with the Simulated Robot, then deploy to the Physical Robot.
