import tkinter as tk
from tkinter import ttk, filedialog
from PIL import Image, ImageTk
import os
import math
import numpy as np
import threading
import re
import yaml
from linorobot2_gazebo.map_to_gazebo import process_maps

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_PKG_DIR = os.path.dirname(_SCRIPT_DIR)


def _resolve_src_pkg_dir() -> str | None:
    """
    Locate <workspace>/src/linorobot2_gazebo/ regardless of whether the
    script is running from the colcon install tree or directly from source.

    Installed layout:
      <ws>/install/linorobot2_gazebo/lib/python3.12/site-packages/linorobot2_gazebo/image_to_gazebo.py
      parts: ['', 'home', 'ros', 'linorobot2_ws', 'install', 'linorobot2_gazebo', 'lib',
              'python3.12', 'site-packages', 'linorobot2_gazebo', 'image_to_gazebo.py']

    Source layout:
      <ws>/src/linorobot2_gazebo/linorobot2_gazebo/image_to_gazebo.py
      parts: ['', 'home', 'ros', 'linorobot2_ws', 'src', 'linorobot2_gazebo',
              'linorobot2_gazebo', 'image_to_gazebo.py']

    Finds the first ('install'|'src') + 'linorobot2_gazebo' pair, takes everything
    before it as the workspace root, then returns <workspace>/src/linorobot2_gazebo/
    if that directory actually exists.
    """
    parts = os.path.abspath(__file__).split(os.sep)
    for marker in ('install', 'src'):
        for i, part in enumerate(parts[:-1]):
            if part == marker and parts[i + 1] == 'linorobot2_gazebo':
                workspace_root = os.sep.join(parts[:i]) or os.sep
                candidate = os.path.join(workspace_root, 'src', 'linorobot2', 'linorobot2_gazebo')
                if os.path.isdir(candidate):
                    return candidate

    return None


_SRC_PKG_DIR = _resolve_src_pkg_dir()
_DEFAULT_IMAGES_DIR = (
    os.path.join(_SRC_PKG_DIR, 'linorobot2_gazebo', 'images')
    if _SRC_PKG_DIR else os.path.join(_SCRIPT_DIR, 'images')
)
_DEFAULT_MAPS_DIR = (
    os.path.join(os.path.dirname(_SRC_PKG_DIR), 'linorobot2_navigation', 'maps')
    if _SRC_PKG_DIR else None
)

_DEFAULT_MODELS_DIR = (
    os.path.join(_SRC_PKG_DIR, 'models')
    if _SRC_PKG_DIR else os.path.join(_PKG_DIR, 'models')
)
_DEFAULT_WORLDS_DIR = (
    os.path.join(_SRC_PKG_DIR, 'worlds')
    if _SRC_PKG_DIR else os.path.join(_PKG_DIR, 'worlds')
)


class MapImageProcessor(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Map Image Processor")
        self.geometry("1000x700")

        # Private map_info variable with default values
        self.__map_info = {
            "map_name": None,
            "image": None,
            "resolution": None,
            "origin": [0.0, 0.0, 0.0],
            "negate": 0,
            "occupied_thresh": 0.65,
            "free_thresh": 0.196,
            "origin_pixel": None  # Store pixel coordinates for redrawing
        }

        # State variables
        self.current_image = None
        self.image_path = None
        self.tk_image = None
        self.canvas_image = None
        self.click_points = []
        self.click_count = 0
        self.click_mode = None

        self.create_widgets()

    def create_widgets(self):
        # Main frame
        main_frame = ttk.Frame(self)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Sidebar frame (left)
        sidebar_frame = ttk.LabelFrame(main_frame, text="Controls")
        sidebar_frame.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))

        # Image view frame (right)
        self.image_frame = ttk.LabelFrame(main_frame, text="Image View")
        self.image_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        # Canvas for displaying the image
        self.canvas = tk.Canvas(self.image_frame, bg="white")
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.canvas.bind("<Button-1>", self.on_canvas_click)

        # Status bar at the bottom of the canvas
        self.status_bar = ttk.Label(self.image_frame, text="No image loaded", anchor=tk.W)
        self.status_bar.pack(fill=tk.X, padx=5, pady=2)

        # Sidebar controls
        # Load Map (YAML) button
        load_map_button = ttk.Button(sidebar_frame, text="Load Map", command=self.load_map)
        load_map_button.pack(fill=tk.X, padx=10, pady=5)

        ttk.Separator(sidebar_frame).pack(fill=tk.X, padx=10, pady=5)

        # Load image button
        load_button = ttk.Button(sidebar_frame, text="Load Image", command=self.load_image)
        load_button.pack(fill=tk.X, padx=10, pady=5)

        ttk.Separator(sidebar_frame).pack(fill=tk.X, padx=10, pady=5)

        # Set Meters Per Pixel button
        self.meters_pixel_button = ttk.Button(sidebar_frame, text="Set Meters Per Pixel", command=self.set_meters_per_pixel)
        self.meters_pixel_button.pack(fill=tk.X, padx=10, pady=5)

        # Set Origin button
        self.origin_button = ttk.Button(sidebar_frame, text="Set Origin", command=self.set_origin)
        self.origin_button.pack(fill=tk.X, padx=10, pady=5)

        ttk.Separator(sidebar_frame).pack(fill=tk.X, padx=10, pady=5)

        # Wall Height input
        wall_height_frame = ttk.Frame(sidebar_frame)
        wall_height_frame.pack(fill=tk.X, padx=10, pady=5)
        ttk.Label(wall_height_frame, text="Wall Height:").pack(side=tk.LEFT)
        self.wall_height_var = tk.DoubleVar(value=0.5)
        wall_height_entry = ttk.Entry(wall_height_frame, textvariable=self.wall_height_var, width=10)
        wall_height_entry.pack(side=tk.RIGHT)

        ttk.Separator(sidebar_frame).pack(fill=tk.X, padx=10, pady=5)

        # Generate World button
        self.generate_button = ttk.Button(sidebar_frame, text="Generate World", command=self.generate_world)
        self.generate_button.pack(fill=tk.X, padx=10, pady=5)

        # Info display
        info_frame = ttk.LabelFrame(sidebar_frame, text="Map Info")
        info_frame.pack(fill=tk.X, padx=10, pady=10)

        self.resolution_var = tk.StringVar(value="Resolution: Not set")
        resolution_label = ttk.Label(info_frame, textvariable=self.resolution_var)
        resolution_label.pack(anchor=tk.W, padx=5, pady=2)

        self.origin_var = tk.StringVar(value="Origin: [0.0, 0.0, 0.0]")
        origin_label = ttk.Label(info_frame, textvariable=self.origin_var)
        origin_label.pack(anchor=tk.W, padx=5, pady=2)

    def load_map(self):
        """Load a SLAM-generated .yaml map file and populate all map info automatically"""
        file_types = [
            ("Map YAML files", "*.yaml *.yml"),
            ("All files", "*.*")
        ]
        yaml_path = filedialog.askopenfilename(
            title="Select a map YAML file",
            filetypes=file_types,
            initialdir=_DEFAULT_MAPS_DIR if _DEFAULT_MAPS_DIR and os.path.isdir(_DEFAULT_MAPS_DIR) else None
        )

        if not yaml_path:
            return

        try:
            with open(yaml_path, 'r') as f:
                map_data = yaml.safe_load(f)
        except Exception as e:
            self._show_result_dialog("Error", f"Failed to read YAML file:\n{str(e)}", is_error=True)
            return

        # Validate required fields
        if 'image' not in map_data:
            self._show_result_dialog("Error", "YAML file has no 'image' field.", is_error=True)
            return
        if 'resolution' not in map_data:
            self._show_result_dialog("Error", "YAML file has no 'resolution' field.", is_error=True)
            return

        # Resolve image path: always a filename next to the YAML
        yaml_dir = os.path.dirname(os.path.abspath(yaml_path))
        image_path = os.path.join(yaml_dir, map_data['image'])

        if not os.path.isfile(image_path):
            self._show_result_dialog(
                "Error",
                f"Map image not found:\n{image_path}\n\nMake sure '{map_data['image']}' is in the same directory as the YAML.",
                is_error=True
            )
            return

        try:
            loaded_image = Image.open(image_path)
        except Exception as e:
            self._show_result_dialog("Error", f"Failed to open map image:\n{str(e)}", is_error=True)
            return

        resolution = float(map_data['resolution'])
        origin = map_data.get('origin', [0.0, 0.0, 0.0])
        if isinstance(origin, (list, tuple)) and len(origin) >= 2:
            origin = [float(origin[0]), float(origin[1]), float(origin[2]) if len(origin) > 2 else 0.0]
        else:
            origin = [0.0, 0.0, 0.0]

        map_name = os.path.splitext(os.path.basename(yaml_path))[0]
        img_width, img_height = loaded_image.size

        # Compute origin pixel from YAML world coordinates (inverse of set_origin_point formula)
        origin_pixel_x = -origin[0] / resolution - 0.5
        origin_pixel_y = img_height + origin[1] / resolution - 0.5

        self.__map_info = {
            "map_name": map_name,
            "image": image_path,
            "resolution": resolution,
            "origin": origin,
            "negate": int(map_data.get('negate', 0)),
            "occupied_thresh": float(map_data.get('occupied_thresh', 0.65)),
            "free_thresh": float(map_data.get('free_thresh', 0.196)),
            "origin_pixel": (origin_pixel_x, origin_pixel_y)
        }

        self.image_path = image_path
        self.current_image = loaded_image
        self.canvas.delete("all")
        self.click_points = []
        self.click_count = 0
        self.click_mode = None

        self.resolution_var.set(f"Resolution: {resolution:.6f} meters/pixel")
        self.origin_var.set(f"Origin: [{origin[0]:.2f}, {origin[1]:.2f}, {origin[2]:.2f}]")

        self.display_image()
        self.status_bar.config(text=f"Map loaded: {os.path.basename(yaml_path)}")

    def load_image(self):
        """Load an image file and display it on the canvas"""
        file_types = [
            ("Image files", "*.png *.jpg *.jpeg *.bmp *.gif *.tiff"),
            ("All files", "*.*")
        ]
        file_path = filedialog.askopenfilename(
            title="Select an image file",
            filetypes=file_types,
            initialdir=_DEFAULT_IMAGES_DIR if os.path.isdir(_DEFAULT_IMAGES_DIR) else None
        )

        if file_path:
            try:
                # Reset map_info when loading a new image
                self.__map_info = {
                    "map_name": None,
                    "image": file_path,
                    "resolution": None,
                    "origin": [0.0, 0.0, 0.0],
                    "negate": 0,
                    "occupied_thresh": 0.65,
                    "free_thresh": 0.196,
                    "origin_pixel": None
                }

                # Add map_name based on the YAML filename
                map_name = os.path.splitext(os.path.basename(file_path))[0]
                self.__map_info['map_name'] = map_name

                # Update UI to reflect reset map_info
                self.resolution_var.set(f"Resolution: Not set")
                self.origin_var.set(f"Origin: [0.0, 0.0, 0.0]")

                # Load and display the image
                self.image_path = file_path
                self.current_image = Image.open(file_path)

                # Clear any markers from previous image
                self.canvas.delete("all")
                self.click_points = []
                self.click_count = 0
                self.click_mode = None

                self.display_image()
                self.status_bar.config(text=f"Loaded: {os.path.basename(file_path)}")
            except Exception as e:
                self._show_result_dialog("Error", f"Failed to load image: {str(e)}", is_error=True)

    def display_image(self):
        """Display the current image on the canvas"""
        if self.current_image:
            # Clear existing canvas items
            self.canvas.delete("all")

            # Get canvas dimensions
            canvas_width = self.canvas.winfo_width()
            canvas_height = self.canvas.winfo_height()

            # Resize image to fit canvas while maintaining aspect ratio
            img_width, img_height = self.current_image.size

            # If canvas size is too small, wait for it to be properly sized
            if canvas_width <= 1 or canvas_height <= 1:
                self.after(100, self.display_image)
                return

            # Calculate scale factor
            scale_w = canvas_width / img_width
            scale_h = canvas_height / img_height
            scale = min(scale_w, scale_h)

            # Calculate new dimensions
            new_width = int(img_width * scale)
            new_height = int(img_height * scale)

            # Resize the image
            resized_img = self.current_image.resize((new_width, new_height), Image.LANCZOS)
            self.tk_image = ImageTk.PhotoImage(resized_img)

            # Calculate position to center the image
            x_pos = (canvas_width - new_width) // 2
            y_pos = (canvas_height - new_height) // 2

            # Display image on canvas
            self.canvas_image = self.canvas.create_image(x_pos, y_pos, anchor=tk.NW, image=self.tk_image)

            # Store the scale factor for later use
            self.image_scale = scale
            self.image_offset = (x_pos, y_pos)

            # Redraw any markers
            self.redraw_markers()

            # Draw axis arrows on top
            self.draw_axis_arrows()

    def redraw_markers(self):
        """Redraw markers on the canvas if any exist"""
        # Redraw origin marker if it exists
        if self.__map_info.get("origin_pixel") is not None:
            x, y = self.__map_info["origin_pixel"]

            # Convert to canvas coordinates
            canvas_x = self.image_offset[0] + x * self.image_scale
            canvas_y = self.image_offset[1] + y * self.image_scale

            # Clear previous origin marker
            self.canvas.delete("origin_marker")

            # Draw axis arrows at origin
            self._draw_origin_marker(canvas_x, canvas_y)

    def _draw_origin_marker(self, canvas_x, canvas_y):
        """Draw X (red, right) and Y (green, up) axis arrows centred on the origin point."""
        arrow_len = 40
        self.canvas.create_line(
            canvas_x, canvas_y, canvas_x + arrow_len, canvas_y,
            fill="red", width=2, arrow=tk.LAST, arrowshape=(10, 12, 4),
            tags="origin_marker"
        )
        self.canvas.create_text(
            canvas_x + arrow_len + 10, canvas_y,
            text="X", fill="red", font=("TkDefaultFont", 9, "bold"),
            tags="origin_marker"
        )
        self.canvas.create_line(
            canvas_x, canvas_y, canvas_x, canvas_y - arrow_len,
            fill="green", width=2, arrow=tk.LAST, arrowshape=(10, 12, 4),
            tags="origin_marker"
        )
        self.canvas.create_text(
            canvas_x, canvas_y - arrow_len - 10,
            text="Y", fill="green", font=("TkDefaultFont", 9, "bold"),
            tags="origin_marker"
        )
        # Small dot at the origin point
        self.canvas.create_oval(
            canvas_x - 3, canvas_y - 3, canvas_x + 3, canvas_y + 3,
            fill="white", outline="black", width=1, tags="origin_marker"
        )

    def draw_axis_arrows(self):
        """Draw X (red, right) and Y (green, up) axis arrows in the bottom-left of the canvas."""
        canvas_width = self.canvas.winfo_width()
        canvas_height = self.canvas.winfo_height()

        self.canvas.delete("axis_arrows")

        origin_x = 40
        origin_y = canvas_height - 40
        arrow_len = 50

        # X axis — red, pointing right
        self.canvas.create_line(
            origin_x, origin_y, origin_x + arrow_len, origin_y,
            fill="red", width=2, arrow=tk.LAST, arrowshape=(10, 12, 4),
            tags="axis_arrows"
        )
        self.canvas.create_text(
            origin_x + arrow_len + 10, origin_y,
            text="X", fill="red", font=("TkDefaultFont", 10, "bold"),
            tags="axis_arrows"
        )

        # Y axis — green, pointing up (negative canvas-y direction)
        self.canvas.create_line(
            origin_x, origin_y, origin_x, origin_y - arrow_len,
            fill="green", width=2, arrow=tk.LAST, arrowshape=(10, 12, 4),
            tags="axis_arrows"
        )
        self.canvas.create_text(
            origin_x, origin_y - arrow_len - 10,
            text="Y", fill="green", font=("TkDefaultFont", 10, "bold"),
            tags="axis_arrows"
        )

    def on_canvas_click(self, event):
        """Handle clicks on the canvas based on the current mode"""
        if not self.current_image or not self.canvas_image:
            return

        # Get the actual position on the image
        x = (event.x - self.image_offset[0]) / self.image_scale
        y = (event.y - self.image_offset[1]) / self.image_scale  # Fixed: was using image_offset[0] for y

        if self.click_mode == "origin":
            self.set_origin_point(x, y)
        elif self.click_mode == "meters_per_pixel":
            self.process_meter_pixel_click(event.x, event.y)

    def set_meters_per_pixel(self):
        """Start the process of setting meters per pixel by clicking two points"""
        if not self.current_image:
            self._show_result_dialog("Info", "Please load an image first.")
            return

        self.click_mode = "meters_per_pixel"
        self.click_points = []
        self.click_count = 0
        self.status_bar.config(text="Click on two points to measure distance")

    def process_meter_pixel_click(self, x, y):
        """Process clicks for the meters per pixel calculation"""
        # Draw a marker at the clicked position
        marker_id = self.canvas.create_oval(x-5, y-5, x+5, y+5, fill="red", outline="white", tags=f"distance_marker_{self.click_count}")

        # Store the click point
        self.click_points.append((x, y))
        self.click_count += 1

        if self.click_count == 1:
            self.status_bar.config(text="Click on second point")
        elif self.click_count == 2:
            # Calculate pixel distance between the two points
            x1, y1 = self.click_points[0]
            x2, y2 = self.click_points[1]
            pixel_distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

            # Draw a line between the points
            self.canvas.create_line(x1, y1, x2, y2, fill="red", width=2, tags="distance_line")

            # Ask user for the real-world distance
            real_distance = self._ask_float_dialog("Distance",
                                                   "Enter the distance in meters:",
                                                   minvalue=0.0001)

            if real_distance:
                # Calculate meters per pixel
                meters_per_pixel = real_distance / (pixel_distance / self.image_scale)

                # Update map_info
                self.__map_info["resolution"] = meters_per_pixel
                self.resolution_var.set(f"Resolution: {meters_per_pixel:.6f} meters/pixel")

                self.status_bar.config(text=f"Meters per pixel set: {meters_per_pixel:.6f}")

                # Clear the distance markers after a delay (they stay visible long enough for user to see)
                self.after(2000, self.clear_distance_markers)
            else:
                # If user canceled, clear markers immediately
                self.clear_distance_markers()

            # Reset click mode
            self.click_mode = None
            self.click_count = 0

    def clear_distance_markers(self):
        """Clear the distance measurement markers"""
        self.canvas.delete("distance_marker_0")
        self.canvas.delete("distance_marker_1")
        self.canvas.delete("distance_line")

    def set_origin(self):
        """Set the origin point on the image"""
        if not self.current_image:
            self._show_result_dialog("Info", "Please load an image first.")
            return

        if self.__map_info["resolution"] is None:
            self._show_result_dialog("Info", "Please set meters per pixel first.")
            return

        self.click_mode = "origin"
        self.status_bar.config(text="Click to set the origin point")

    def set_origin_point(self, x, y):
        """Set the origin at the clicked point and convert to world coordinates"""
        if not self.current_image:
            return

        # Store the original clicked position for drawing the marker
        clicked_x, clicked_y = x, y

        # Convert from image coordinates to world coordinates
        # In ROS map_server, origin is in bottom-left corner in world frame
        # Note: y-axis is inverted in image coordinates
        img_height = self.current_image.height

        # Convert pixel coordinates to world coordinates using the ROS map_server convention
        # The y coordinate needs to be flipped because in image coordinates,
        # origin is at top-left, but in ROS coordinates, origin is at bottom-left
        world_x = - ((x + 0.5) * self.__map_info["resolution"])
        world_y = - (((img_height - y) + 0.5) * self.__map_info["resolution"])

        # Update map_info origin
        self.__map_info["origin"] = [world_x, world_y, 0.0]
        # Store the pixel coordinates of the origin for redrawing later
        self.__map_info["origin_pixel"] = (clicked_x, clicked_y)
        self.origin_var.set(f"Origin: [{world_x:.2f}, {world_y:.2f}, 0.0]")

        # Draw a marker at the EXACT clicked position (not at the computed origin)
        # Convert clicked position to canvas coordinates
        canvas_x = self.image_offset[0] + clicked_x * self.image_scale
        canvas_y = self.image_offset[1] + clicked_y * self.image_scale

        # Clear previous origin marker if exists
        self.canvas.delete("origin_marker")

        # Draw axis arrows at the clicked position
        self._draw_origin_marker(canvas_x, canvas_y)

        self.status_bar.config(text=f"Origin set at world coordinates: [{world_x:.2f}, {world_y:.2f}, 0.0]")
        self.click_mode = None

    def _convert_world_name(self, raw_name: str) -> str:
        """Convert a raw name to snake_case format."""
        s = re.sub(r'([a-z])([A-Z])', r'\1_\2', raw_name)  # CamelCase → under_score
        s = re.sub(r'\s+', '_', s)                           # spaces → underscore
        return s.lower()                                      # lowercase

    def _show_result_dialog(self, title: str, message: str, is_error: bool = False):
        """Small centered modal result dialog, similar in size to _ask_float_dialog."""
        dialog = tk.Toplevel(self)
        dialog.title(title)
        dialog.resizable(False, False)
        dialog.transient(self)

        self.update_idletasks()
        px = self.winfo_x() + self.winfo_width() // 2 - 130
        py = self.winfo_y() + self.winfo_height() // 2 - 55
        dialog.geometry(f"260x110+{px}+{py}")
        dialog.wait_visibility()
        dialog.grab_set()

        color = "red" if is_error else "black"
        ttk.Label(dialog, text=message, wraplength=230, justify=tk.LEFT,
                  foreground=color).pack(padx=15, pady=(15, 10))
        ttk.Button(dialog, text="OK", command=dialog.destroy).pack(pady=(0, 10))

        dialog.bind("<Return>", lambda e: dialog.destroy())
        dialog.bind("<Escape>", lambda e: dialog.destroy())
        dialog.wait_window()

    def _ask_float_dialog(self, title: str, prompt: str, minvalue: float = None) -> float | None:
        """Small centered modal dialog to ask for a float value."""
        result = [None]

        dialog = tk.Toplevel(self)
        dialog.title(title)
        dialog.resizable(False, False)
        dialog.transient(self)

        # Center on parent
        self.update_idletasks()
        px = self.winfo_x() + self.winfo_width() // 2 - 115
        py = self.winfo_y() + self.winfo_height() // 2 - 70
        dialog.geometry(f"230x140+{px}+{py}")
        dialog.wait_visibility()
        dialog.grab_set()

        ttk.Label(dialog, text=prompt).pack(padx=15, pady=(15, 5))
        entry_var = tk.StringVar()
        entry = ttk.Entry(dialog, textvariable=entry_var, width=20)
        entry.pack(padx=15, pady=5)
        entry.focus_set()

        error_label = ttk.Label(dialog, text="", foreground="red")
        error_label.pack()

        def on_ok(event=None):
            try:
                val = float(entry_var.get())
                if minvalue is not None and val < minvalue:
                    error_label.config(text=f"Value must be >= {minvalue}")
                    return
                result[0] = val
                dialog.destroy()
            except ValueError:
                error_label.config(text="Please enter a valid number")

        def on_cancel(event=None):
            dialog.destroy()

        btn_frame = ttk.Frame(dialog)
        btn_frame.pack(pady=5)
        ttk.Button(btn_frame, text="OK", command=on_ok).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="Cancel", command=on_cancel).pack(side=tk.LEFT, padx=5)

        dialog.bind("<Return>", on_ok)
        dialog.bind("<Escape>", on_cancel)

        dialog.wait_window()
        return result[0]

    def _ask_world_save_dialog(self, default_name: str = "") -> tuple[str | None, str | None, str | None]:
        """Centered dialog to get world name, model dir, and world SDF dir before generating."""
        result = [None, None, None]

        dialog = tk.Toplevel(self)
        dialog.title("Generate World")
        dialog.resizable(False, False)
        dialog.transient(self)

        # Center on parent
        self.update_idletasks()
        px = self.winfo_x() + self.winfo_width() // 2 - 210
        py = self.winfo_y() + self.winfo_height() // 2 - 165
        dialog.geometry(f"420x330+{px}+{py}")
        dialog.wait_visibility()
        dialog.grab_set()

        # World Name row
        ttk.Label(dialog, text="World Name:").pack(anchor=tk.W, padx=15, pady=(15, 0))
        name_var = tk.StringVar(value=default_name)
        name_entry = ttk.Entry(dialog, textvariable=name_var, width=45)
        name_entry.pack(fill=tk.X, padx=15, pady=(3, 0))
        name_entry.focus_set()

        # Live preview label
        preview_var = tk.StringVar(value="Preview: ")
        ttk.Label(dialog, textvariable=preview_var, foreground="gray").pack(anchor=tk.W, padx=15)

        def on_name_change(*args):
            raw = name_var.get()
            if raw.strip():
                preview_var.set(f"Preview: {self._convert_world_name(raw)}")
            else:
                preview_var.set("Preview: ")

        name_var.trace_add("write", on_name_change)

        def make_path_row(label_text, default_dir):
            ttk.Label(dialog, text=label_text).pack(anchor=tk.W, padx=15, pady=(10, 0))
            row = ttk.Frame(dialog)
            row.pack(fill=tk.X, padx=15, pady=(3, 0))
            var = tk.StringVar(value=default_dir)
            entry = ttk.Entry(row, textvariable=var, state="readonly", width=36)
            entry.pack(side=tk.LEFT, expand=True, fill=tk.X)

            def browse(v=var, d=default_dir):
                chosen = filedialog.askdirectory(
                    title=f"Select {label_text.rstrip(':')}",
                    initialdir=d if os.path.isdir(d) else None
                )
                if chosen:
                    v.set(chosen)

            ttk.Button(row, text="Browse", command=browse).pack(side=tk.LEFT, padx=(5, 0))
            return var

        model_var = make_path_row("Model Directory:", _DEFAULT_MODELS_DIR)
        world_var = make_path_row("World SDF Directory:", _DEFAULT_WORLDS_DIR)

        error_label = ttk.Label(dialog, text="", foreground="red")
        error_label.pack(pady=(8, 0))

        def on_generate(event=None):
            raw_name = name_var.get().strip()
            model_path = model_var.get().strip()
            world_path = world_var.get().strip()
            if not raw_name:
                error_label.config(text="World name cannot be empty")
                return
            if not model_path:
                error_label.config(text="Please select a model directory")
                return
            if not world_path:
                error_label.config(text="Please select a world SDF directory")
                return
            result[0] = self._convert_world_name(raw_name)
            result[1] = model_path
            result[2] = world_path
            dialog.destroy()

        def on_cancel(event=None):
            dialog.destroy()

        btn_frame = ttk.Frame(dialog)
        btn_frame.pack(pady=10)
        ttk.Button(btn_frame, text="Generate", command=on_generate).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="Cancel", command=on_cancel).pack(side=tk.LEFT, padx=5)

        dialog.bind("<Return>", on_generate)
        dialog.bind("<Escape>", on_cancel)

        dialog.wait_window()
        return result[0], result[1], result[2]

    def _show_generating_splash(self):
        """Show a small non-closeable splash with an animated label while generating."""
        splash = tk.Toplevel(self)
        splash.title("Please Wait")
        splash.resizable(False, False)
        splash.transient(self)
        splash.protocol("WM_DELETE_WINDOW", lambda: None)

        # Center on parent
        self.update_idletasks()
        px = self.winfo_x() + self.winfo_width() // 2 - 130
        py = self.winfo_y() + self.winfo_height() // 2 - 50
        splash.geometry(f"260x100+{px}+{py}")

        anim_var = tk.StringVar(value="Generating.")
        ttk.Label(splash, textvariable=anim_var, font=("TkDefaultFont", 11)).pack(expand=True)

        dots = [1]

        def animate():
            if not splash.winfo_exists():
                return
            dots[0] = dots[0] % 3 + 1
            anim_var.set("Generating" + "." * dots[0])
            splash.after(500, animate)

        splash.after(500, animate)
        splash.update()
        return splash

    def generate_world(self):
        """Generate the world using the map_info and wall height"""
        if not self.current_image:
            self._show_result_dialog("Info", "Please load an image first.")
            return

        if self.__map_info["resolution"] is None:
            self._show_result_dialog("Info", "Please set meters per pixel first.")
            return

        # Get the wall height from the input field
        wall_height = self.wall_height_var.get()

        # Show save dialog, pre-fill with the loaded map/image name
        default_name = self.__map_info.get("map_name") or ""
        world_name, model_dir, world_dir = self._ask_world_save_dialog(default_name=default_name)
        if world_name is None:
            return

        # Prepare map_info copy with the chosen world name
        map_info_copy = dict(self.__map_info)
        map_info_copy["map_name"] = world_name

        # Show splash and disable button
        self.generate_button.config(state=tk.DISABLED)
        splash = self._show_generating_splash()

        def run_generation():
            try:
                process_maps([map_info_copy], model_dir, world_dir, wall_height)
                self.after(0, lambda: _on_done(True, None))
            except Exception as e:
                self.after(0, lambda: _on_done(False, str(e)))

        def _on_done(success, error_msg):
            splash.destroy()
            self.generate_button.config(state=tk.NORMAL)
            if success:
                world_file = os.path.join(world_dir, f'{world_name}.sdf')
                self._show_result_dialog(
                    "Success",
                    f"World '{world_name}' generated!\n\nModel: {model_dir}\nWorld SDF: {world_file}"
                )
            else:
                self._show_result_dialog("Error", f"Generation failed:\n{error_msg}", is_error=True)

        threading.Thread(target=run_generation, daemon=True).start()

    def on_resize(self, event):
        """Handle window resize events"""
        if self.current_image:
            # Redisplay the image to fit the new canvas size
            self.after(100, self.display_image)

def main():
    app = MapImageProcessor()
    # Bind resize event
    app.bind("<Configure>", app.on_resize)
    app.mainloop()


if __name__ == "__main__":
    main()
