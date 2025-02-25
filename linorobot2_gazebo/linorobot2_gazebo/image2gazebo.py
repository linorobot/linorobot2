import tkinter as tk
from tkinter import ttk, filedialog, simpledialog
from PIL import Image, ImageTk
import os
import math
import numpy as np
from map2gazebo import process_maps


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
        self.wall_height_var = tk.DoubleVar(value=1.0)
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
        
    def load_image(self):
        """Load an image file and display it on the canvas"""
        file_types = [
            ("Image files", "*.png *.jpg *.jpeg *.bmp *.gif *.tiff"),
            ("All files", "*.*")
        ]
        file_path = filedialog.askopenfilename(title="Select an image file", filetypes=file_types)
        
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
                tk.messagebox.showerror("Error", f"Failed to load image: {str(e)}")
    
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
            
            # Draw crosshair marker
            size = 10
            self.canvas.create_line(canvas_x-size, canvas_y, canvas_x+size, canvas_y, 
                                  fill="blue", width=2, tags="origin_marker")
            self.canvas.create_line(canvas_x, canvas_y-size, canvas_x, canvas_y+size, 
                                  fill="blue", width=2, tags="origin_marker")
            self.canvas.create_oval(canvas_x-3, canvas_y-3, canvas_x+3, canvas_y+3, 
                                  fill="blue", outline="white", tags="origin_marker")
    
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
            tk.messagebox.showinfo("Info", "Please load an image first.")
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
            real_distance = simpledialog.askfloat("Distance", 
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
            tk.messagebox.showinfo("Info", "Please load an image first.")
            return
        
        if self.__map_info["resolution"] is None:
            tk.messagebox.showinfo("Info", "Please set meters per pixel first.")
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
        
        # Draw crosshair marker at the clicked position
        size = 10
        self.canvas.create_line(canvas_x-size, canvas_y, canvas_x+size, canvas_y, 
                               fill="blue", width=2, tags="origin_marker")
        self.canvas.create_line(canvas_x, canvas_y-size, canvas_x, canvas_y+size, 
                               fill="blue", width=2, tags="origin_marker")
        self.canvas.create_oval(canvas_x-3, canvas_y-3, canvas_x+3, canvas_y+3, 
                               fill="blue", outline="white", tags="origin_marker")
        
        self.status_bar.config(text=f"Origin set at world coordinates: [{world_x:.2f}, {world_y:.2f}, 0.0]")
        self.click_mode = None
    
    def generate_world(self):
        """Generate the world using the map_info and wall height"""
        if not self.current_image:
            tk.messagebox.showinfo("Info", "Please load an image first.")
            return
        
        if self.__map_info["resolution"] is None:
            tk.messagebox.showinfo("Info", "Please set meters per pixel first.")
            return
        
        # Get the wall height from the input field
        wall_height = self.wall_height_var.get()
        
        # Get script directory
        script_dir = os.path.dirname(os.path.abspath(__file__))
        
        # Prepare parameters for the process_maps function
        map_info_list = [self.__map_info]
        model_dir = script_dir
        world_dir = script_dir
        
        # Call the process_maps function
        process_maps(map_info_list, model_dir, world_dir, wall_height)
        
        tk.messagebox.showinfo("Success", "World generation complete!")
    
    def on_resize(self, event):
        """Handle window resize events"""
        if self.current_image:
            # Redisplay the image to fit the new canvas size
            self.after(100, self.display_image)

if __name__ == "__main__":
    app = MapImageProcessor()
    # Bind resize event
    app.bind("<Configure>", app.on_resize)
    app.mainloop()
