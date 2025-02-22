import cv2
import numpy as np
import trimesh
import yaml
import argparse
import os
import sys


XML_MODEL_CONFIG_TEMPLATE = """
<?xml version="1.0" ?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.5">{name}.sdf</sdf>
  <author>
    <name>your name</name>
    <email>youremail.com</email>
  </author>
  <description></description>
</model>
"""

XML_MODEL_TEMPLATE = """
    <model name="{name}">
      <link name="link">
        <inertial>
          <mass>15</mass>
          <inertia>
            <ixx>0.0</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>0.0</iyy>
            <iyz>0.0</iyz>
            <izz>0.0</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <pose>0 0 0 0 0 0</pose>
          <geometry>
            <mesh>
              <uri>model://{name}/meshes/{name}.stl</uri>
            </mesh>
          </geometry>
        </collision>
        <visual name="visual">
          <pose>0 0 0 0 0 0</pose>
          <geometry>
            <mesh>
              <uri>model://{name}/meshes/{name}.stl</uri>
            </mesh>
          </geometry>
          <material>
            <ambient>1 1 1 1</ambient>
            <diffuse>1 1 1 1</diffuse>
            <specular>0.5 0.5 0.5 1</specular>
            <emissive>0 0 0 1</emissive>
          </material>
        </visual>
      </link>
      <static>1</static>
    </model>
"""

XML_SDF_TEMPLATE = """
<?xml version="1.0" ?>
<sdf version="1.4">
  {model_template}
</sdf>
"""


XML_WORLD_TEMPLATE="""
<sdf version="1.8">
  <world name="playground">

    <gravity>0 0 -9.8</gravity>
    <physics default="0" name="default_physics" type="ode">
      <max_step_size>0.01</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>100</real_time_update_rate>
    </physics>

    <plugin
      filename="gz-sim-physics-system"
      name="gz::sim::systems::Physics">
    </plugin>
    <plugin
      filename="gz-sim-user-commands-system"
      name="gz::sim::systems::UserCommands">
    </plugin>
    <plugin
      filename="gz-sim-scene-broadcaster-system"
      name="gz::sim::systems::SceneBroadcaster">
    </plugin>

    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>

    {model_template}
    
  </world>
</sdf>
"""

def create_mesh_from_map(map_array, metadata, height=1.5):
    height_vector = np.array([0, 0, height])
    vertices = []
    faces = []
    vertex_count = 0

    thresh_map = map_array.copy()

    # Apply the thresholds
    thresh_map[map_array >= metadata["occupied_thresh"] * 255] = 255  # Occupied cells
    thresh_map[map_array <= metadata["free_thresh"] * 255] = 0  # Free cells
    thresh_map[(map_array > metadata["free_thresh"] * 255) & (map_array < metadata["occupied_thresh"] * 255)] = 127  # Unknown cells

    # Reduce resolution to simplify the mesh
    step = 1  # Adjust this value to change the simplification level

    for y in range(0, thresh_map.shape[0] - 1, step):
        for x in range(0, thresh_map.shape[1] - 1, step):
            if thresh_map[y, x] == 0:  # If the pixel is black (occupied)
                new_vertices = [
                    coords_to_loc((x, y), metadata),
                    coords_to_loc((x, y+step), metadata),
                    coords_to_loc((x+step, y), metadata),
                    coords_to_loc((x+step, y+step), metadata)
                ]
                vertices.extend(new_vertices)
                vertices.extend([v + height_vector for v in new_vertices])

                new_faces = [
                    [vertex_count + i for i in face]
                    for face in [[0, 2, 4], [4, 2, 6], [1, 2, 0], [3, 2, 1],
                                [5, 0, 4], [1, 0, 5], [3, 7, 2], [7, 6, 2],
                                [7, 4, 6], [5, 4, 7], [1, 5, 3], [7, 3, 5]]
                ]
                faces.extend(new_faces)
                vertex_count += 8

    mesh = trimesh.Trimesh(vertices=vertices, faces=faces)
    if not mesh.is_volume:
        mesh.fix_normals()
    
    mesh.update_faces(mesh.unique_faces())
    
    return mesh

def coords_to_loc(coords, metadata):
    x, y = coords
    loc_x = x * metadata['resolution'] + metadata['origin'][0]
    loc_y = y * metadata['resolution'] + metadata['origin'][1]
    return np.array([loc_x, loc_y, 0.0])

def process_map(map_info, export_dir, world_dir, height=1.5):
    # Check if the required keys exist in the map info dictionary
    if 'map_name' not in map_info or 'image' not in map_info:
        print(f"Error: Map info missing required keys 'map_name' or 'image'")
        return False
        
    map_name = map_info['map_name']
    image_path = map_info['image']
    
    # Check if the image file exists
    if not os.path.exists(image_path):
        print(f"Error: Image file {image_path} not found")
        return False
        
    print(f'Loading map file: {image_path}')
    
    try:
        map_array = cv2.imread(image_path)
        map_array = cv2.flip(map_array, 0)
        map_array = cv2.cvtColor(map_array, cv2.COLOR_BGR2GRAY)
    except cv2.error as err:
        print(err, "Conversion failed: Invalid image input, please check your file path")    
        return False

    # Set all -1 (unknown) values to 255 (white/unoccupied)
    # map_array[map_array < 0] = 255
    map_array[map_array < 253] = 0
    map_array[map_array >= 253] = 255
    print('Processing...')
    mesh = create_mesh_from_map(map_array, map_info, height)

    if not export_dir.endswith('/'):
        export_dir = export_dir + '/'

    if not world_dir.endswith('/'):
        world_dir = world_dir + '/'
    
    model_dir = export_dir + f'{map_name}'
    meshes_dir = model_dir + '/meshes/'
    
    if not os.path.exists(meshes_dir):
        os.makedirs(meshes_dir)

    if not os.path.exists(world_dir):
        os.makedirs(world_dir)

    stl_dir = meshes_dir + f'{map_name}.stl'
    sdf_dir = model_dir + f'/{map_name}.sdf'
    config_dir = model_dir + '/model.config'

    model_template = XML_MODEL_TEMPLATE.format(name=map_name)
    sdf_data = XML_SDF_TEMPLATE.format(model_template=model_template)
    config_data = XML_MODEL_CONFIG_TEMPLATE.format(name=map_name)
    print(f'Exporting to file: {stl_dir}')
    
    with open(stl_dir, 'wb') as f:
        mesh.export(f, "stl")

    with open(sdf_dir, 'w') as f:
        f.write(sdf_data)

    with open(config_dir, 'w') as f:
        f.write(config_data)

    # Create world file
    world_data = XML_WORLD_TEMPLATE.format(model_template=model_template)

    world_file = world_dir + f'{map_name}.sdf'
    with open(world_file, 'w') as f:
        f.write(world_data)
    
    print(f'Successfully processed map: {map_name}')
    return True

def process_maps(map_info_list, export_dir, world_dir, height=1.5):
    success_count = 0
    fail_count = 0
    
    for map_info in map_info_list:
        if process_map(map_info, export_dir, world_dir, height):
            success_count += 1
        else:
            fail_count += 1
    
    print(f'Conversion completed. Success: {success_count}, Failed: {fail_count}')
    return success_count, fail_count

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    
    parser.add_argument(
        '--map_dir', type=str, required=True,
        help='Directory containing YAML map files'
    )
    
    parser.add_argument(
        '--model_dir', type=str, default=os.path.abspath('.'),
        help='Gazebo model output directory'
    )
    
    parser.add_argument(
        '--world_dir', type=str, default=os.path.abspath('.'),
        help='World output directory'
    )
    
    parser.add_argument(
        '--height', type=float, default=1.5,
        help='Height of the 3D map mesh'
    )
    
    args = parser.parse_args()
    
    # Check if map_dir exists
    if not os.path.isdir(args.map_dir):
        print(f"Error: Map directory {args.map_dir} not found or is not a directory.")
        sys.exit(1)
    
    # Find all YAML files in the map_dir
    yaml_files = []
    for file in os.listdir(args.map_dir):
        if file.lower().endswith('.yaml') or file.lower().endswith('.yml'):
            yaml_files.append(os.path.join(args.map_dir, file))
    
    if not yaml_files:
        print(f"Error: No YAML files found in {args.map_dir}")
        sys.exit(1)
    
    print(f"Found {len(yaml_files)} YAML files in {args.map_dir}")
    
    # Process the list of YAML files and create map_info_list
    map_info_list = []
    for yaml_file in yaml_files:
        try:
            with open(yaml_file, 'r') as stream:
                map_info = yaml.safe_load(stream)
                
                # Add map_name based on the YAML filename
                map_name = os.path.splitext(os.path.basename(yaml_file))[0]
                map_info['map_name'] = map_name
                
                # Make image path absolute if it's relative
                if not os.path.isabs(map_info['image']):
                    yaml_dir = os.path.dirname(os.path.abspath(yaml_file))
                    map_info['image'] = os.path.join(yaml_dir, map_info['image'])
                
                map_info_list.append(map_info)
                print(f"Added map: {map_name}")
        except Exception as e:
            print(f"Error loading YAML file {yaml_file}: {str(e)}")
    
    if not map_info_list:
        print("No valid map files found. Exiting.")
        sys.exit(1)
    
    # Process all maps
    process_maps(map_info_list, args.model_dir, args.world_dir, args.height)
