import cv2
import numpy as np
import trimesh
from matplotlib.tri import Triangulation
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

class MapConverter:
    def __init__(self, map_dir, export_dir, world_dir, height=1.5):
        self.height = height
        self.export_dir = export_dir
        self.world_dir = world_dir
        self.map_dir = map_dir

    def map_callback(self):
        all_maps = self._extract_maps(self.map_dir)
        for key, value in all_maps.items():
            map_file_dir = value[0] if ".pgm" in value[0] or ".png" in value[0] else value[1]
            info_dir = value[0] if ".yaml" in value[0] else value[1]
            map_array = cv2.imread(map_file_dir)
            map_array = cv2.flip(map_array, 0)
            print(f'loading map file: {map_file_dir}')
            try:
                map_array = cv2.cvtColor(map_array, cv2.COLOR_BGR2GRAY)
            except cv2.error as err:
                print(err, "Conversion failed: Invalid image input, please check your file path")    
                sys.exit()

            with open(info_dir, 'r') as stream:
                map_info = yaml.safe_load(stream)
            
            # set all -1 (unknown) values to 255 (white/unoccupied)
            map_array[map_array < 0] = 255
            
            print('Processing...')
            mesh = self.create_mesh_from_map(map_array, map_info)

            if not self.export_dir.endswith('/'):
                self.export_dir = self.export_dir + '/'

            if not self.world_dir.endswith('/'):
                self.world_dir = self.world_dir + '/'
            
            if not os.path.exists(self.export_dir + f'{key}/meshes/'):
                os.makedirs(self.export_dir + f'{key}/meshes/')

            if not os.path.exists(self.world_dir):
                os.makedirs(self.world_dir)

            stl_dir = self.export_dir + f'{key}/meshes/' + f'{key}.stl'
            sdf_dir = self.export_dir + f'{key}/' + f'{key}.sdf'
            config_dir = self.export_dir + f'{key}/model.config'

            model_template = XML_MODEL_TEMPLATE.format(name=key)
            sdf_data = XML_SDF_TEMPLATE.format(model_template=model_template)
            config_data = XML_MODEL_CONFIG_TEMPLATE.format(name=key)
            print(f'export file: {stl_dir}')
            
            with open(stl_dir, 'wb') as f:
                mesh.export(f, "stl")

            with open(sdf_dir, 'w') as f:
                f.write(sdf_data)

            with open(config_dir, 'w') as f:
                f.write(config_data)

            # create world file
            world_data = XML_WORLD_TEMPLATE.format(model_template=model_template)

            world_dir = self.world_dir + f'{key}.sdf'
            with open(world_dir, 'w') as f:
                f.write(world_data)

    def _extract_maps(self, directory_path):
        files_dict = {}

        for filename in os.listdir(directory_path):
            base_name, extension = os.path.splitext(filename)
            
            if extension in ['.pgm', '.yaml', '.png']:
                if base_name in files_dict:
                    files_dict[base_name].append(os.path.join(directory_path, filename))
                else:
                    files_dict[base_name] = [os.path.join(directory_path, filename)]
        
        return files_dict

    def create_mesh_from_map(self, map_array, metadata):
        height = np.array([0, 0, self.height])
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
                        self.coords_to_loc((x, y), metadata),
                        self.coords_to_loc((x, y+step), metadata),
                        self.coords_to_loc((x+step, y), metadata),
                        self.coords_to_loc((x+step, y+step), metadata)
                    ]
                    vertices.extend(new_vertices)
                    vertices.extend([v + height for v in new_vertices])

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

    def coords_to_loc(self, coords, metadata):
        x, y = coords
        loc_x = x * metadata['resolution'] + metadata['origin'][0]
        loc_y = y * metadata['resolution'] + metadata['origin'][1]
        return np.array([loc_x, loc_y, 0.0])

if __name__ == "__main__":
    parser = argparse.ArgumentParser(argument_default=argparse.SUPPRESS)
    parser.add_argument(
      '--map_dir', type=str, required=True,
      help='File name of the map to convert'
    )

    parser.add_argument(
      '--model_dir', type=str, default=os.path.abspath('.'),
      help='Gazebo model output directory'
    )

    parser.add_argument(
      '--world_dir', type=str, default=os.path.abspath('.'),
      help='World output directory'
    )

    option = parser.parse_args()

    Converter = MapConverter(option.map_dir, option.model_dir, option.world_dir)
    Converter.map_callback()
    print('Conversion Done')
