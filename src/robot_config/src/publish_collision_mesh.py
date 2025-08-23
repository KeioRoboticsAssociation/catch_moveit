#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import Mesh, MeshTriangle
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Header
import numpy as np
from stl import mesh
import math

class CollisionMeshPublisher(Node):
    def __init__(self):
        super().__init__('collision_mesh_publisher')
        self.declare_parameter('field', 'none')
        self.declare_parameter('field_mesh_path', '/home/a/ws_moveit2/src/field_description-20250822T021318Z-1-001/field_description/meshes/base_link.stl')
        self.declare_parameter('object_mesh_path', '')
        # Parameter for multiple object mesh positions: list of [x, y, z, yaw_degrees]
        # Example: [[0.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 90.0]]
        self.declare_parameter(
            'object_mesh_positions', 
            Parameter.Type.DOUBLE_ARRAY
        )
        self.publisher_ = self.create_publisher(CollisionObject, 'collision_object', 10)
        self.timer = self.create_timer(1.0, self.publish_collision_mesh)

    def load_mesh_and_calculate_center(self, mesh_path, scale=0.001):
        """Load mesh and calculate its bounding box center"""
        try:
            your_mesh = mesh.Mesh.from_file(mesh_path)
        except Exception as e:
            self.get_logger().error(f"Failed to load mesh file {mesh_path}: {e}")
            return None, None
            
        # Calculate bounding box center
        all_vertices = []
        for v in your_mesh.vectors:
            for vertex in v:
                # Apply scaling
                x = float(vertex[0]) * scale
                y = float(vertex[1]) * scale
                z = float(vertex[2]) * scale
                all_vertices.append([x, y, z])
        
        if all_vertices:
            vertices_array = np.array(all_vertices)
            min_x, max_x = np.min(vertices_array[:, 0]), np.max(vertices_array[:, 0])
            min_y, max_y = np.min(vertices_array[:, 1]), np.max(vertices_array[:, 1])
            min_z, max_z = np.min(vertices_array[:, 2]), np.max(vertices_array[:, 2])
            
            # Center is the middle of the bounding box
            center_x = (min_x + max_x) / 2.0
            center_y = (min_y + max_y) / 2.0
            center_z = (min_z + max_z) / 2.0
            self.get_logger().info(f'Mesh {mesh_path} bounding box center: ({center_x}, {center_y}, {center_z})')
            return your_mesh, (center_x, center_y, center_z)
        else:
            return your_mesh, (0.0, 0.0, 0.0)

    def create_mesh_with_pose(self, your_mesh, center, scale, rotation_angle, position=(0.0, 0.0, 0.0)):
        """Create a Mesh object with specified rotation and position"""
        center_x, center_y, center_z = center
        px, py, pz = position
        
        # Rotation matrix for Z-axis
        cos_theta = math.cos(rotation_angle)
        sin_theta = math.sin(rotation_angle)
        
        shape_mesh = Mesh()
        for v in your_mesh.vectors:
            for vertex in v:
                # Apply scaling first
                x = float(vertex[0]) * scale
                y = float(vertex[1]) * scale
                z = float(vertex[2]) * scale
                
                # Translate to origin (center as origin)
                translated_x = x - center_x
                translated_y = y - center_y
                translated_z = z - center_z
                
                # Apply Z-axis rotation around mesh center
                rotated_x = translated_x * cos_theta - translated_y * sin_theta
                rotated_y = translated_x * sin_theta + translated_y * cos_theta
                rotated_z = translated_z
                
                # Translate back and then to final position
                final_x = rotated_x + center_x + px
                final_y = rotated_y + center_y + py
                final_z = rotated_z + center_z + pz
                
                p = Point()
                p.x = final_x
                p.y = final_y
                p.z = final_z
                shape_mesh.vertices.append(p)

        for i in range(len(your_mesh.vectors)):
            triangle = MeshTriangle()
            triangle.vertex_indices = [i*3, i*3+1, i*3+2]
            shape_mesh.triangles.append(triangle)
            
        return shape_mesh

    def publish_collision_mesh(self):
        # Get parameters
        field = self.get_parameter('field').get_parameter_value().string_value
        field_mesh_path = self.get_parameter('field_mesh_path').get_parameter_value().string_value
        object_mesh_path = self.get_parameter('object_mesh_path').get_parameter_value().string_value
        object_mesh_positions = self.get_parameter('object_mesh_positions').get_parameter_value().double_array_value
        
        self.get_logger().info(f'Field parameter: {field}')
        self.get_logger().info(f'Field mesh path: {field_mesh_path}')
        self.get_logger().info(f'Object mesh path: {object_mesh_path}')
        
        scale = 0.001
        mesh_index = 0

        # 1. Publish field mesh (with rotation based on field parameter)
        if field_mesh_path:
            your_mesh, center = self.load_mesh_and_calculate_center(field_mesh_path, scale)
            if your_mesh is not None and center is not None:
                # Determine rotation angle based on field parameter
                rotation_angle = 0.0
                if field == 'red':
                    rotation_angle = math.radians(0)   # No rotation for red
                elif field == 'blue':
                    rotation_angle = math.radians(180) # 180 degrees rotation for blue
                else:
                    rotation_angle = 0.0  # Default no rotation

                self.get_logger().info(f'Applying rotation of {math.degrees(rotation_angle)} degrees to field mesh')

                collision_object = CollisionObject()
                collision_object.header.frame_id = "world"
                collision_object.id = f"field_mesh_{mesh_index}"
                mesh_index += 1

                shape_mesh = self.create_mesh_with_pose(your_mesh, center, scale, rotation_angle)
                
                pose = Pose()
                pose.position.x = 0.0
                pose.position.y = 0.0
                pose.position.z = 0.0
                pose.orientation.w = 1.0

                collision_object.meshes.append(shape_mesh)
                collision_object.mesh_poses.append(pose)
                collision_object.operation = CollisionObject.ADD

                self.publisher_.publish(collision_object)
                self.get_logger().info('Publishing field collision mesh')

        # 2. Publish object meshes (multiple instances with positions)
        if object_mesh_path and object_mesh_positions:
            # Reshape object_mesh_positions into list of [x, y, z, yaw_degrees]
            if len(object_mesh_positions) % 4 != 0:
                self.get_logger().error('object_mesh_positions parameter must contain groups of 4 values [x, y, z, yaw_degrees]')
                return
            
            # Limit to 100 objects
            max_objects = 100
            num_objects = len(object_mesh_positions) // 4
            if num_objects > max_objects:
                self.get_logger().warn(f'Too many object positions specified ({num_objects}). Limiting to {max_objects}.')
                num_objects = max_objects
            
            positions = []
            for i in range(0, num_objects * 4, 4):
                positions.append(object_mesh_positions[i:i+4])
            
            your_mesh, center = self.load_mesh_and_calculate_center(object_mesh_path, scale)
            if your_mesh is not None and center is not None:
                for idx, (px, py, pz, yaw_deg) in enumerate(positions):
                    collision_object = CollisionObject()
                    collision_object.header.frame_id = "world"
                    collision_object.id = f"object_mesh_{mesh_index}"
                    mesh_index += 1

                    # Convert yaw from degrees to radians (no base rotation for objects)
                    yaw_rad = math.radians(yaw_deg)
                    
                    shape_mesh = self.create_mesh_with_pose(your_mesh, center, scale, yaw_rad, (px, py, pz))
                    
                    pose = Pose()
                    pose.position.x = 0.0
                    pose.position.y = 0.0
                    pose.position.z = 0.0
                    pose.orientation.w = 1.0

                    collision_object.meshes.append(shape_mesh)
                    collision_object.mesh_poses.append(pose)
                    collision_object.operation = CollisionObject.ADD

                    self.publisher_.publish(collision_object)
                    self.get_logger().info(f'Publishing object collision mesh {idx} at position ({px}, {py}, {pz}) with yaw {yaw_deg} degrees')

        self.get_logger().info('Finished publishing all collision meshes')
        self.timer.cancel() # Publish only once

def main(args=None):
    rclpy.init(args=args)
    node = CollisionMeshPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()