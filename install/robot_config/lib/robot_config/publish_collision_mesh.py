#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
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
        self.publisher_ = self.create_publisher(CollisionObject, 'collision_object', 10)
        self.timer = self.create_timer(1.0, self.publish_collision_mesh)

    def publish_collision_mesh(self):
        # Get the field parameter
        field = self.get_parameter('field').get_parameter_value().string_value
        self.get_logger().info(f'Field parameter: {field}')

        collision_object = CollisionObject()
        collision_object.header.frame_id = "world"
        collision_object.id = "base_link_stl"

        mesh_path = "/home/a/ws_moveit2/src/field_description-20250822T021318Z-1-001/field_description/meshes/base_link.stl"

        try:
            your_mesh = mesh.Mesh.from_file(mesh_path)
        except Exception as e:
            self.get_logger().error(f"Failed to load mesh file: {e}")
            return

        scale = 0.001  # ここで倍率を指定します

        # Determine rotation angle based on field parameter
        rotation_angle = 0.0
        if field == 'red':
            rotation_angle = math.radians(0)   # No rotation for red
        elif field == 'blue':
            rotation_angle = math.radians(180) # 180 degrees rotation for blue
        else:
            rotation_angle = 0.0  # Default no rotation

        self.get_logger().info(f'Applying rotation of {math.degrees(rotation_angle)} degrees')

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
            self.get_logger().info(f'Mesh bounding box center: ({center_x}, {center_y}, {center_z})')
        else:
            center_x = center_y = center_z = 0.0

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
                
                # Translate back
                final_x = rotated_x + center_x
                final_y = rotated_y + center_y
                final_z = rotated_z + center_z
                
                p = Point()
                p.x = final_x
                p.y = final_y
                p.z = final_z
                shape_mesh.vertices.append(p)

        for i in range(len(your_mesh.vectors)):
            triangle = MeshTriangle()
            triangle.vertex_indices = [i*3, i*3+1, i*3+2]
            shape_mesh.triangles.append(triangle)

        pose = Pose()
        pose.position.x = 0.0
        pose.position.y = 0.0
        pose.position.z = 0.0
        pose.orientation.w = 1.0

        collision_object.meshes.append(shape_mesh)
        collision_object.mesh_poses.append(pose)
        collision_object.operation = CollisionObject.ADD

        self.publisher_.publish(collision_object)
        self.get_logger().info('Publishing collision mesh')
        self.timer.cancel() # Publish only once

def main(args=None):
    rclpy.init(args=args)
    node = CollisionMeshPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()