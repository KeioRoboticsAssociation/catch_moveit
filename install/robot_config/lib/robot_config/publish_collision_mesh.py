#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import Mesh, MeshTriangle, SolidPrimitive
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Header
import numpy as np
from stl import mesh
import math
import time

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
        # Parameter for box objects: list of 8 points [x1,y1,z1, x2,y2,z2, ..., x8,y8,z8]
        # Creates box primitives from 8 corner coordinates
        self.declare_parameter(
            'box_coordinates',
            Parameter.Type.DOUBLE_ARRAY
        )
        # Use reliable QoS to ensure all messages are delivered
        qos_profile = QoSProfile(
            depth=200,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.publisher_ = self.create_publisher(CollisionObject, 'collision_object', qos_profile)
        self.object_index = 0
        self.all_positions = []
        self.field_published = False
        self.object_timer = None
        self.timer = self.create_timer(3.0, self.publish_collision_mesh)

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
    
    def calculate_box_from_8_points(self, coordinates):
        """Calculate box center, dimensions, and orientation from 8 corner points"""
        if len(coordinates) != 24:  # 8 points * 3 coordinates each
            self.get_logger().error('Box coordinates must contain exactly 24 values (8 points with x,y,z each)')
            return None
            
        # Extract 8 points
        points = []
        for i in range(0, 24, 3):
            points.append([coordinates[i], coordinates[i+1], coordinates[i+2]])
        
        # Convert to numpy array for easier calculation
        points_array = np.array(points)
        
        # Calculate center (average of all points)
        center = np.mean(points_array, axis=0)
        
        # Calculate bounding box dimensions
        min_coords = np.min(points_array, axis=0)
        max_coords = np.max(points_array, axis=0)
        dimensions = max_coords - min_coords
        
        return {
            'center': center,
            'dimensions': dimensions
        }
    
    def create_plane_mesh(self, coordinates):
        """Create a planar mesh from 4 coordinate points"""
        if len(coordinates) != 12:  # 4 points * 3 coordinates each
            self.get_logger().error('Plane coordinates must contain exactly 12 values (4 points with x,y,z each)')
            return None
            
        # Extract 4 points
        points = []
        for i in range(0, 12, 3):
            points.append([coordinates[i], coordinates[i+1], coordinates[i+2]])
        
        shape_mesh = Mesh()
        
        # Add vertices
        for point in points:
            p = Point()
            p.x = float(point[0])
            p.y = float(point[1]) 
            p.z = float(point[2])
            shape_mesh.vertices.append(p)
        
        # Create two triangles to form a rectangular plane with proper winding order
        # Ensure counter-clockwise winding for correct normals
        # Triangle 1: points 0, 1, 2 (counter-clockwise)
        triangle1 = MeshTriangle()
        triangle1.vertex_indices = [0, 1, 2]
        shape_mesh.triangles.append(triangle1)
        
        # Triangle 2: points 0, 2, 3 (counter-clockwise)
        triangle2 = MeshTriangle()
        triangle2.vertex_indices = [0, 2, 3]
        shape_mesh.triangles.append(triangle2)
        
        # Add reverse triangles for double-sided plane (visible from both sides)
        triangle3 = MeshTriangle()
        triangle3.vertex_indices = [0, 2, 1]  # Reverse winding
        shape_mesh.triangles.append(triangle3)
        
        triangle4 = MeshTriangle()
        triangle4.vertex_indices = [0, 3, 2]  # Reverse winding
        shape_mesh.triangles.append(triangle4)
        
        return shape_mesh
    
    def remove_all_collision_objects(self):
        """Remove all existing collision objects"""
        self.get_logger().info("Removing all existing collision objects")
        
        # Remove field mesh
        collision_object = CollisionObject()
        collision_object.header.frame_id = "world"
        collision_object.id = "field_mesh_0"
        collision_object.operation = CollisionObject.REMOVE
        self.publisher_.publish(collision_object)
        
        # Remove up to 100 object meshes (safe upper bound)
        for i in range(1, 101):
            collision_object = CollisionObject()
            collision_object.header.frame_id = "world"
            collision_object.id = f"object_mesh_{i}"
            collision_object.operation = CollisionObject.REMOVE
            self.publisher_.publish(collision_object)
        
        # Remove box objects
        for i in range(200):
            collision_object = CollisionObject()
            collision_object.header.frame_id = "world"
            collision_object.id = f"box_from_8points_{i}"
            collision_object.operation = CollisionObject.REMOVE
            self.publisher_.publish(collision_object)

    def publish_collision_mesh(self):
        # Get parameters
        field = self.get_parameter('field').get_parameter_value().string_value
        field_mesh_path = self.get_parameter('field_mesh_path').get_parameter_value().string_value
        object_mesh_path = self.get_parameter('object_mesh_path').get_parameter_value().string_value
        object_mesh_positions = self.get_parameter('object_mesh_positions').get_parameter_value().double_array_value
        try:
            box_coordinates = self.get_parameter('box_coordinates').get_parameter_value().double_array_value
        except:
            box_coordinates = []
        
        self.get_logger().info(f'Field parameter: {field}')
        self.get_logger().info(f'Field mesh path: {field_mesh_path}')
        self.get_logger().info(f'Object mesh path: {object_mesh_path}')
        self.get_logger().info(f'Object mesh positions array length: {len(object_mesh_positions)}')
        
        scale = 0.001
        mesh_index = 0

        # 1. Publish field mesh (without rotation)
        if field_mesh_path:
            your_mesh, center = self.load_mesh_and_calculate_center(field_mesh_path, scale)
            if your_mesh is not None and center is not None:
                collision_object = CollisionObject()
                collision_object.header.frame_id = "world"
                collision_object.id = f"field_mesh_{mesh_index}"
                mesh_index += 1

                shape_mesh = self.create_mesh_with_pose(your_mesh, center, scale, 0.0)
                
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
                
                # Small delay to ensure message is sent
                time.sleep(0.1)

        # 2. Publish object meshes (multiple instances with positions)
        if object_mesh_path and object_mesh_positions:
            # Reshape object_mesh_positions into list of [x, y, z, yaw_degrees]
            if len(object_mesh_positions) % 4 != 0:
                self.get_logger().error('object_mesh_positions parameter must contain groups of 4 values [x, y, z, yaw_degrees]')
                return
            
            # Limit to 100 objects
            max_objects = 120
            num_objects = len(object_mesh_positions) // 4
            if num_objects > max_objects:
                self.get_logger().warn(f'Too many object positions specified ({num_objects}). Limiting to {max_objects}.')
                num_objects = max_objects
            
            positions = []
            for i in range(0, num_objects * 4, 4):
                positions.append(object_mesh_positions[i:i+4])
            
            your_mesh, center = self.load_mesh_and_calculate_center(object_mesh_path, scale)
            if your_mesh is not None and center is not None:
                self.get_logger().info(f'Publishing {len(positions)} object meshes')
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
                    self.get_logger().info(f'Publishing object collision mesh {idx} (ID: {collision_object.id}) at position ({px}, {py}, {pz}) with yaw {yaw_deg} degrees')
                    
                    # Small delay between object meshes
                    time.sleep(0.75)

        # 3. Publish box primitive objects from 8 corner coordinates
        if box_coordinates:
            # Each box needs 24 values (8 points × 3 coordinates)
            if len(box_coordinates) % 24 != 0:
                self.get_logger().error('box_coordinates parameter must contain groups of 24 values (8 points with x,y,z each)')
            else:
                num_boxes = len(box_coordinates) // 24
                # Limit to 50 box objects
                max_boxes = 50
                if num_boxes > max_boxes:
                    self.get_logger().warn(f'Too many box coordinates specified ({num_boxes}). Limiting to {max_boxes}.')
                    num_boxes = max_boxes
                
                for i in range(num_boxes):
                    start_idx = i * 24
                    box_coords = box_coordinates[start_idx:start_idx + 24]
                    
                    box_info = self.calculate_box_from_8_points(box_coords)
                    if box_info is not None:
                        collision_object = CollisionObject()
                        collision_object.header.frame_id = "world"
                        collision_object.id = f"box_from_8points_{mesh_index}"
                        mesh_index += 1
                        
                        # Create box primitive
                        box_primitive = SolidPrimitive()
                        box_primitive.type = SolidPrimitive.BOX
                        box_primitive.dimensions = [float(box_info['dimensions'][0]), 
                                                   float(box_info['dimensions'][1]), 
                                                   float(box_info['dimensions'][2])]
                        
                        # Set pose (center position)
                        pose = Pose()
                        pose.position.x = float(box_info['center'][0])
                        pose.position.y = float(box_info['center'][1])
                        pose.position.z = float(box_info['center'][2])
                        pose.orientation.w = 1.0
                        
                        collision_object.primitives.append(box_primitive)
                        collision_object.primitive_poses.append(pose)
                        collision_object.operation = CollisionObject.ADD
                        
                        self.publisher_.publish(collision_object)
                        self.get_logger().info(f'Publishing box from 8 points {i} at ({box_info["center"][0]:.3f}, {box_info["center"][1]:.3f}, {box_info["center"][2]:.3f}) with dimensions ({box_info["dimensions"][0]:.3f}, {box_info["dimensions"][1]:.3f}, {box_info["dimensions"][2]:.3f})')
                        
                        # Small delay between box objects
                        time.sleep(0.05)

        # Ensure all messages are sent before canceling timer
        time.sleep(0.1)
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