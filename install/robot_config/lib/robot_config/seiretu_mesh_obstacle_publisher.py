#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import Mesh, MeshTriangle
from geometry_msgs.msg import Pose, Point
import numpy as np

class SeiretuMeshObstaclePublisher(Node):
    def __init__(self):
        super().__init__('seiretu_mesh_obstacle_publisher')
        
        # Planning scene publisher
        self.planning_scene_pub = self.create_publisher(
            PlanningScene, 
            '/planning_scene', 
            10
        )
        
        # Parameters
        self.declare_parameter('blue_seiretu_position', [0.5, 0.5, 0.0])
        self.declare_parameter('red_seiretu_position', [1.0, 0.5, 0.0])
        self.declare_parameter('blue_mesh_path', '/home/a/ws_moveit2/src/blue_seiretu_description-20250907T041805Z-1-001/blue_seiretu_description/meshes/blue_seiretu_1.stl')
        self.declare_parameter('red_mesh_path', '/home/a/ws_moveit2/src/red_seiretu_description-20250907T041805Z-1-001/red_seiretu_description/meshes/red_seiretu_2.stl')
        
        # Wait for planning scene to be ready
        self.timer = self.create_timer(5.0, self.publish_obstacles)
        self.get_logger().info('Seiretu mesh obstacle publisher started')

    def load_mesh_from_stl(self, file_path):
        """Load mesh from STL file and convert to ROS Mesh message"""
        try:
            import trimesh
            mesh_data = trimesh.load(file_path)
            
            # Create ROS Mesh message
            ros_mesh = Mesh()
            
            # Add vertices
            for vertex in mesh_data.vertices:
                point = Point()
                point.x = float(vertex[0])
                point.y = float(vertex[1]) 
                point.z = float(vertex[2])
                ros_mesh.vertices.append(point)
            
            # Add triangles
            for face in mesh_data.faces:
                triangle = MeshTriangle()
                triangle.vertex_indices = [int(face[0]), int(face[1]), int(face[2])]
                ros_mesh.triangles.append(triangle)
                
            self.get_logger().info(f'Loaded mesh with {len(ros_mesh.vertices)} vertices and {len(ros_mesh.triangles)} triangles from {file_path}')
            return ros_mesh
            
        except Exception as e:
            self.get_logger().error(f'Failed to load mesh from {file_path}: {str(e)}')
            # Create a simple box mesh as fallback
            return self.create_box_mesh(0.2, 0.1, 0.3)

    def create_box_mesh(self, x_size, y_size, z_size):
        """Create a simple box mesh as fallback"""
        mesh = Mesh()
        
        # Box vertices
        vertices = [
            [-x_size/2, -y_size/2, -z_size/2],
            [x_size/2, -y_size/2, -z_size/2],
            [x_size/2, y_size/2, -z_size/2],
            [-x_size/2, y_size/2, -z_size/2],
            [-x_size/2, -y_size/2, z_size/2],
            [x_size/2, -y_size/2, z_size/2],
            [x_size/2, y_size/2, z_size/2],
            [-x_size/2, y_size/2, z_size/2]
        ]
        
        for vertex in vertices:
            point = Point()
            point.x = float(vertex[0])
            point.y = float(vertex[1])
            point.z = float(vertex[2])
            mesh.vertices.append(point)
        
        # Box faces (triangles)
        faces = [
            [0, 1, 2], [0, 2, 3],  # bottom
            [4, 7, 6], [4, 6, 5],  # top
            [0, 4, 5], [0, 5, 1],  # front
            [2, 6, 7], [2, 7, 3],  # back
            [0, 3, 7], [0, 7, 4],  # left
            [1, 5, 6], [1, 6, 2]   # right
        ]
        
        for face in faces:
            triangle = MeshTriangle()
            triangle.vertex_indices = face
            mesh.triangles.append(triangle)
            
        return mesh

    def create_mesh_collision_object(self, name, mesh, position):
        """Create a collision object from mesh"""
        collision_object = CollisionObject()
        collision_object.header.frame_id = "world"
        collision_object.header.stamp = self.get_clock().now().to_msg()
        collision_object.id = name
        
        # Set pose
        pose = Pose()
        pose.position.x = position[0]
        pose.position.y = position[1] 
        pose.position.z = position[2]
        pose.orientation.w = 1.0
        
        collision_object.meshes = [mesh]
        collision_object.mesh_poses = [pose]
        collision_object.operation = CollisionObject.ADD
        
        return collision_object

    def publish_obstacles(self):
        """Publish seiretu robots as mesh obstacles"""
        try:
            # Get parameters
            blue_pos = self.get_parameter('blue_seiretu_position').get_parameter_value().double_array_value
            red_pos = self.get_parameter('red_seiretu_position').get_parameter_value().double_array_value
            blue_mesh_path = self.get_parameter('blue_mesh_path').get_parameter_value().string_value
            red_mesh_path = self.get_parameter('red_mesh_path').get_parameter_value().string_value
            
            # Load meshes
            blue_mesh = self.load_mesh_from_stl(blue_mesh_path)
            red_mesh = self.load_mesh_from_stl(red_mesh_path)
            
            # Create collision objects
            blue_obstacle = self.create_mesh_collision_object("blue_seiretu_obstacle", blue_mesh, blue_pos)
            red_obstacle = self.create_mesh_collision_object("red_seiretu_obstacle", red_mesh, red_pos)
            
            # Create planning scene message
            planning_scene = PlanningScene()
            planning_scene.world.collision_objects = [blue_obstacle, red_obstacle]
            planning_scene.is_diff = True
            
            # Publish
            self.planning_scene_pub.publish(planning_scene)
            self.get_logger().info('Published seiretu mesh obstacles to planning scene')
            
            # Stop the timer after publishing once
            self.timer.cancel()
            
        except Exception as e:
            self.get_logger().error(f'Failed to publish obstacles: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = SeiretuMeshObstaclePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()