#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import Mesh, MeshTriangle
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Header
import numpy as np
from stl import mesh

class CollisionMeshPublisher(Node):
    def __init__(self):
        super().__init__('collision_mesh_publisher')
        self.publisher_ = self.create_publisher(CollisionObject, 'collision_object', 10)
        self.timer = self.create_timer(1.0, self.publish_collision_mesh)

    def publish_collision_mesh(self):
        collision_object = CollisionObject()
        collision_object.header.frame_id = "world"
        collision_object.id = "base_link_stl"

        mesh_path = "/home/a/ws_moveit2/src/field_description-20250804T033832Z-1-001/field_description/meshes/base_link.stl"

        try:
            your_mesh = mesh.Mesh.from_file(mesh_path)
        except Exception as e:
            self.get_logger().error(f"Failed to load mesh file: {e}")
            return

        scale = 0.001  # ここで倍率を指定します

        shape_mesh = Mesh()
        for v in your_mesh.vectors:
            for vertex in v:
                p = Point()
                p.x = float(vertex[0]) * scale
                p.y = float(vertex[1]) * scale
                p.z = float(vertex[2]) * scale
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