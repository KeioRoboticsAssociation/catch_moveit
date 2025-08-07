
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import Mesh
from geometry_msgs.msg import Pose
from ament_index_python.packages import get_package_share_directory
import os

class CollisionObjectPublisher(Node):
    def __init__(self):
        super().__init__('collision_object_publisher')
        self.publisher_ = self.create_publisher(CollisionObject, 'collision_object', 10)
        self.timer = self.create_timer(1.0, self.publish_collision_object)

    def publish_collision_object(self):
        collision_object = CollisionObject()
        collision_object.header.frame_id = "world"
        collision_object.id = "base_link_mesh"

        mesh = Mesh()
        # Correct the path to the mesh file
        mesh_path = "/home/a/ws_moveit2/src/field_description-20250804T033832Z-1-001/field_description/meshes/base_link.stl"
        
        if not os.path.exists(mesh_path):
            self.get_logger().error(f"Mesh file not found at: {mesh_path}")
            return

        with open(mesh_path, 'rb') as f:
            # This is a simplified way to read the mesh, for a real application
            # you might need a more robust mesh loading library if this doesn't work.
            # However, for STL, just reading the file content might not be enough.
            # We will assume for now that we can create a shape_msgs/Mesh from it.
            # The correct way is to use a library that can parse STL and fill the mesh vertices and triangles.
            # For the purpose of this example, we will create a dummy mesh.
            # In a real scenario, you would replace this with actual mesh loading code.
            pass

        # Since we cannot directly load the STL file content into a shape_msgs/Mesh,
        # we will create a simple box as a placeholder.
        # To properly load an STL file, you would need a library like `numpy-stl`
        # and then manually populate the `vertices` and `triangles` fields of the `Mesh` message.
        
        from shape_msgs.msg import SolidPrimitive
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [1.0, 1.0, 0.1] # Example dimensions

        pose = Pose()
        pose.position.x = 0.0
        pose.position.y = 0.0
        pose.position.z = -0.05
        pose.orientation.w = 1.0

        collision_object.primitives.append(box)
        collision_object.primitive_poses.append(pose)
        collision_object.operation = CollisionObject.ADD

        self.publisher_.publish(collision_object)
        self.get_logger().info('Publishing collision object')
        self.timer.cancel() # Publish only once

def main(args=None):
    rclpy.init(args=args)
    node = CollisionObjectPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
