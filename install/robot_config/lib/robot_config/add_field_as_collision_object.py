import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import Mesh, SolidPrimitive
from geometry_msgs.msg import Pose
from ament_index_python.packages import get_package_share_directory
import os
import xacro
from urdf_parser_py.urdf import URDF, Mesh as URDFMesh
from xml.etree import ElementTree as ET

class FieldCollisionObjectPublisher(Node):
    def __init__(self):
        super().__init__('field_collision_object_publisher')
        self.publisher_ = self.create_publisher(CollisionObject, '/planning_scene', 10)
        self.timer = self.create_timer(1.0, self.publish_collision_object) # Publish every second

        self.get_logger().info('FieldCollisionObjectPublisher node started.')

        self.field_urdf_path = os.path.join(
            get_package_share_directory('field_description'),
            'urdf',
            'field.urdf'
        )

        if not os.path.exists(self.field_urdf_path):
            self.get_logger().error(f"Field URDF file not found at: {self.field_urdf_path}")
            return

        self.robot = URDF.from_xml_file(self.field_urdf_path)
        self.get_logger().info(f"Loaded field URDF from: {self.field_urdf_path}")

    def publish_collision_object(self):
        co = CollisionObject()
        co.header.frame_id = "world"
        co.id = "field"

        for link in self.robot.links:
            if link.visual and link.visual.geometry:
                if isinstance(link.visual.geometry, URDFMesh):
                    mesh_filename = link.visual.geometry.filename
                    # Resolve package:// path
                    if mesh_filename.startswith("package://"):
                        package_name = mesh_filename.split('/')[2]
                        relative_path = '/'.join(mesh_filename.split('/')[3:])
                        try:
                            package_share_directory = get_package_share_directory(package_name)
                            mesh_path = os.path.join(package_share_directory, relative_path)
                        except Exception as e:
                            self.get_logger().error(f"Could not find package {package_name}: {e}")
                            continue
                    else:
                        mesh_path = mesh_filename # Assume absolute or relative to current working dir

                    if not os.path.exists(mesh_path):
                        self.get_logger().warn(f"Mesh file not found: {mesh_path}")
                        continue

                    mesh = Mesh()
                    # For simplicity, we're not loading the actual mesh data here.
                    # In a real application, you would load the mesh from the file.
                    # MoveIt typically handles this if the mesh is referenced in the URDF.
                    # Here, we're just creating a placeholder.
                    # The important part is the pose relative to the CollisionObject's frame.

                    # For a simple example, let's assume the field is at the origin of the world frame
                    # and its meshes are defined relative to its base link.
                    # We need to transform the mesh's origin relative to the link's origin.
                    # This part can be complex depending on how the URDF is structured.
                    # For now, we'll assume the mesh origin is relative to the link's origin.

                    visual_origin = link.visual.origin
                    pose = Pose()
                    if visual_origin:
                        pose.position.x = visual_origin.xyz[0]
                        pose.position.y = visual_origin.xyz[1]
                        pose.position.z = visual_origin.xyz[2]
                        # Assuming rpy is in radians
                        # For simplicity, not converting rpy to quaternion here.
                        # In a real scenario, you'd use transformations.
                        # For now, assuming field is static and aligned with world.
                    
                    # Add a dummy mesh for now, as actual mesh loading is complex
                    # MoveIt will use the mesh from the URDF if it's properly linked.
                    # This is more about telling MoveIt *where* the object is.
                    co.meshes.append(mesh)
                    co.mesh_poses.append(pose)
                    
                elif isinstance(link.visual.geometry, SolidPrimitive):
                    # Handle primitive shapes if necessary
                    pass

        co.operation = CollisionObject.ADD
        self.publisher_.publish(co)
        self.get_logger().info('Published field as CollisionObject.')

def main(args=None):
    rclpy.init(args=args)
    node = FieldCollisionObjectPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
