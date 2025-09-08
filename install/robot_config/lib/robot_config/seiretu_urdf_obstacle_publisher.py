#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import Mesh, MeshTriangle
from geometry_msgs.msg import Pose, Point
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
import xml.etree.ElementTree as ET

class SeiretuURDFObstaclePublisher(Node):
    def __init__(self):
        super().__init__('seiretu_urdf_obstacle_publisher')
        
        # Planning scene publisher
        self.planning_scene_pub = self.create_publisher(
            PlanningScene, 
            '/planning_scene', 
            10
        )
        
        # TF buffer for getting transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribe to seiretu joint states
        self.blue_joint_sub = self.create_subscription(
            JointState,
            '/blue_seiretu/joint_states',
            self.blue_joint_callback,
            10
        )
        
        self.red_joint_sub = self.create_subscription(
            JointState,
            '/red_seiretu/joint_states',
            self.red_joint_callback,
            10
        )
        
        # Get robot descriptions
        self.declare_parameter('blue_urdf_content', '')
        self.declare_parameter('red_urdf_content', '')
        
        # Timer to publish obstacles
        self.timer = self.create_timer(5.0, self.publish_obstacles_once)
        self.published = False
        
        self.get_logger().info('Seiretu URDF obstacle publisher started')

    def parse_urdf_collision(self, urdf_content, prefix):
        """Parse URDF and extract collision geometry"""
        try:
            root = ET.fromstring(urdf_content)
            collision_objects = []
            
            for link in root.findall('.//link'):
                link_name = link.get('name')
                if not link_name:
                    continue
                    
                for collision in link.findall('.//collision'):
                    geometry = collision.find('.//geometry')
                    if geometry is not None:
                        mesh = geometry.find('mesh')
                        if mesh is not None:
                            # Create collision object for this mesh
                            collision_obj = self.create_collision_from_urdf_link(
                                link_name, collision, prefix
                            )
                            if collision_obj:
                                collision_objects.append(collision_obj)
                                
            return collision_objects
            
        except Exception as e:
            self.get_logger().error(f'Failed to parse URDF: {str(e)}')
            return []

    def create_collision_from_urdf_link(self, link_name, collision_element, prefix):
        """Create collision object from URDF collision element"""
        try:
            # Get the transform from world to this link
            try:
                transform = self.tf_buffer.lookup_transform(
                    'world', 
                    link_name,
                    rclpy.time.Time()
                )
            except Exception:
                self.get_logger().warn(f'Could not get transform for {link_name}')
                return None
                
            collision_object = CollisionObject()
            collision_object.header.frame_id = "world"
            collision_object.header.stamp = self.get_clock().now().to_msg()
            collision_object.id = f"{prefix}_{link_name}_obstacle"
            
            # Set pose from transform
            pose = Pose()
            pose.position.x = transform.transform.translation.x
            pose.position.y = transform.transform.translation.y
            pose.position.z = transform.transform.translation.z
            pose.orientation = transform.transform.rotation
            
            # Create a simple box for each link (as approximation)
            # You can make this more sophisticated by parsing actual mesh files
            collision_object.primitives = []
            collision_object.primitive_poses = [pose]
            
            # Add as box primitive for simplicity
            from shape_msgs.msg import SolidPrimitive
            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = [0.1, 0.1, 0.1]  # 10cm box
            collision_object.primitives = [box]
            
            collision_object.operation = CollisionObject.ADD
            
            return collision_object
            
        except Exception as e:
            self.get_logger().error(f'Failed to create collision object: {str(e)}')
            return None

    def blue_joint_callback(self, msg):
        """Handle blue seiretu joint state updates"""
        # Could update collision objects based on joint states
        pass

    def red_joint_callback(self, msg):
        """Handle red seiretu joint state updates"""
        # Could update collision objects based on joint states
        pass

    def publish_obstacles_once(self):
        """Publish seiretu robots as obstacles once"""
        if self.published:
            return
            
        try:
            all_collision_objects = []
            
            # Create collision objects for blue seiretu
            blue_frames = ['blue_seiretu_base_link', 'blue_seiretu_1']
            blue_objects = self.create_urdf_based_collision_objects('blue_seiretu', blue_frames)
            all_collision_objects.extend(blue_objects)
            
            # Create collision objects for red seiretu  
            red_frames = ['red_seiretu_base_link', 'red_seiretu_2']
            red_objects = self.create_urdf_based_collision_objects('red_seiretu', red_frames)
            all_collision_objects.extend(red_objects)
            
            # If TF-based approach fails, fall back to simple collision objects
            if not all_collision_objects:
                self.get_logger().warn('TF-based collision objects failed, using fallback')
                blue_collision = self.create_simple_collision_object(
                    "blue_seiretu_obstacle", 
                    [0.5, 0.5, 0.15],  # Position
                    [0.2, 0.1, 0.3]    # Size
                )
                
                red_collision = self.create_simple_collision_object(
                    "red_seiretu_obstacle",
                    [1.0, 0.5, 0.15],  # Position  
                    [0.2, 0.1, 0.3]    # Size
                )
                all_collision_objects = [blue_collision, red_collision]
            
            # Create planning scene message
            planning_scene = PlanningScene()
            planning_scene.world.collision_objects = all_collision_objects
            planning_scene.is_diff = True
            
            # Publish
            self.planning_scene_pub.publish(planning_scene)
            self.get_logger().info(f'Published {len(all_collision_objects)} seiretu URDF obstacles to planning scene')
            
            self.published = True
            
        except Exception as e:
            self.get_logger().error(f'Failed to publish obstacles: {str(e)}')

    def create_urdf_based_collision_objects(self, robot_prefix, frame_names):
        """Create collision objects based on actual URDF collision geometry"""
        collision_objects = []
        
        for frame_name in frame_names:
            try:
                # Get transform from world to this frame
                transform = self.tf_buffer.lookup_transform(
                    'world', 
                    frame_name,
                    rclpy.time.Time()
                )
                
                collision_object = CollisionObject()
                collision_object.header.frame_id = "world"
                collision_object.header.stamp = self.get_clock().now().to_msg()
                collision_object.id = f"{robot_prefix}_{frame_name}_obstacle"
                
                # Set pose from transform
                pose = Pose()
                pose.position.x = transform.transform.translation.x
                pose.position.y = transform.transform.translation.y  
                pose.position.z = transform.transform.translation.z
                pose.orientation = transform.transform.rotation
                
                # Create collision geometry based on the link
                # For seiretu robots, create appropriate collision shapes
                from shape_msgs.msg import SolidPrimitive
                
                if 'base_link' in frame_name:
                    # Main body
                    box = SolidPrimitive()
                    box.type = SolidPrimitive.BOX
                    box.dimensions = [0.15, 0.08, 0.25]  # Main body size
                    collision_object.primitives = [box]
                elif 'seiretu_1' in frame_name or 'seiretu_2' in frame_name:
                    # Moving part
                    box = SolidPrimitive()
                    box.type = SolidPrimitive.BOX
                    box.dimensions = [0.12, 0.06, 0.20]  # Moving part size
                    collision_object.primitives = [box]
                else:
                    # Default small box
                    box = SolidPrimitive()
                    box.type = SolidPrimitive.BOX
                    box.dimensions = [0.05, 0.05, 0.05]
                    collision_object.primitives = [box]
                
                collision_object.primitive_poses = [pose]
                collision_object.operation = CollisionObject.ADD
                
                collision_objects.append(collision_object)
                
            except Exception as e:
                self.get_logger().warn(f'Could not create collision for {frame_name}: {str(e)}')
                continue
                
        return collision_objects

    def create_simple_collision_object(self, name, position, size):
        """Create a simple box collision object"""
        from shape_msgs.msg import SolidPrimitive
        
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
        
        # Create box
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = size
        
        collision_object.primitives = [box]
        collision_object.primitive_poses = [pose]
        collision_object.operation = CollisionObject.ADD
        
        return collision_object

def main(args=None):
    rclpy.init(args=args)
    node = SeiretuURDFObstaclePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()