import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/a/ws_moveit2/install/realsense2_ros_mqtt_bridge'
