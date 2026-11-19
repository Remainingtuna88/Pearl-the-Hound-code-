# Runtime (both modes)
flask>=2.3
requests>=2.28
numpy>=1.24
opencv-python-headless>=4.8     # on the Pi you may already have OpenCV from ROS; then drop this line

# Real robot only — provided by the ROS2 Humble install, NOT pip:
#   rclpy, rosidl_runtime_py, tf2_ros, action_msgs, nav2_msgs, sensor_msgs, geometry_msgs, nav_msgs
#   (cv_bridge is NOT required; raw images are decoded with numpy)

# Tests
pytest>=7
