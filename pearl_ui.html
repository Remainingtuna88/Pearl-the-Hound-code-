#!/usr/bin/env bash
# Run ON THE PI with the Hiwonder stock stack up. Prints everything needed to
# fill in the TODO(hw) placeholders in pearl/config.py.
#   source /opt/ros/humble/setup.bash   (and the Hiwonder workspace setup.bash)
#   bash scripts/discover_hw.sh | tee hw_discovery.txt
set -u
hr() { printf '\n==== %s ====\n' "$1"; }

hr "TOPICS (name : type)";        ros2 topic list -t
hr "ACTIONS (name : type)";       ros2 action list -t
hr "NODES";                       ros2 node list

for t in /scan /cmd_vel /controller/cmd_vel /cmd_vel_nav /odom; do
  if ros2 topic list | grep -qx "$t"; then hr "topic info $t"; ros2 topic info -v "$t" | head -40; fi
done

hr "camera-ish topics";  ros2 topic list -t | grep -iE 'image|camera|depth|rgb' || echo "(none matched)"
hr "battery-ish topics"; ros2 topic list -t | grep -iE 'batt|volt|power' || echo "(none matched)"
hr "servo-ish topics";   ros2 topic list -t | grep -iE 'servo|pan|tilt|pwm' || echo "(none matched)"

hr "one battery message (first match)"
bt=$(ros2 topic list | grep -iE 'batt|volt' | head -1)
[ -n "$bt" ] && timeout 5 ros2 topic echo --once "$bt" || echo "(no battery topic found)"

hr "one LaserScan header (angle_min/max/increment, range_min/max)"
timeout 5 ros2 topic echo --once /scan --field angle_min 2>/dev/null; timeout 5 ros2 topic echo --once /scan --field angle_increment 2>/dev/null
timeout 5 ros2 topic echo --once /scan --field range_min 2>/dev/null;   timeout 5 ros2 topic echo --once /scan --field range_max 2>/dev/null

hr "Nav2 NavigateToPose interface"
ros2 interface show nav2_msgs/action/NavigateToPose 2>/dev/null || echo "(nav2_msgs not found — is Nav2 installed/sourced?)"

hr "TF frames (map / odom / base_*)"
timeout 6 ros2 run tf2_ros tf2_echo map base_footprint 2>&1 | head -5
timeout 6 ros2 run tf2_ros tf2_echo map base_link 2>&1 | head -5

hr "custom message packages that may hold battery/servo types"
ros2 pkg list | grep -iE 'hiwonder|ros_robot_controller|mentor|servo' || echo "(none matched)"
