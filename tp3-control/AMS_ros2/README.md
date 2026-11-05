======== For Exercise 3 =========

(cd into workspace src)

ros2 pkg create --build-type ament_cmake my_scan_analyzer

move script in my_scan_analyzer src

colcon build --packages-select my_scan_analyzer

source install/setup.bash

ros2 run my_scan_analyzer scan_analyzer_node

ros2 bag play rosbag2_2026_04_27-15_40_42/

Achtung : the script doesnt do what the assignement require, it is simply a try to make a node interact with the /scan topic. 
