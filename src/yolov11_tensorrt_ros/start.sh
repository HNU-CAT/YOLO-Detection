source ./devel/setup.bash
rosrun rviz rviz -d $(rospack find yolo_detector_node)/rviz/detector_lv.rviz  & sleep 3
roslaunch yolo_detector_node detector.launch
wait;