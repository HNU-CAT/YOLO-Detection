#include <ros/ros.h>
#include "d2p/target_detector.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "target_detector");
    ros::NodeHandle nh;
    
    d2p::TargetDetector detector(nh);
    
    ros::spin();
    return 0;
}