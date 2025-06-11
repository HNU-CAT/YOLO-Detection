#include "utils.h"
#include "infer.h"
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "yolo_detector");
    ros::NodeHandle nh("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    YoloDetector detector(nh);

    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
