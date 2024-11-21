#include "ros/ros.h"
#include "ars548_ros/ars548_ros.h"

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "ars548_driver");
    ars548_driver_class node;
    node.run();
    ros::spin();
    return 0;
}
