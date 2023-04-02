#include "dvs_acc_renderer.h"

#include <ros/ros.h>

using namespace dvs_acc_renderer;

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "dvs_acc_renderer_node");
    ros::NodeHandle nh("~");

    auto config_file = nh.param<std::string>("config_file", "");
    Renderer renderer(config_file);

    ros::spin();

    return 0;
}