#pragma once

#include <cv_bridge/cv_bridge.h>
#include <dvs_msgs/EventArray.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

#include <opencv2/opencv.hpp>
#include <string>

namespace dvs_acc_renderer {

class Renderer {
public:
    Renderer(const std::string &config_file);
    ~Renderer() = default;

private:
    void eventsCallback(const dvs_msgs::EventArray::ConstPtr &msg);

    ros::NodeHandle nh_;
    ros::Subscriber events_sub_;
    image_transport::Publisher image_pub_;

    cv_bridge::CvImage cv_image_;

    bool pub_render_cost_;
    size_t step_;
};

} // namespace dvs_acc_renderer