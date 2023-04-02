#pragma once

#include <cv_bridge/cv_bridge.h>
#include <dvs_msgs/EventArray.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

#include <opencv2/opencv.hpp>
#include <string>
#include <tbb/concurrent_vector.h>

namespace dvs_acc_renderer {

class Renderer {
public:
    Renderer(const std::string &config_file);
    ~Renderer() = default;

private:
    void eventsCallback(const dvs_msgs::EventArray::ConstPtr &msg);
    void compareAndSet(const double &t, const bool &p, double &t_mat, bool &p_mat);

    ros::NodeHandle nh_;
    ros::Subscriber events_sub_;
    image_transport::Publisher image_pub_;

    cv_bridge::CvImage cv_image_;
    tbb::concurrent_vector<bool> pol_mat_;
    tbb::concurrent_vector<double> time_mat_;

    bool pub_render_cost_;
};

} // namespace dvs_acc_renderer