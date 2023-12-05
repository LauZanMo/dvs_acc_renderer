#include "dvs_acc_renderer.h"

#include <yaml-cpp/yaml.h>

namespace dvs_acc_renderer {

Renderer::Renderer(const std::string &config_file)
    : nh_("~") {

    // 加载配置文件
    YAML::Node config;
    try {
        config = YAML::LoadFile(config_file);
    } catch (YAML::Exception &e) {
        ROS_ERROR_STREAM("Failed to open configuration file");
        return;
    }

    // 获取参数
    auto events_topic = config["events_topic"].as<std::string>();
    auto image_topic  = config["image_topic"].as<std::string>();
    pub_render_cost_  = config["pub_render_cost"].as<bool>();

    // 初始化
    image_transport::ImageTransport it(nh_);
    events_sub_               = nh_.subscribe(events_topic, 1000, &Renderer::eventsCallback, this);
    image_pub_                = it.advertise(image_topic, 100);
    cv_image_.encoding        = "bgr8";
    cv_image_.header.frame_id = "dvs";
}

void Renderer::eventsCallback(const dvs_msgs::EventArray::ConstPtr &msg) {
    if (image_pub_.getNumSubscribers() > 0) {

        if (msg->events.size() > 0)
            cv_image_.header.stamp = msg->events[msg->events.size() / 2].ts;
        else
            return;

        // 若未初始化，则初始化，否则重置
        if (cv_image_.image.rows != static_cast<int>(msg->height) ||
            cv_image_.image.cols != static_cast<int>(msg->width)) {
            cv_image_.image = cv::Mat(msg->height, msg->width, CV_8UC3, cv::Scalar(0, 0, 0));
        } else {
            cv_image_.image.setTo(cv::Scalar(0, 0, 0));
        }

        ros::Time t0 = ros::Time::now();

        // 顺序遍历嵌入事件
        for (auto &e : msg->events) {
            // 若事件在图像范围内，则绘制
            uchar *pixel = cv_image_.image.ptr(e.y, e.x);
            // 如果所在像素值不为黑色，则不绘制

            // 绘制
            if (e.polarity) {
                pixel[0] = 255;
                pixel[2] = 0;
            } else {
                pixel[2] = 255;
                pixel[0] = 0;
            }
        }

        ros::Duration dt = ros::Time::now() - t0;
        if (pub_render_cost_)
            ROS_INFO_STREAM("Rendering time: " << dt.toSec() * 1000 << " ms");

        // 发布图像
        image_pub_.publish(cv_image_.toImageMsg());
    }
}

} // namespace dvs_acc_renderer