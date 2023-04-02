#include "dvs_acc_renderer.h"

#include <tbb/parallel_for.h>
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
            pol_mat_.resize(msg->height * msg->width, false);
            time_mat_.resize(msg->height * msg->width, 0.0);
        } else {
            cv_image_.image.setTo(cv::Scalar(0, 0, 0));
            pol_mat_.assign(pol_mat_.size(), false);
            time_mat_.assign(time_mat_.size(), 0.0);
        }

        ros::Time t0 = ros::Time::now();

        // 并行嵌入事件
        tbb::parallel_for(tbb::blocked_range<size_t>(0, msg->events.size()), [&](const tbb::blocked_range<size_t> &r) {
            for (auto i = r.begin(); i != r.end(); i++) {
                const int x   = msg->events[i].x;
                const int y   = msg->events[i].y;
                const int idx = y * msg->width + x;
                compareAndSet(msg->events[i].ts.toSec(), msg->events[i].polarity, time_mat_[idx], pol_mat_[idx]);
            }
        });

        // 生成图像
        tbb::parallel_for(tbb::blocked_range<size_t>(0, msg->height), [&](const tbb::blocked_range<size_t> &r) {
            for (auto i = r.begin(); i != r.end(); i++) {
                for (size_t j = 0; j < msg->width; j++) {
                    const int idx = i * msg->width + j;
                    if (time_mat_[idx] > 0.0) {
                        cv_image_.image.at<cv::Vec3b>(cv::Point(j, i)) =
                            (pol_mat_[idx] ? cv::Vec3b(255, 0, 0) : cv::Vec3b(0, 0, 255));
                    }
                }
            }
        });

        ros::Duration dt = ros::Time::now() - t0;
        if (pub_render_cost_)
            ROS_INFO_STREAM("Rendering time: " << dt.toSec() * 1000 << " ms");

        // 发布图像
        image_pub_.publish(cv_image_.toImageMsg());
    }
}

void Renderer::compareAndSet(const double &t, const bool &p, double &t_mat, bool &p_mat) {
    if (t > t_mat) {
        t_mat = t;
        p_mat = p;
    }
}

} // namespace dvs_acc_renderer