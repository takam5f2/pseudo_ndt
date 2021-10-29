#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>
#include "pseudo_ndt/pseudo_pointcloud.hpp"
#include <chrono>
#include <memory>
#include <thread>
#include <sstream>
#include <iostream>

using namespace std::literals::chrono_literals;

namespace pseudo_ndt
{

    PseudoPointCloud::PseudoPointCloud(const rclcpp::NodeOptions &options)
    : Node("pseudo_pointcloud", options)
    {
        publish_count_ = 0;
        // timer.
        timer_ = this->create_wall_timer(1s, std::bind(&PseudoPointCloud::cyclic_send_message, this));

        // publisher.
        sender_ = this->create_publisher<std_msgs::msg::String>("pseudo_pointcloud", rclcpp::SensorDataQoS().keep_last(1));
    }

    void PseudoPointCloud::cyclic_send_message()
    {
        rclcpp::Clock system_clock;
        rclcpp::Time now = system_clock.now();
        uint64_t sec = now.nanoseconds() / 1000000000;
        uint64_t nanosec = now.nanoseconds() % 1000000000;
        std::ostringstream str_nanosec;
        str_nanosec << std::setw(9) << std::setfill('0') << nanosec;

        std_msgs::msg::String msg;
        msg.data = "Pointcloud from prep #"\
                   + std::to_string(publish_count_) \
                   + " times at [" \
                   + std::to_string(sec) \
                   + "."\
                   + str_nanosec.str() + "]";

        sender_->publish(msg);
        publish_count_++;
        RCLCPP_INFO(this->get_logger(), "PointCloud: %s", msg.data.c_str());
    }

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pseudo_ndt::PseudoPointCloud)