#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>
#include "pseudo_ndt/pseudo_ndt.hpp"
#include <chrono>
#include <memory>
#include <thread>

using namespace std::literals::chrono_literals;
using std::placeholders::_1;

namespace pseudo_ndt
{

    PseudoNDT::PseudoNDT(const rclcpp::NodeOptions &options)
    : Node("pseudo_ndt", options)
    {
        counter_called_pseudo_pc_callback = 0;
        counter_called_pseudo_initial_pose_callback = 0;

        callback_group_mutex_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = callback_group_mutex_;

        // subscriber.
        pseudo_pc_sub_ = this->create_subscription<std_msgs::msg::String>("pseudo_pointcloud", rclcpp::SensorDataQoS().keep_last(1), std::bind(&PseudoNDT::pseudo_recv_pointcloud, this, _1), sub_opt);
        pseudo_initialpose_sub_ = this->create_subscription<std_msgs::msg::String>("pseudo_initialpose", 100, std::bind(&PseudoNDT::pseudo_recv_initialpose, this, _1), sub_opt);
    }

    void PseudoNDT::pseudo_recv_pointcloud(const std_msgs::msg::String::SharedPtr msg)
    {
        print_count_common("pseudo_recv_pointcloud",
                            counter_called_pseudo_pc_callback);
        received_pc_ = msg->data;

        RCLCPP_INFO(this->get_logger(), "PointCloud: %s", received_pc_.c_str());
        RCLCPP_INFO(this->get_logger(), "InitialPose: %s", received_initial_pose_.c_str());

        // wait for 700 millisec
        std::this_thread::sleep_for(std::chrono::milliseconds(700));
    }

    void PseudoNDT::pseudo_recv_initialpose(const std_msgs::msg::String::SharedPtr msg)
    {
        print_count_common("pseudo_recv_initialpose",
                            counter_called_pseudo_initial_pose_callback);
        received_initial_pose_ = msg->data;

        // wait for 100 millisec
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    void PseudoNDT::print_count_common(const std::string & callback_name, 
                                        uint64_t & counter)
    {
        rclcpp::Clock system_clock;
        rclcpp::Time now = system_clock.now();
        uint64_t sec = now.nanoseconds() / 1000000000;
        uint64_t nanosec = now.nanoseconds() % 1000000000;
        RCLCPP_INFO(this->get_logger(), "%s called [%ld] at [%ld.%ld]",
                    callback_name.c_str(), counter, sec, nanosec);
        counter++;
        return;
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pseudo_ndt::PseudoNDT)