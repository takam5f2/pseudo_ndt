#ifndef __PSEUDO_NDH_HPP__
#define __PSEUDO_NDH_HPP__

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace pseudo_ndt
{
    class PseudoNDT : public rclcpp::Node
    {
        public:
            explicit PseudoNDT(const rclcpp::NodeOptions &options);

        private:
            rclcpp::CallbackGroup::SharedPtr callback_group_mutex_;

            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr pseudo_pc_sub_;
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr pseudo_initialpose_sub_;

            uint64_t counter_called_pseudo_pc_callback;
            uint64_t counter_called_pseudo_initial_pose_callback;

            std::string received_pc_;
            std::string received_initial_pose_;

            void pseudo_recv_pointcloud(const std_msgs::msg::String::SharedPtr msg);
            void pseudo_recv_initialpose(const std_msgs::msg::String::SharedPtr msg);
            void print_count_common(const std::string & callback_name,
                                    uint64_t & counter);
    };
}

#endif