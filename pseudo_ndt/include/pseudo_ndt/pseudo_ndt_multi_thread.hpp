#ifndef __PSEUDO_NDH_MULTI_THREAD_HPP__
#define __PSEUDO_NDH_MULTI_THREAD_HPP__

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <mutex>

namespace pseudo_ndt
{
    class PseudoNDTMultiThread : public rclcpp::Node
    {
        public:
            explicit PseudoNDTMultiThread(const rclcpp::NodeOptions &options);

        private:
            std::mutex initial_pose_mtx_;
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