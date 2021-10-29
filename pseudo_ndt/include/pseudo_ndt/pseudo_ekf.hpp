#ifndef __PSEUDO_EKF_HPP__
#define __PSEUDO_EKF_HPP__

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace pseudo_ndt
{
    class PseudoEKF : public rclcpp::Node
    {
        public:
            explicit PseudoEKF(const rclcpp::NodeOptions &options);

        private:
            rclcpp::TimerBase::SharedPtr timer_;
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sender_;

            uint64_t publish_count_;

            void cyclic_send_message();
    };
}

#endif