#include "pseudo_ndt/pseudo_ndt_multi_thread.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;

    rclcpp::executors::MultiThreadedExecutor exec;

    auto pseudo_ndt_node = std::make_shared<pseudo_ndt::PseudoNDTMultiThread>(options);

    exec.add_node(pseudo_ndt_node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}

