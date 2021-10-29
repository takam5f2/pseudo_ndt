#include "pseudo_ndt/pseudo_ndt.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;

    rclcpp::executors::SingleThreadedExecutor exec;

    auto pseudo_ndt_node = std::make_shared<pseudo_ndt::PseudoNDT>(options);

    exec.add_node(pseudo_ndt_node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}

