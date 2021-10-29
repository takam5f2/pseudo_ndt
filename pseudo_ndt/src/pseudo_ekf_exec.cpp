#include "pseudo_ndt/pseudo_ekf.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;

    rclcpp::executors::SingleThreadedExecutor exec;

    auto pseudo_ekf_node = std::make_shared<pseudo_ndt::PseudoEKF>(options);

    exec.add_node(pseudo_ekf_node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
