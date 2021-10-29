#include "pseudo_ndt/pseudo_pointcloud.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;

    rclcpp::executors::SingleThreadedExecutor exec;

    auto pseudo_pc_node = std::make_shared<pseudo_ndt::PseudoPointCloud>(options);

    exec.add_node(pseudo_pc_node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
