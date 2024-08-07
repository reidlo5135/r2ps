#include "r2ps_launch_operator.hpp"

int main(int argc, const char *const *argv)
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = std::make_shared<r2ps::launch::LaunchOperator>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    
    return 0;
}