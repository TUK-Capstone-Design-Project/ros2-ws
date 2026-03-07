#include "app_bridge/app-bridge-node.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<app_bridge::AppBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
