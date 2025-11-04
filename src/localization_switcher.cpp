// src/localization_switcher.cpp
#include <rclcpp/rclcpp.hpp>
#include "localization_switcher/localization_switcher_node.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<localization_switcher::LocalizationSwitchNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
