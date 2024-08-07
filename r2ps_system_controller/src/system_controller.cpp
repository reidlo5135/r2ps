#include "system_controller.hpp"

r2ps::system::SystemController::SystemController()
    : Node(NODE_NAME)
{
    this->node_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});
    this->usb_manager_ = std::make_shared<r2ps::system::UsbManager>(this->node_);
    this->service_manager_ = std::make_shared<r2ps::system::ServiceManager>(this->node_);
}

r2ps::system::SystemController::~SystemController()
{
}