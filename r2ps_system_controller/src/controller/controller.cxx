#include "controller/controller.hxx"

r2ps::system::Controller::Controller()
    : Node(NODE_NAME)
{
    this->node_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});
    this->usb_service_ = std::make_shared<r2ps::system::UsbService>(this->node_);
}

r2ps::system::Controller::~Controller()
{
}