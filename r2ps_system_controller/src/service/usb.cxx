#include "service/usb.hxx"

r2ps::system::UsbService::UsbService(rclcpp::Node::SharedPtr node)
{
    this->node_ = node;
}

r2ps::system::UsbService::~UsbService()
{

}