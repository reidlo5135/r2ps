#include "service/ps.hxx"

r2ps::process::PSService::PSService(rclcpp::Node::SharedPtr node)
{
    this->node_ = node;
    RCLCPP_INFO(this->node_->get_logger(), "%s registered", "ps service");
}

r2ps::process::PSService::~PSService()
{
}