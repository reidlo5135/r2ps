#include "controller/controller.hxx"

r2ps::process::Controller::Controller()
    : Node(NODE_NAME)
{
    this->node_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});
    RCLCPP_INFO(this->get_logger(), "%s created", this->node_->get_name());

    this->r2_service_ = std::make_shared<r2ps::process::R2Service>(this->node_);
    this->ps_service_ = std::make_shared<r2ps::process::PSService>(this->node_);
}

r2ps::process::Controller::~Controller()
{
}
