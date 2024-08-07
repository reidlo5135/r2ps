#include "r2ps_launch_operator.hpp"

r2ps::launch::LaunchOperator::LaunchOperator()
    : Node(NODE_NAME),
      is_total_launched_(false),
      is_enable_to_launch_(false),
	launch_checking_count_(0),
      is_pcan_ok_(false),
      is_mosquitto_ok_(false)
{
    this->node_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});
    this->r2ps_process_utils_ = std::make_shared<r2ps::utils::Process>();

    this->total_launch_checking_timer_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->total_launch_checking_timer_ = this->node_->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&r2ps::launch::LaunchOperator::total_launch_checking_timer_cb, this),
        this->total_launch_checking_timer_cb_group_);

    this->pcan_status_subscription_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions pcan_status_subscription_opts;
    pcan_status_subscription_opts.callback_group = this->pcan_status_subscription_cb_group_;
    this->pcan_status_subscription_ = this->node_->create_subscription<std_msgs::msg::Bool>(
        PCAN_STATUS_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(1)),
        std::bind(&r2ps::launch::LaunchOperator::pcan_status_subscription_cb, this, _1),
        pcan_status_subscription_opts);

    this->mosquitto_status_subscription_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions mosquitto_subscription_opts;
    mosquitto_subscription_opts.callback_group = this->mosquitto_status_subscription_cb_group_;
    this->mosquitto_subscription_ = this->node_->create_subscription<std_msgs::msg::Bool>(
        MOSQUITTO_STATUS_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(1)),
        std::bind(&r2ps::launch::LaunchOperator::mosquitto_status_subscription_cb, this, _1),
        mosquitto_subscription_opts);
}

r2ps::launch::LaunchOperator::~LaunchOperator()
{
}

void r2ps::launch::LaunchOperator::total_launch_checking_timer_cb()
{
    try
    {

        this->is_enable_to_launch_ = this->is_pcan_ok_ == true && this->is_mosquitto_ok_ == true;

        RCLCPP_INFO(this->node_->get_logger(), "!!!!! Launch Checking !!!!");
        RCLCPP_INFO(this->node_->get_logger(), "\n\tlaunch_checking_count : [%d]", this->launch_checking_count_);
        RCLCPP_INFO(this->node_->get_logger(), "\n\tis_pcan_ok : [%d]", this->is_pcan_ok_);
        RCLCPP_INFO(this->node_->get_logger(), "\n\tis_mosquitto_ok : [%d]", this->is_mosquitto_ok_);
        RCLCPP_INFO(this->node_->get_logger(), "\n\tis_enable_to_launch : [%d]", this->is_enable_to_launch_);

        if (this->is_total_launched_ == false && this->is_enable_to_launch_ == true)
        {
		RCLCPP_INFO(this->node_->get_logger(), "!!!!! Launch Ready !!!!!");
            this->launch_checking_count_++;

            if (this->launch_checking_count_ == 5)
            {
                const std::string &source_output = this->r2ps_process_utils_->execute_command(SOURCE_COMMAND);

                if (source_output == "")
                {
                    const std::string &total_launch_output = this->r2ps_process_utils_->execute_command(TOTAL_LAUNCH_COMMAND);

                    if (total_launch_output != "")
                    {
                        RCLCPP_INFO(this->node_->get_logger(), "===== Total Launched =====");
                        this->is_total_launched_ = true;
                    }
                    else
                    {
                        this->is_total_launched_ = false;
                    }
                }
                else
                {
                    this->is_total_launched_ = false;
                }
            }
        }
        else
        {
            RCLCPP_ERROR(this->node_->get_logger(), "launch failed, retrying");
            this->launch_checking_count_ = 0;
            this->is_total_launched_ = false;
        }
    }
    catch (const std::exception &expn)
    {
        RCLCPP_ERROR(this->node_->get_logger(), "%s", expn.what());
    }
}

void r2ps::launch::LaunchOperator::pcan_status_subscription_cb(std_msgs::msg::Bool::SharedPtr pcan_status)
{
    this->is_pcan_ok_ = pcan_status->data;
}

void r2ps::launch::LaunchOperator::mosquitto_status_subscription_cb(std_msgs::msg::Bool::SharedPtr mosquitto_status)
{
    this->is_mosquitto_ok_ = mosquitto_status->data;
}
