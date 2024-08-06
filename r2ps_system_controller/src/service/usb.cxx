#include "service/usb.hxx"

r2ps::system::UsbService::UsbService(rclcpp::Node::SharedPtr node)
    : is_pcan_disconnected_(false)
{
    this->node_ = node;
    this->r2ps_string_utils_ = std::make_shared<r2ps::utils::String>();
    this->r2ps_process_utils_ = std::make_shared<r2ps::utils::Process>();

    this->lsusb_timer_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->lsusb_timer_ = this->node_->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&r2ps::system::UsbService::lsusb_timer_cb, this),
        this->lsusb_timer_cb_group_);

    this->pcan_status_publisher_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions pcan_status_publisher_opts;
    pcan_status_publisher_opts.callback_group = this->pcan_status_publisher_cb_group_;
    this->pcan_status_publisher_ = this->node_->create_publisher<std_msgs::msg::Bool>(
        PCAN_STATUS_PUBLIHSER_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(1)),
        pcan_status_publisher_opts
    );
}

r2ps::system::UsbService::~UsbService()
{
}

void r2ps::system::UsbService::lsusb_timer_cb()
{
    const std::string &lsusb_result = this->r2ps_process_utils_->execute_command(LSUSB_COMMAND);
    std_msgs::msg::Bool::UniquePtr pcan_status = std::make_unique<std_msgs::msg::Bool>();

    if (lsusb_result == "")
    {
        RCLCPP_WARN(this->node_->get_logger(), "!!!!! PCAN Has Died.. Restart !!!!!");
        this->is_pcan_disconnected_ = true;
        pcan_status->set__data(false);
    }
    else
    {
        if (this->is_pcan_disconnected_ == true)
        {
            const std::string &ifconfig_result = this->r2ps_process_utils_->execute_command(IFCONFIG_COMMAND);

            if (ifconfig_result == "")
            {
                const std::string &can_restart_result = this->r2ps_process_utils_->execute_command(CAN_RESTART_COMMAND);
                
                if (can_restart_result != "")
                {
                    this->is_pcan_disconnected_ = false;
                    pcan_status->set__data(true);
                }
                else
                {
                    pcan_status->set__data(false);
                }
            }
            else
            {
                pcan_status->set__data(false);
                return;
            }
        }
        else
        {
            pcan_status->set__data(true);
        }
    }
    this->pcan_status_publisher_->publish(std::move(*pcan_status));
}