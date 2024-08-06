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
}

r2ps::system::UsbService::~UsbService()
{
}

void r2ps::system::UsbService::lsusb_timer_cb()
{
    const std::string &lsusb_result = this->r2ps_process_utils_->execute_command(LSUSB_COMMAND);

    if (lsusb_result == "")
    {
        RCLCPP_WARN(this->node_->get_logger(), "!!!!! PCAN Has Died.. Restart !!!!!");
        this->is_pcan_disconnected_ = true;
    }
    else
    {
        if (this->is_pcan_disconnected_ == true)
        {
            const std::string &ifconfig_result = this->r2ps_process_utils_->execute_command(IFCONFIG_COMMAND);

            if (ifconfig_result == "")
            {
                const std::string &can_restart_result = this->r2ps_process_utils_->execute_command(CAN_RESTART_COMMAND);
                this->is_pcan_disconnected_ = false;
            }
            else
            {
                return;
            }
        }
    }
}