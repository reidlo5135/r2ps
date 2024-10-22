#ifndef USB_MANAGER__HPP
#define USB_MANAGER__HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <r2ps_utils/string.hpp>
#include <r2ps_utils/process.hpp>

#define LSUSB_COMMAND "lsusb | grep CAN"
#define IFCONFIG_COMMAND "ifconfig | grep can0"
#define CAN_RESTART_COMMAND "~/RobotData/script/run_pcan_usb.sh"
#define PCAN_STATUS_PUBLIHSER_TOPIC "/r2ps/sys/usb/pcan/status"

namespace r2ps
{
    namespace system
    {
        class UsbManager
        {
        private:
            rclcpp::Node::SharedPtr node_;
            r2ps::utils::String::SharedPtr r2ps_string_utils_;
            r2ps::utils::Process::SharedPtr r2ps_process_utils_;

            rclcpp::CallbackGroup::SharedPtr lsusb_timer_cb_group_;
            rclcpp::TimerBase::SharedPtr lsusb_timer_;

            rclcpp::CallbackGroup::SharedPtr pcan_status_publisher_cb_group_;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pcan_status_publisher_;

            bool is_pcan_disconnected_;

        private:
            void lsusb_timer_cb();

        public:
            explicit UsbManager(rclcpp::Node::SharedPtr node);
            virtual ~UsbManager();

        public:
            using SharedPtr = std::shared_ptr<UsbManager>;
        };
    }
}

#endif // !USB_MANAGER__HPP