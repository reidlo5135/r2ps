#ifndef USB__HXX
#define USB__HXX

#include <rclcpp/rclcpp.hpp>
#include <r2ps_utils/string.hxx>
#include <r2ps_utils/process.hxx>

#define LSUSB_COMMAND "lsusb | grep CAN"
#define IFCONFIG_COMMAND "ifconfig | grep can0"
#define CAN_RESTART_COMMAND "~/RobotData/script/run_pcan_usb.sh"

namespace r2ps
{
    namespace system
    {
        class UsbService
        {
        private:
            rclcpp::Node::SharedPtr node_;
            r2ps::utils::String::SharedPtr r2ps_string_utils_;
            r2ps::utils::Process::SharedPtr r2ps_process_utils_;

            rclcpp::CallbackGroup::SharedPtr lsusb_timer_cb_group_;
            rclcpp::TimerBase::SharedPtr lsusb_timer_;

            bool is_pcan_disconnected_;

        private:
            void lsusb_timer_cb();

        public:
            explicit UsbService(rclcpp::Node::SharedPtr node);
            virtual ~UsbService();

        public:
            using SharedPtr = std::shared_ptr<UsbService>;
        };
    }
}

#endif // !USB__HXX