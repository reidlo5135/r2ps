#ifndef R2PS_LAUNCH_OPERATOR__HPP
#define R2PS_LAUNCH_OPERATOR__HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <r2ps_utils/process.hpp>

#define NODE_NAME "r2ps_launch_operator"
#define PCAN_STATUS_TOPIC "/r2ps/sys/usb/pcan/status"
#define MOSQUITTO_STATUS_TOPIC "/r2ps/sys/service/mosquitto/status"
#define SOURCE_COMMAND "source /opt/ros/humble/setup.bash && source ~/hs_ws/total.sh"
#define TOTAL_LAUNCH_COMMAND "ros2 launch total_launcher total_launch.py"

using std::placeholders::_1;

namespace r2ps
{
    namespace launch
    {
        class LaunchOperator : public rclcpp::Node
        {
        private:
            rclcpp::Node::SharedPtr node_;
            bool is_total_launched_;
            bool is_enable_to_launch_;
	    int launch_checking_count_;
            bool is_pcan_ok_;
            bool is_mosquitto_ok_;

            r2ps::utils::Process::SharedPtr r2ps_process_utils_;

            rclcpp::CallbackGroup::SharedPtr total_launch_checking_timer_cb_group_;
            rclcpp::TimerBase::SharedPtr total_launch_checking_timer_;

            rclcpp::CallbackGroup::SharedPtr pcan_status_subscription_cb_group_;
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pcan_status_subscription_;

            rclcpp::CallbackGroup::SharedPtr mosquitto_status_subscription_cb_group_;
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mosquitto_subscription_;

        private:
            void total_launch_checking_timer_cb();
            void pcan_status_subscription_cb(std_msgs::msg::Bool::SharedPtr pcan_status);
            void mosquitto_status_subscription_cb(std_msgs::msg::Bool::SharedPtr mosquitto_status);

        public:
            explicit LaunchOperator();
            virtual ~LaunchOperator();

        public:
            using SharedPtr = std::shared_ptr<LaunchOperator>;
        };
    }
}

#endif // !R2PS_LAUNCH_OPERATOR__HPP
