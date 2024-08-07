#ifndef SERVICE_MANAGER__HPP
#define SERVICE_MANAGER__HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <r2ps_utils/string.hpp>
#include <r2ps_utils/process.hpp>

#define MOSQUITTO_STATUS_COMMAND "systemctl status mosquitto.service"
#define MOSQUITTO_RESTART_COMMAND "echo 1234 | sudo -S systemctl restart mosquitto.service"
#define MOSQUITTO_STATUS_PUBLISHER_TOPIC "/r2ps/sys/service/mosquitto/status"

namespace r2ps
{
    namespace system
    {
        class ServiceManager
        {
        private:
            rclcpp::Node::SharedPtr node_;
            r2ps::utils::String::SharedPtr r2ps_string_utils_;
            r2ps::utils::Process::SharedPtr r2ps_process_utils_;

            rclcpp::CallbackGroup::SharedPtr service_status_timer_cb_group_;
            rclcpp::TimerBase::SharedPtr service_status_timer_;

            rclcpp::CallbackGroup::SharedPtr mosquitto_status_publisher_cb_group_;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mosquitto_status_publisher_;

        private:
            void service_status_timer_cb();
            void restart_mosquitto_service();

        public:
            explicit ServiceManager(rclcpp::Node::SharedPtr node);
            virtual ~ServiceManager();

        public:
            using SharedPtr = std::shared_ptr<ServiceManager>;
        };
    }
}

#endif // !SERVICE_MANAGER__HPP