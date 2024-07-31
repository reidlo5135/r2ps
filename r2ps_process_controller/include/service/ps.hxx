#ifndef PS__HXX
#define PS__HXX

#include <sstream>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <r2ps_msgs/msg/process.hpp>
#include <r2ps_msgs/msg/process_list.hpp>
#include <r2ps_utils/string.hxx>

#define PS_EF "ps -ef"
#define PROCESS_LIST_TOPIC "/r2ps/ps/process/list"

namespace r2ps
{
    namespace process
    {
        class PSService
        {
        private:
            rclcpp::Node::SharedPtr node_;
            r2ps::utils::String::SharedPtr r2ps_string_utils_;

            rclcpp::CallbackGroup::SharedPtr process_check_timer_cb_group_;
            rclcpp::TimerBase::SharedPtr process_check_timer_;

            rclcpp::CallbackGroup::SharedPtr process_list_publisher_cb_group_;
            rclcpp::Publisher<r2ps_msgs::msg::ProcessList>::SharedPtr process_list_publisher_;

        private:
            std::string exec_command(const char *command);
            void process_check_timer_cb();

        public:
            explicit PSService(rclcpp::Node::SharedPtr node);
            virtual ~PSService();

        public:
            using SharedPtr = std::shared_ptr<r2ps::process::PSService>;
        };
    }
}

#endif // !PS__HXX