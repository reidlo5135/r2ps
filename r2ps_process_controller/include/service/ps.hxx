#ifndef PS__HXX
#define PS__HXX

#include <sstream>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <r2ps_msgs/msg/process.hpp>
#include <r2ps_msgs/msg/process_list.hpp>

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

            rclcpp::CallbackGroup::SharedPtr process_check_timer_cb_group_;
            rclcpp::TimerBase::SharedPtr process_check_timer_;

            rclcpp::CallbackGroup::SharedPtr process_list_publisher_cb_group_;
            rclcpp::Publisher<r2ps_msgs::msg::ProcessList>::SharedPtr process_list_publisher_;

        private:
            std::vector<std::string> split(const std::string &str, char delimiter);
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