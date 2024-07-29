#ifndef R2__HXX
#define R2__HXX

#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <r2ps_msgs/msg/node_list.hpp>

#define NODE_LIST_TOPIC "/r2ps/r2/node/list"

namespace r2ps
{
    namespace process
    {
        class R2Service
        {
        private:
            rclcpp::Node::SharedPtr node_;

            rclcpp::CallbackGroup::SharedPtr current_node_check_timer_cb_group_;
            rclcpp::TimerBase::SharedPtr current_node_check_timer_;

            rclcpp::CallbackGroup::SharedPtr node_list_publisher_cb_group_;
            rclcpp::Publisher<r2ps_msgs::msg::NodeList>::SharedPtr node_list_publisher_;

        private:
            void current_node_check_timer_cb();

        public:
            explicit R2Service(rclcpp::Node::SharedPtr node);
            virtual ~R2Service();

        public:
            using SharedPtr = std::shared_ptr<r2ps::process::R2Service>;
        };
    }
}

#endif // !R2__HXX