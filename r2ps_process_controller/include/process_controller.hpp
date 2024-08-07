#ifndef PROCESS_CONTROLLER__HPP
#define PROCESS_CONTROLLER__HPP

#include <rclcpp/rclcpp.hpp>
#include "r2_manager.hpp"
#include "ps_manager.hpp"

#define NODE_NAME "r2ps_process_controller"

namespace r2ps
{
    namespace process
    {
        class Controller : public rclcpp::Node
        {
        private:
            rclcpp::Node::SharedPtr node_;
            r2ps::process::R2Service::SharedPtr r2_service_;
            r2ps::process::PSService::SharedPtr ps_service_;

        public:
            explicit Controller();
            virtual ~Controller();
        };
    }
}

#endif // !PROCESS_CONTROLLER__HPP