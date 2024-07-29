#ifndef CONTROLLER__HXX
#define CONTROLLER__HXX

#include <rclcpp/rclcpp.hpp>
#include "service/r2.hxx"
#include "service/ps.hxx"

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

#endif // !CONTROLLER__HXX