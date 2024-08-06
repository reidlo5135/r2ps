#ifndef CONTROLLER__HXX
#define CONTROLLER__HXX

#include <rclcpp/rclcpp.hpp>
#include "service/usb.hxx"

#define NODE_NAME "r2ps_system_controller"

namespace r2ps
{
    namespace system
    {
        class Controller : public rclcpp::Node
        {
        private:
            rclcpp::Node::SharedPtr node_;
            r2ps::system::UsbService::SharedPtr usb_service_;
        public:
            explicit Controller();
            virtual ~Controller();
        public:
            using SharedPtr = std::shared_ptr<Controller>;
        };
    }
}

#endif // !CONTROLLER__HXX