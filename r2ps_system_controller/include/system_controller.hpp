#ifndef SYSTEM_CONTROLLER__HXX
#define SYSTEM_CONTROLLER__HXX

#include <rclcpp/rclcpp.hpp>
#include "usb_manager.hpp"
#include "service_manager.hpp"

#define NODE_NAME "r2ps_system_controller"

namespace r2ps
{
    namespace system
    {
        class SystemController : public rclcpp::Node
        {
        private:
            rclcpp::Node::SharedPtr node_;
            r2ps::system::UsbManager::SharedPtr usb_manager_;
            r2ps::system::ServiceManager::SharedPtr service_manager_;

        public:
            explicit SystemController();
            virtual ~SystemController();

        public:
            using SharedPtr = std::shared_ptr<SystemController>;
        };
    }
}

#endif // !SYSTEM_CONTROLLER__HXX