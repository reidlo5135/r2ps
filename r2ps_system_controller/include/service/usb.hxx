#ifndef USB__HXX
#define USB__HXX

#include <rclcpp/rclcpp.hpp>

namespace r2ps
{
    namespace system
    {
        class UsbService
        {
        private:
            rclcpp::Node::SharedPtr node_;
        public:
            explicit UsbService(rclcpp::Node::SharedPtr node);
            virtual ~UsbService();
        public:
            using SharedPtr = std::shared_ptr<UsbService>;
        };
    }
}

#endif // !USB__HXX