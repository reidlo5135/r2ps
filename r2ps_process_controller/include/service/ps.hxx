#ifndef PS__HXX
#define PS__HXX

#include <rclcpp/rclcpp.hpp>
#include <r2ps_msgs/msg/process.hpp>
#include <r2ps_msgs/msg/process_list.hpp>

namespace r2ps
{
    namespace process
    {
        class PSService
        {
        private:
            rclcpp::Node::SharedPtr node_;

        public:
            explicit PSService(rclcpp::Node::SharedPtr node);
            virtual ~PSService();

        public:
            using SharedPtr = std::shared_ptr<r2ps::process::PSService>;
        };
    }
}

#endif // !PS__HXX