#ifndef MESSAGE__HPP
#define MESSAGE__HPP

#include <memory>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <std_msgs/msg/header.hpp>
#include <builtin_interfaces/msg/time.hpp>

namespace r2ps
{
    namespace utils
    {
        class Message
        {
        private:
            builtin_interfaces::msg::Time build_time();

        public:
            explicit Message();
            virtual ~Message();
            std_msgs::msg::Header build_header(const char *frame_id);

        public:
            using SharedPtr = std::shared_ptr<Message>;
        };
    }
}

#endif // !MESSAGE__HPP