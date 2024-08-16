#include "r2ps_utils/message.hpp"

r2ps::utils::Message::Message()
{
}

r2ps::utils::Message::~Message()
{
}

builtin_interfaces::msg::Time r2ps::utils::Message::build_time()
{
    builtin_interfaces::msg::Time::UniquePtr time = std::make_unique<builtin_interfaces::msg::Time>();

    const std::chrono::_V2::system_clock::time_point &now = std::chrono::system_clock::now();

    const std::time_t &now_time_t = std::chrono::system_clock::to_time_t(now);

    std::chrono::time_point now_seconds = std::chrono::time_point_cast<std::chrono::seconds>(now);
    auto now_fraction = now - now_seconds;
    const int32_t &now_microseconds = std::chrono::duration_cast<std::chrono::microseconds>(now_fraction).count();

    time->set__sec(static_cast<int32_t>(now_time_t));
    time->set__nanosec(now_microseconds);

    return std::move(*time);
}

std_msgs::msg::Header r2ps::utils::Message::build_header(const char *frame_id)
{
    std_msgs::msg::Header::UniquePtr header = std::make_unique<std_msgs::msg::Header>();

    const builtin_interfaces::msg::Time &stamp = this->build_time();
    header->set__stamp(stamp);
    header->set__frame_id(frame_id);

    return std::move(*header);
}