#include "service_manager.hpp"

r2ps::system::ServiceManager::ServiceManager(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    this->r2ps_string_utils_ = std::make_shared<r2ps::utils::String>();
    this->r2ps_process_utils_ = std::make_shared<r2ps::utils::Process>();

    this->service_status_timer_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->service_status_timer_ = this->node_->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&r2ps::system::ServiceManager::service_status_timer_cb, this),
        this->service_status_timer_cb_group_);

    this->mosquitto_status_publisher_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions mosquitto_status_publisher_opts;
    mosquitto_status_publisher_opts.callback_group = this->mosquitto_status_publisher_cb_group_;
    this->mosquitto_status_publisher_ = this->node_->create_publisher<std_msgs::msg::Bool>(
        MOSQUITTO_STATUS_PUBLISHER_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(1)),
        mosquitto_status_publisher_opts);
}

r2ps::system::ServiceManager::~ServiceManager()
{
}

void r2ps::system::ServiceManager::service_status_timer_cb()
{
    try
    {
        std_msgs::msg::Bool::UniquePtr mosquitto_status = std::make_unique<std_msgs::msg::Bool>();
        const std::string &mosquitto_status_output = this->r2ps_process_utils_->execute_command(MOSQUITTO_STATUS_COMMAND);

        if (mosquitto_status_output != "")
        {
            std::vector<std::string> lines = this->r2ps_string_utils_->split(mosquitto_status_output, '\n');

            for (const std::string &line : lines)
            {
                if (line.find("Active:") != std::string::npos)
                {
                    std::vector<std::string> words = this->r2ps_string_utils_->split(line, ' ');
                    words.erase(std::remove(words.begin(), words.end(), ""), words.end());

                    if (words.size() > 1 && words[0] == "Active:")
                    {
                        if (words[1] == "active" && words[2] == "(running)")
                        {
                            mosquitto_status->set__data(true);
                        }
                        else
                        {
                            this->restart_mosquitto_service();
                            mosquitto_status->set__data(false);
                        }
                        break;
                    }
                }
            }
        }
        else
        {
            RCLCPP_WARN(this->node_->get_logger(), "!!!!! Mosquitto Has Died.. !!!!!");
            this->restart_mosquitto_service();
            mosquitto_status->set__data(false);
        }
        this->mosquitto_status_publisher_->publish(std::move(*mosquitto_status));
    }
    catch (const std::exception &expn)
    {
        RCLCPP_ERROR(this->node_->get_logger(), "%s", expn.what());
    }
}

void r2ps::system::ServiceManager::restart_mosquitto_service()
{
    try
    {
        const std::string &restart_mosquitto_service_ouptut = this->r2ps_process_utils_->execute_command(MOSQUITTO_RESTART_COMMAND);

        if (restart_mosquitto_service_ouptut != "")
        {
            RCLCPP_INFO(this->node_->get_logger(), "===== Mosquitto Restarted =====");
        }
    }
    catch (const std::exception &expn)
    {
        RCLCPP_ERROR(this->node_->get_logger(), "%s", expn.what());
    }
}