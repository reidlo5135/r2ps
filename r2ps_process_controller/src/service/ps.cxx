#include "service/ps.hxx"

r2ps::process::PSService::PSService(rclcpp::Node::SharedPtr node)
{
    this->node_ = node;
    RCLCPP_INFO(this->node_->get_logger(), "%s registered", "ps service");

    this->r2ps_string_utils_ = std::make_shared<r2ps::utils::String>();
    this->r2ps_process_utils_ = std::make_shared<r2ps::utils::Process>();

    this->process_check_timer_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->process_check_timer_ = this->node_->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&r2ps::process::PSService::process_check_timer_cb, this),
        this->process_check_timer_cb_group_);

    this->process_list_publisher_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions process_list_publisher_opts;
    process_list_publisher_opts.callback_group = this->process_list_publisher_cb_group_;
    this->process_list_publisher_ = this->node_->create_publisher<r2ps_msgs::msg::ProcessList>(
        PROCESS_LIST_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(1)),
        process_list_publisher_opts);
}

r2ps::process::PSService::~PSService()
{
}

void r2ps::process::PSService::process_check_timer_cb()
{
    try
    {
        r2ps_msgs::msg::ProcessList::UniquePtr process_list = std::make_unique<r2ps_msgs::msg::ProcessList>();
        std::vector<r2ps_msgs::msg::Process> process_vec;

        const char *ros_process_command = "ps aux | grep ros";
        const std::string ros_process_output = this->r2ps_process_utils_->execute_command(ros_process_command);

        if (ros_process_output != "")
        {
            std::vector<std::string> lines = this->r2ps_string_utils_->split(ros_process_output, '\n');

            for (const std::string &line : lines)
            {
                if (line.find("r2ps") != std::string::npos ||
                    line.find("_ros2") != std::string::npos ||
                    line.find("daemon") != std::string::npos ||
                    line.find("echo") != std::string::npos ||
                    line.find("grep") != std::string::npos ||
                    line.find("/opt/ros/") != std::string::npos)
                {
                    continue;
                }

                std::vector<std::string> words = this->r2ps_string_utils_->split(line, ' ');
                words.erase(std::remove(words.begin(), words.end(), ""), words.end());

                r2ps_msgs::msg::Process::UniquePtr process = std::make_unique<r2ps_msgs::msg::Process>();

                if (words.size() >= 1)
                {
                    const int32_t &process_id = this->r2ps_string_utils_->parse_int(words[1]);
                    const std::string &process_name = words[11];

                    process->set__pid(process_id);
                    process->set__pname(process_name);
                    process_vec.push_back(std::move(*process));
                }
            }

            process_list->set__process_list(std::move(process_vec));
            this->process_list_publisher_->publish(std::move(*process_list));
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->node_->get_logger(), "Failed to run command: %s", e.what());
    }
}