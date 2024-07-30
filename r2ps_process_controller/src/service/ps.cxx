#include "service/ps.hxx"

r2ps::process::PSService::PSService(rclcpp::Node::SharedPtr node)
{
    this->node_ = node;
    RCLCPP_INFO(this->node_->get_logger(), "%s registered", "ps service");

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

std::vector<std::string> r2ps::process::PSService::split(const std::string &str, char delimiter)
{
    std::vector<std::string> tokens;
    std::stringstream ss(str);
    std::string token;
    while (std::getline(ss, token, delimiter))
    {
        tokens.push_back(token);
    }
    return tokens;
}

std::string r2ps::process::PSService::exec_command(const char *command)
{
    try
    {
        const int &command_result = system(command);

        if (command_result == 0)
        {
            FILE *pipe = popen(command, "r");
            if (!pipe)
            {
                RCLCPP_ERROR(this->node_->get_logger(), "Failed to run command: %s", command);
                return "";
            }

            char buffer[128];
            std::string result = "";

            while (fgets(buffer, sizeof(buffer), pipe) != nullptr)
            {
                result += buffer;
            }

            const int &exit_status = pclose(pipe);

            return result;
        }
        else
        {
            RCLCPP_ERROR(this->node_->get_logger(), "Failed to run command: %s", command);
            return "";
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->node_->get_logger(), "Failed to run command: %s", command);
        return "";
    }
}

void r2ps::process::PSService::process_check_timer_cb()
{
    try
    {
        r2ps_msgs::msg::ProcessList::UniquePtr process_list = std::make_unique<r2ps_msgs::msg::ProcessList>();
        std::vector<r2ps_msgs::msg::Process> processes;

        const char *ros_process_command = "ps aux | grep ros";
        const std::string ros_process_output = this->exec_command(ros_process_command);

        if (ros_process_output != "")
        {
            std::vector<std::string> lines = split(ros_process_output, '\n');

            for (const std::string &line : lines)
            {
                if (line.find("r2ps") != std::string::npos ||
                    line.find("_ros2") != std::string::npos ||
                    line.find("daemon") != std::string::npos ||
                    line.find("echo") != std::string::npos ||
                    line.find("grep") != std::string::npos)
                {
                    continue;
                }

                std::vector<std::string> words = split(line, ' ');
                words.erase(std::remove(words.begin(), words.end(), ""), words.end());

                if (words.size() >= 1)
                {
                    r2ps_msgs::msg::Process::UniquePtr process = std::make_unique<r2ps_msgs::msg::Process>();

                    const std::string &process_id = words[1];
                    const std::string &process_name = words[11];

                    process->set__pid(std::stoi(process_id));
                    process->set__pname(process_name);
                    processes.push_back(std::move(*process));
                }

                process_list->set__process_list(processes);
                this->process_list_publisher_->publish(std::move(*process_list));
            }
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
}