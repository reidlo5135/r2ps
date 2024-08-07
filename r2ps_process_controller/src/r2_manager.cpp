#include "r2_manager.hpp"

r2ps::process::R2Service::R2Service(rclcpp::Node::SharedPtr node)
{
    this->node_ = node;
    RCLCPP_INFO(this->node_->get_logger(), "%s registered", "r2 service");

    this->current_node_check_timer_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->current_node_check_timer_ = this->node_->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&r2ps::process::R2Service::current_node_check_timer_cb, this),
        this->current_node_check_timer_cb_group_);

    this->node_list_publisher_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions node_list_publisher_opts;
    node_list_publisher_opts.callback_group = this->node_list_publisher_cb_group_;
    this->node_list_publisher_ = this->node_->create_publisher<r2ps_msgs::msg::NodeList>(
        NODE_LIST_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(10)),
        node_list_publisher_opts);
}

r2ps::process::R2Service::~R2Service()
{
}

void r2ps::process::R2Service::current_node_check_timer_cb()
{
    try
    {
        const std::vector<std::string> &current_node_vec = this->node_->get_node_names();
        std::vector<std::string> filtered_node_vec;

        std::copy_if(
            current_node_vec.begin(), current_node_vec.end(), std::back_inserter(filtered_node_vec),
            [](const std::string &current_node)
            {
                const bool &contains_r2ps = current_node.find("r2ps") != std::string::npos;
                const bool &contains_ros2 = current_node.find("_ros2") != std::string::npos;
                const bool &contains_daemon = current_node.find("daemon") != std::string::npos;

                return !(contains_r2ps || contains_ros2 || contains_daemon);
            });

        r2ps_msgs::msg::NodeList::UniquePtr node_list = std::make_unique<r2ps_msgs::msg::NodeList>();
        node_list->set__node_list(std::move(filtered_node_vec));

        for (const std::string &node : node_list->node_list)
        {
            RCLCPP_INFO(this->node_->get_logger(), "%s", node.c_str());
        }
        this->node_list_publisher_->publish(std::move(*node_list));
    }
    catch (const std::exception &expn)
    {
        RCLCPP_ERROR(this->node_->get_logger(), "%s", expn.what());
    }
    catch (const rclcpp::exceptions::RCLError &rcl_expn)
    {
        RCLCPP_ERROR(this->node_->get_logger(), "%s", rcl_expn.what());
    }
}