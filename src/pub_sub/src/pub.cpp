#include <iostream>
#include <functional>
#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <std_msgs/msg/bool.hpp>

class Publisher : public rclcpp::Node
{
public:
    Publisher(rclcpp::NodeOptions nodeOptions = rclcpp::NodeOptions()) : Node{"publisher_node", nodeOptions}
    {
        // make various qos setting in here
        // auto qos_profile_default = rclcpp::SystemDefaultsQoS{};
        auto qos_profile_custom = rclcpp::QoS{rclcpp::KeepLast{10}};

        // _pub_testMsg = this->create_publisher<std_msgs::msg::Bool>("test_topic_pubsub", qos_profile_default);
        _pub_testMsg = this->create_publisher<std_msgs::msg::Bool>("test_topic_pubsub", qos_profile_custom);

        _timer = this->create_wall_timer(std::chrono::milliseconds{1000}, std::bind(&Publisher::callback, this));
        auto asdf = _timer;
        RCLCPP_INFO_STREAM(this->get_logger(), std::to_string(asdf.use_count()) + "/" + std::to_string(_timer.use_count()));
    }

private:
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pub_testMsg{nullptr};

    rclcpp::TimerBase::SharedPtr _timer{nullptr};

    void callback()
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "this is test!!!");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Publisher>());
    rclcpp::shutdown();

    return 0;
}