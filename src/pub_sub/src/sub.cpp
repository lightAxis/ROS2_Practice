#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

// subscriber node
class Subscriber : public rclcpp::Node
{
public:
    // add node options initializer
    Subscriber(rclcpp::NodeOptions nodeOptions = rclcpp::NodeOptions()) : Node{"Subscriber_node", nodeOptions}
    {
        // init subscriber
        _sub_testMsg = this->create_subscription<std_msgs::msg::Bool>("test_topic_pubsub", 10,
                                                                      std::bind(&Subscriber::test_callback, this, std::placeholders::_1));
    }

private:
    // callback function.
    // can use const Bool&, Bool, Bool::ConstPtr etc..
    // cannot use Bool&, *Bool
    void test_callback(const std_msgs::msg::Bool &msg)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "recieved!" << msg.data);
    }

        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _sub_testMsg{nullptr};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Subscriber>());
    rclcpp::shutdown();
    return 0;
}