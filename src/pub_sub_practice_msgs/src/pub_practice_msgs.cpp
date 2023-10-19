#include <rclcpp/rclcpp.hpp>

#include <practice_msgs/msg/test_msg1.hpp>
#include <practice_msgs/msg/test_msg2.hpp>
#include <practice_msgs/msg/test_msg3.hpp>
#include <practice_msgs/msg/test_msg4.hpp>

class Pub_PracticeMsg : public rclcpp::Node
{
public:
    Pub_PracticeMsg(rclcpp::NodeOptions nodeOptions = rclcpp::NodeOptions{}) : Node{"pub_practice_msgs_node", nodeOptions}
    {

        _pub_msg1 = this->create_publisher<practice_msgs::msg::TestMsg1>("test_msg1", 10);
        _pub_msg2 = this->create_publisher<practice_msgs::msg::TestMsg2>("test_msg2", 10);
        _pub_msg3 = this->create_publisher<practice_msgs::msg::TestMsg3>("test_msg3", 10);
        _pub_msg4 = this->create_publisher<practice_msgs::msg::TestMsg4>("test_msg4", 10);

        _timer = this->create_wall_timer(std::chrono::milliseconds{1000}, std::bind(&Pub_PracticeMsg::timer_callback, this));
    }

private:
    void timer_callback()
    {
        // msg1

        using namespace practice_msgs::msg;
        TestMsg1 msg1{};
        msg1.the_bool = false;
        msg1.the_double_array_10.at(3) = 1;
        msg1.the_float_vector.resize(10);
        msg1.the_string = "this is streing!";

        _pub_msg1->publish(msg1);
    }
    rclcpp::Publisher<practice_msgs::msg::TestMsg1>::SharedPtr _pub_msg1{nullptr};
    rclcpp::Publisher<practice_msgs::msg::TestMsg2>::SharedPtr _pub_msg2{nullptr};
    rclcpp::Publisher<practice_msgs::msg::TestMsg3>::SharedPtr _pub_msg3{nullptr};
    rclcpp::Publisher<practice_msgs::msg::TestMsg4>::SharedPtr _pub_msg4{nullptr};

    rclcpp::TimerBase::SharedPtr _timer{nullptr};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Pub_PracticeMsg>());
    rclcpp::shutdown();
    return 0;
}