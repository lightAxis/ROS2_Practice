#include <rclcpp/rclcpp.hpp>

#include <practice_msgs/msg/test_msg1.hpp>
#include <practice_msgs/msg/test_msg2.hpp>
#include <practice_msgs/msg/test_msg3.hpp>
#include <practice_msgs/msg/test_msg4.hpp>
#include <practice_msgs/msg/test_msg5.hpp>

class Pub_PracticeMsg : public rclcpp::Node
{
public:
    Pub_PracticeMsg(rclcpp::NodeOptions nodeOptions = rclcpp::NodeOptions{}) : Node{"pub_practice_msgs_node", nodeOptions}
    {

        _pub_msg1 = this->create_publisher<practice_msgs::msg::TestMsg1>("test_msg1", 10);
        _pub_msg2 = this->create_publisher<practice_msgs::msg::TestMsg2>("test_msg2", 10);
        _pub_msg3 = this->create_publisher<practice_msgs::msg::TestMsg3>("test_msg3", 10);
        _pub_msg4 = this->create_publisher<practice_msgs::msg::TestMsg4>("test_msg4", 10);
        _pub_msg5 = this->create_publisher<practice_msgs::msg::TestMsg5>("test_msg5", 10);

        _timer = this->create_wall_timer(std::chrono::milliseconds{1000}, std::bind(&Pub_PracticeMsg::timer_callback, this));
    }

private:
    void timer_callback()
    {
        // msg1
        using namespace practice_msgs::msg;
        TestMsg1 msg1;
        {
            TestMsg1 msg{};
            msg.the_bool = false;
            msg.the_double_array_10.at(3) = 1;
            msg.the_float_vector.resize(10);
            msg.the_float_vector[2] = 3;
            msg.the_string = "this is streing!";
            _pub_msg1->publish(msg);
            msg1 = msg;
        }

        // msg2
        {
            TestMsg2 msg{};
            msg.string_of_unbounded_size = "this is unbounded!";
            msg.five_integers_array[2] = 3;

            msg.up_to_ten_characters_string = "123";
            msg.up_to_five_strings_up_to_ten_characters_e.resize(2);
            msg.up_to_five_strings_up_to_ten_characters_e[1] = "asdf"; // what is the BoundedVector..
            _pub_msg2->publish(msg);
        }

        // msg3
        {
            TestMsg3 msg{};
            msg.header.frame_id = "this_is_frame_ID";
            msg.header.stamp = this->now() - rclcpp::Duration{std::chrono::milliseconds{1000}};
            msg.vec_single = make_vector3(1, 2, 3);
            _pub_msg3->publish(msg);
        }

        // msg4
        {
            TestMsg4 msg{};
            msg.header = std_msgs::msg::Header{};
            msg.prev_msg = msg1;
            _pub_msg4->publish(msg);
        }

        // msg 5
        {
            TestMsg5 msg{};
            msg.this_is_enum = TestMsg5::THIS_IS_ENUM_FIRST;
            msg.this_is_enum = TestMsg5::THIS_IS_ENUM_SECOND;
            msg.this_is_enum = TestMsg5::THIS_IS_ENUM_LAST;
            _pub_msg5->publish(msg);
        }
    }
    rclcpp::Publisher<practice_msgs::msg::TestMsg1>::SharedPtr _pub_msg1{nullptr};
    rclcpp::Publisher<practice_msgs::msg::TestMsg2>::SharedPtr _pub_msg2{nullptr};
    rclcpp::Publisher<practice_msgs::msg::TestMsg3>::SharedPtr _pub_msg3{nullptr};
    rclcpp::Publisher<practice_msgs::msg::TestMsg4>::SharedPtr _pub_msg4{nullptr};
    rclcpp::Publisher<practice_msgs::msg::TestMsg5>::SharedPtr _pub_msg5{nullptr};

    rclcpp::TimerBase::SharedPtr _timer{nullptr};

    geometry_msgs::msg::Vector3 make_vector3(const double &x, const double &y, const double &z)
    {
        geometry_msgs::msg::Vector3 vec{};
        vec.x = x;
        vec.y = y;
        vec.z = z;
        return vec;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Pub_PracticeMsg>());
    rclcpp::shutdown();
    return 0;
}