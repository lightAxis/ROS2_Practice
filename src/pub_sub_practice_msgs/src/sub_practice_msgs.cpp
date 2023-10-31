#include <rclcpp/rclcpp.hpp>

#include <practice_msgs/msg/test_msg1.hpp>
#include <practice_msgs/msg/test_msg2.hpp>
#include <practice_msgs/msg/test_msg3.hpp>
#include <practice_msgs/msg/test_msg4.hpp>
#include <practice_msgs/msg/test_msg5.hpp>

using namespace practice_msgs::msg;

class Sub_PracticeMsg : public rclcpp::Node
{
public:
    Sub_PracticeMsg(rclcpp::NodeOptions nodeOptions = rclcpp::NodeOptions{}) : Node{"sub_practice_msgs_node", nodeOptions}
    {
        _sub_testmsg1 = this->create_subscription<TestMsg1>("test_msg1", rclcpp::SystemDefaultsQoS{}, std::bind(&Sub_PracticeMsg::callback_msg1, this, std::placeholders::_1, std::placeholders::_2));
        _sub_testmsg2 = this->create_subscription<TestMsg2>("test_msg2", rclcpp::SystemDefaultsQoS{}, std::bind(&Sub_PracticeMsg::callback_msg2, this, std::placeholders::_1));
        _sub_testmsg3 = this->create_subscription<TestMsg3>("test_msg3", rclcpp::SystemDefaultsQoS{}, std::bind(&Sub_PracticeMsg::callback_msg3, this, std::placeholders::_1));
        _sub_testmsg4 = this->create_subscription<TestMsg4>("test_msg4", rclcpp::SystemDefaultsQoS{}, std::bind(&Sub_PracticeMsg::callback_msg4, this, std::placeholders::_1));
        _sub_testmsg5 = this->create_subscription<TestMsg5>("test_msg5", rclcpp::SystemDefaultsQoS{}, std::bind(&Sub_PracticeMsg::callback_msg5, this, std::placeholders::_1));

        _sub_testmsg5_1 = this->create_subscription<TestMsg5>("test_msg5", rclcpp::SystemDefaultsQoS{}, std::bind(&Sub_PracticeMsg::callback_msg5_1, this, std::placeholders::_1));
        _sub_testmsg5_2 = this->create_subscription<TestMsg5>("test_msg5", rclcpp::SystemDefaultsQoS{}, std::bind(&Sub_PracticeMsg::callback_msg5_2, this, std::placeholders::_1));
    }

    // these are allowed callback function types

    void callback_msg1(TestMsg1 msg, const rclcpp::MessageInfo &info)
    {
        // info.get_rmw_message_info().from_intra_process
        std::cout << "msg1 recieved! " << msg.the_bool << std::endl;
    }
    void callback_msg2(const TestMsg2 msg)
    {
        std::cout << "msg2 recieved! " << msg.string_of_unbounded_size << std::endl;
    }
    void callback_msg3(const TestMsg3 &msg)
    {
        std::cout << "msg3 arrived! " << msg.vec_array_10[0].x << std::endl;
    }
    void callback_msg4(TestMsg4::SharedPtr msg)
    {
        std::cout << "msg4 arrived!" << msg->prev_msg.the_bool << std::endl;
    }
    void callback_msg5(TestMsg5::ConstSharedPtr msg)
    {
        std::cout << "msg5 arrived!" << msg->this_is_enum << std::endl;
    }

    void callback_msg5_1(TestMsg5::UniquePtr msg)
    {
        std::cout << "function msg5_1 changing value" << std::endl;
        msg->this_is_enum = 100;
        std::cout << "function msg5_1 changed value!" << std::endl;

        std::cout << "msg5 unique ptr " << static_cast<int>(msg->this_is_enum) << std::endl;
        msg.release();
    }
    void callback_msg5_2(const TestMsg5::ConstSharedPtr &msg)
    {
        std::cout << "function msg5_2 " << static_cast<int>(msg->this_is_enum) << std::endl;
    }

    // the available callback function candidates are,
    //     candidate: ‘std::variant<_Types>& std::variant<_Types>::operator=(std::variant<_Types>&&)
    // [with _Types = {
    //   std::function<void(const practice_msgs::msg::TestMsg4_<std::allocator<void> >&)>,
    //   std::function<void(const practice_msgs::msg::TestMsg4_<std::allocator<void> >&, const rclcpp::MessageInfo&)>,
    //   std::function<void(const rclcpp::SerializedMessage&)>,
    //   std::function<void(const rclcpp::SerializedMessage&, const rclcpp::MessageInfo&)>,
    //   std::function<void(std::unique_ptr<practice_msgs::msg::TestMsg4_<std::allocator<void> >,
    //   std::default_delete<practice_msgs::msg::TestMsg4_<std::allocator<void> > > >)>,
    //   std::function<void(std::unique_ptr<practice_msgs::msg::TestMsg4_<std::allocator<void> >,
    //   std::default_delete<practice_msgs::msg::TestMsg4_<std::allocator<void> > > >, const rclcpp::MessageInfo&)>,
    //   std::function<void(std::unique_ptr<rclcpp::SerializedMessage,
    //   std::default_delete<rclcpp::SerializedMessage> >)>,
    //   std::function<void(std::unique_ptr<rclcpp::SerializedMessage,
    //   std::default_delete<rclcpp::SerializedMessage> >, const rclcpp::MessageInfo&)>,
    //   std::function<void(std::shared_ptr<const practice_msgs::msg::TestMsg4_<std::allocator<void> > >)>,
    //   std::function<void(std::shared_ptr<const practice_msgs::msg::TestMsg4_<std::allocator<void> > >, const rclcpp::MessageInfo&)>,
    //   std::function<void(std::shared_ptr<const rclcpp::SerializedMessage>)>,
    //   std::function<void(std::shared_ptr<const rclcpp::SerializedMessage>, const rclcpp::MessageInfo&)>,
    //   std::function<void(const std::shared_ptr<const practice_msgs::msg::TestMsg4_<std::allocator<void> > >&)>,
    //   std::function<void(const std::shared_ptr<const practice_msgs::msg::TestMsg4_<std::allocator<void> > >&, const rclcpp::MessageInfo&)>,
    //   std::function<void(const std::shared_ptr<const rclcpp::SerializedMessage>&)>,
    //   std::function<void(const std::shared_ptr<const rclcpp::SerializedMessage>&, const rclcpp::MessageInfo&)>,
    //   std::function<void(std::shared_ptr<practice_msgs::msg::TestMsg4_<std::allocator<void> > >)>,
    //   std::function<void(std::shared_ptr<practice_msgs::msg::TestMsg4_<std::allocator<void> > >, const rclcpp::MessageInfo&)>,
    //   std::function<void(std::shared_ptr<rclcpp::SerializedMessage>)>,
    //   std::function<void(std::shared_ptr<rclcpp::SerializedMessage>, const rclcpp::MessageInfo&)>}]’

private:
    rclcpp::Subscription<TestMsg1>::SharedPtr _sub_testmsg1{nullptr};
    rclcpp::Subscription<TestMsg2>::SharedPtr _sub_testmsg2{nullptr};
    rclcpp::Subscription<TestMsg3>::SharedPtr _sub_testmsg3{nullptr};
    rclcpp::Subscription<TestMsg4>::SharedPtr _sub_testmsg4{nullptr};
    rclcpp::Subscription<TestMsg5>::SharedPtr _sub_testmsg5{nullptr};

    rclcpp::Subscription<TestMsg5>::SharedPtr _sub_testmsg5_1{nullptr};
    rclcpp::Subscription<TestMsg5>::SharedPtr _sub_testmsg5_2{nullptr};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Sub_PracticeMsg>());
    rclcpp::shutdown();
    return 0;
}