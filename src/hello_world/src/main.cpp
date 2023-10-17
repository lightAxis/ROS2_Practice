// to using std::cout
#include <iostream>

// wrapping time structs (sec,ms, hour ..) and provide literal namespace
#include <chrono>

// wrapping function pointer and provide function object
#include <functional>

// basic header file must need to use ROS2 c++ library
#include <rclcpp/rclcpp.hpp>

// using chrono_literals, add various user literal like _ms, _s, _ns..
using namespace std::chrono_literals;

// In ROS2, Class inherited frome rclcpp::Node is considered as Node object constructor or class
class HelloWorldNode : public rclcpp::Node
{
public:
    // Constructor of this Node, HelloWorldNode.
    // hello_world_node is the node name for this node
    HelloWorldNode() : rclcpp::Node{"hello_world_node"}
    {
        // as rclcpp::Node has function create_wall_timer()
        // 1000ms is duration to call.
        // set the callback function using std::bind
        // result is shard_ptr of timerbase
        _timer = this->create_wall_timer(1000ms, std::bind(&HelloWorldNode::timer_callback, this));
    }

private:
    // timer callback function
    void timer_callback()
    {
        std::cout << "Hello World!" << std::endl;
    }

    rclcpp::TimerBase::SharedPtr _timer;
};

int main(int argc, char **argv)
{
    // init this rclcpp node with argc and argv from ros system
    rclcpp::init(argc, argv);

    // start spin the node that passed by shared_ptr form
    rclcpp::spin(std::make_shared<HelloWorldNode>());

    // when the node finished, manually shutdown this node thread
    rclcpp::shutdown();

    return 0;
}