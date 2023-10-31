#include <rclcpp/rclcpp.hpp>
#include <thread>

#include <practice_msgs/srv/test_srv1.hpp>

using namespace practice_msgs::srv;
class Client_Test : public rclcpp::Node
{
public:
    Client_Test(rclcpp::NodeOptions nodeOptions = rclcpp::NodeOptions{}) : Node{"client_test_node", nodeOptions}
    {

        static_assert(static_cast<uint8_t>(Operator::MAX) == practice_msgs::srv::TestSrv1::Request::CALCULATION_TYPE_MAX, "enum unmet with msg definition!");

        // create a client
        _client_test_srv1 = this->create_client<TestSrv1>("client_test_srv_1");

        // timer 1 times a sec
        _timer = this->create_wall_timer(std::chrono::milliseconds{1000}, std::bind(&Client_Test::timer_callback, this));
    }

    enum class Operator
    {
        Plus = 0,
        Minus = 1,
        MAX = 2,
    };

    void timer_callback()
    {
        // initiate a new thread for this
        // because send_request() function has std::future::wait_for() function. and this will block the main node spin thread

        // start the instance threade and detach
        std::thread(&Client_Test::send_request, this, 2, 4, Operator::Minus).detach();
    }

private:
    void send_request(float a, float b, Operator op)
    {

        // the result if wait_for_service() function is , true : there's a service server exist for this client /  false : not exist
        // so, this means if server for this service is detected, than send request to it
        if (!_client_test_srv1->wait_for_service(std::chrono::milliseconds(100)))
        {
            RCLCPP_ERROR(this->get_logger(), "no service server is available now");
            return;
        }

        // init new message with make_shared
        auto req = std::make_shared<TestSrv1::Request>();

        // fill with contents
        req->header.stamp = this->get_clock()->now();
        req->a = a;
        req->b = b;
        req->calculation_type = cvt_Operator_to_uint8(op);
        std::cout << "send request " << a << b << static_cast<int>(op) << std::endl;

        // sending with async function.
        // return is clcpp::Client<practice_msgs::srv::TestSrv1>::SharedFutureAndRequestId
        // callback function is rclcpp::Client<TestSrv1>::SharedFuture
        // the callback function will called after response recieved.
        // so if we want to implement a timeout logic, we have to deal directly with result SharedFuture
        auto asdf = _client_test_srv1->async_send_request(req, std::bind(&Client_Test::response_callback, this, std::placeholders::_1));

        // wait maximum 500ms until entangled std::promise fill the value to std::shared_future
        // if filled, this line will immidiatly terminate and goto next line, result std::future_status::ready
        // if not filled untill 500ms passed, skip this line, and result std::future_status::timeout
        auto wait_res = asdf.wait_for(std::chrono::milliseconds{500});

        // no response in 500ms, give up
        if (wait_res == std::future_status::timeout)
        {
            std::cout << "no response for 500ms, id:" << asdf.request_id << std::endl;
            return;
        }

        // there was a response in 500ms, print the value
        TestSrv1::Response::SharedPtr aa = asdf.get();
        std::cout
            << "request recieved!~ " << aa->c << std::endl;
    }

    // this is a client callback function
    // when client recieves a response from server, this function will be triggered
    // be carefull not to block a main spin thread of callback node or thread
    void response_callback(rclcpp::Client<TestSrv1>::SharedFuture res)
    {
        TestSrv1::Response::SharedPtr aa = res.get();
        // auto response = res;
        // std::cout << "response get! " << response.c << std::endl;
        std::cout << "callback response recieved! " << aa->c << std::endl;
    }

    rclcpp::Client<TestSrv1>::SharedPtr _client_test_srv1{nullptr};
    rclcpp::TimerBase::SharedPtr _timer{nullptr};

    uint8_t cvt_Operator_to_uint8(const Operator &op)
    {
        using REQ = practice_msgs::srv::TestSrv1::Request;

        switch (op)
        {
        case Operator::Plus:
            return REQ::CALCULATION_TYPE_PLUS;
        case Operator::Minus:
            return REQ::CALCULATION_TYPE_MINUS;
        case Operator::MAX:
            return REQ::CALCULATION_TYPE_MAX;
        }

        return REQ::CALCULATION_TYPE_MAX;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Client_Test>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}