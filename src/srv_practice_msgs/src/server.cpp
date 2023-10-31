#include <rclcpp/rclcpp.hpp>

#include <practice_msgs/srv/test_srv1.hpp>

using namespace practice_msgs::srv;
using namespace std::placeholders;

class Server_Test : public rclcpp::Node
{
public:
    Server_Test(rclcpp::NodeOptions nodeOptions = rclcpp::NodeOptions{}) : Node{"server_test_node", nodeOptions}
    {
        _service_test_srv1 = this->create_service<TestSrv1>("client_test_srv_1", std::bind(&Server_Test::callback_service, this, ::_1, ::_2));
    }

private:
    // this is a callback function for service
    // only this type, const shared_ptr request , shared_ptr response is available for callback function of service server
    void callback_service(TestSrv1::Request::ConstSharedPtr req_ptr, TestSrv1::Response::SharedPtr res_ptr)
    {

        // the grammer -> is sucks, so just change them in form of reference
        const auto &req = *req_ptr.get();
        auto &res = *res_ptr.get();

        std::cout << "req recieved, " << req.a << req.b << req.calculation_type << std::endl;

        switch (req.calculation_type)
        {
        case TestSrv1::Request::CALCULATION_TYPE_MINUS:
            res.c = req.a - req.b;
            break;
        case TestSrv1::Request::CALCULATION_TYPE_PLUS:
            res.c = req.a + req.b;
            break;
        case TestSrv1::Request::CALCULATION_TYPE_MAX:
            res.c = 100;
            break;
        }
        std::cout << "returning value is : " << res.c << std::endl;
    }
    rclcpp::Service<TestSrv1>::SharedPtr _service_test_srv1{nullptr};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Server_Test>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}