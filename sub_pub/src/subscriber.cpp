#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

class SimpleSubscriber : public rclcpp::Node
{
    public:
        SimpleSubscriber()
        : Node("subscriber_node")
        {
            subscriberObject = this->create_subscription<std_msgs::msg::String>(
            "communication_topic", 10, std::bind(&SimpleSubscriber::callBackFunction, this, _1));
        }

    private:
        void callBackFunction(const std_msgs::msg::String & msg) const
        {
            RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
        }
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriberObject;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleSubscriber>());
    rclcpp::shutdown();
    return 0;
}
