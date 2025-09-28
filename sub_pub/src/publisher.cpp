#include <string>
#include <chrono>           // time library
#include <memory>           // library used to manage dynamic memory
#include <functional>       // provides a set of predifined class templates for function objects

#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class SimplePublisher : public rclcpp::Node
{
    public:
        SimplePublisher()
        : Node("publisher_node")
        {
            CounterValue = 0;
            publisherObject = this->create_publisher<std_msgs::msg::String>("communication_topic", 20);
            timerObject = this->create_wall_timer(1000ms, std::bind(&SimplePublisher::callBackFunction, this));
        }

    private:
        void callBackFunction()
        {
            CounterValue++;
            auto message = std_msgs::msg::String();
            message.data = "Message Number: " + std::to_string(CounterValue);

            RCLCPP_INFO(this->get_logger(), "Publishing message: '%s'", message.data.c_str());
            publisherObject->publish(message);
        }
        rclcpp::TimerBase::SharedPtr timerObject;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisherObject;
        int CounterValue;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimplePublisher>());
    rclcpp::shutdown();
    return 0;
}
