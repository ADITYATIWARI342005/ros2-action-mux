#include "rclcpp/rclcpp.hpp"  
#include "std_msgs/msg/int64.hpp"  

using namespace std; // Use the std namespace for convenience  

class IntPublisher : public rclcpp::Node  
{  
public:  
  IntPublisher()  
  : Node("int_publisher")  
  {  
    // Declare parameters for topic, publish rate, start value, and increment  
    this->declare_parameter("topic", "/int_topic");  
    this->declare_parameter("publish_rate", 1.0);  
    this->declare_parameter("start_value", 0);  
    this->declare_parameter("increment", 1);  
    
    // Retrieve parameter values  
    string topic = this->get_parameter("topic").as_string();  
    double publish_rate = this->get_parameter("publish_rate").as_double();  
    current_value_ = this->get_parameter("start_value").as_int();  
    increment_ = this->get_parameter("increment").as_int();  
    
    // Create a publisher for Int64 messages on the specified topic  
    publisher_ = this->create_publisher<std_msgs::msg::Int64>(topic, 10);  
    
    // Create a timer that calls the publish_message method at the specified rate  
    timer_ = this->create_wall_timer(  
      chrono::duration<double>(1.0 / publish_rate),  
      bind(&IntPublisher::publish_message, this));  
    
    RCLCPP_INFO(this->get_logger(), "Int publisher started, publishing to '%s' at %.1f Hz",  
                topic.c_str(), publish_rate);  
  }  

private:  
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr publisher_; // Publisher for Int64 messages  
  rclcpp::TimerBase::SharedPtr timer_; // Timer for publishing messages  
  int current_value_; // Current value to be published  
  int increment_; // Amount to increment the current value each time  

  void publish_message()  
  {  
    // Create a new message and assign the current value  
    auto message = std_msgs::msg::Int64();  
    message.data = current_value_;  
    
    // Increment the current value after publishing  
    current_value_ += increment_;  
    
    RCLCPP_INFO(this->get_logger(), "Publishing: %ld", message.data);  
    publisher_->publish(message); // Publish the message  
  }  
};  

int main(int argc, char * argv[])  
{  
  rclcpp::init(argc, argv); // Initialize ROS 2 communication  
  auto node = make_shared<IntPublisher>(); // Create an instance of IntPublisher  
  rclcpp::spin(node); // Spin the node to keep it active  
  rclcpp::shutdown(); // Shutdown ROS 2 communication cleanly  
  return 0;  
}  