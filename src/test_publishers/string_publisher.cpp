#include "rclcpp/rclcpp.hpp"  
#include "std_msgs/msg/string.hpp"  

using namespace std; // Use the std namespace for convenience  

class StringPublisher : public rclcpp::Node  
{  
public:  
  StringPublisher()  
  : Node("string_publisher")  
  {  
    // Declare parameters for topic, publish rate, and message prefix  
    this->declare_parameter("topic", "/trigger_topic");  
    this->declare_parameter("publish_rate", 1.0);  
    this->declare_parameter("message_prefix", "Request");  
    
    // Retrieve parameter values  
    string topic = this->get_parameter("topic").as_string();  
    double publish_rate = this->get_parameter("publish_rate").as_double();  
    message_prefix_ = this->get_parameter("message_prefix").as_string();  
    
    // Create a publisher on the specified topic  
    publisher_ = this->create_publisher<std_msgs::msg::String>(topic, 10);  
    
    // Create a timer that triggers the publish_message method at the specified rate  
    timer_ = this->create_wall_timer(  
      chrono::duration<double>(1.0 / publish_rate),  
      bind(&StringPublisher::publish_message, this));  
    
    RCLCPP_INFO(this->get_logger(), "String publisher started, publishing to '%s' at %.1f Hz",  
                topic.c_str(), publish_rate);  
  }  

private:  
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_; // Publisher for string messages  
  rclcpp::TimerBase::SharedPtr timer_; // Timer for publishing messages  
  string message_prefix_; // Prefix for the messages  
  int count_ = 0; // Counter for message numbering  
  
  void publish_message()  
  {  
    // Create a new message  
    auto message = std_msgs::msg::String();  
    message.data = message_prefix_ + " #" + to_string(++count_);  
    
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());  
    publisher_->publish(message); // Publish the message  
  }  
};  

int main(int argc, char * argv[])  
{  
  rclcpp::init(argc, argv); // Initialize the ROS 2 communication  
  auto node = make_shared<StringPublisher>(); // Create an instance of StringPublisher  
  rclcpp::spin(node); // Spin the node to keep it running  
  rclcpp::shutdown(); // Shutdown ROS 2 communication cleanly  
  return 0;  
}  