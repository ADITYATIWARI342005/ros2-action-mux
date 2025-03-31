#include <rclcpp/rclcpp.hpp>  
#include <std_msgs/msg/string.hpp>  

// Use namespaces to simplify code readability  
using namespace std;  

// Class for publishing string messages to a ROS 2 topic  
class StringPublisher : public rclcpp::Node {  
public:  
    // Constructor for the StringPublisher node  
    StringPublisher() : Node("string_publisher") {  
        // Create a publisher to send String messages on the "/string_topic" topic  
        publisher_ = this->create_publisher<std_msgs::msg::String>("/string_topic", 10);  
        
        // Create a timer that calls the publish_message function every second  
        timer_ = this->create_wall_timer(chrono::seconds(1), bind(&StringPublisher::publish_message, this));  
    }  

private:  
    // Function to publish a message to the topic  
    void publish_message() {  
        // Create a message and set its data  
        auto msg = std_msgs::msg::String();  
        msg.data = "Hello, ROS 2!"; // Message content  
        
        // Log the message being published  
        RCLCPP_INFO(this->get_logger(), "Publishing: %s", msg.data.c_str());  
        
        // Publish the message to the topic  
        publisher_->publish(msg);  
    }  

    // Shared pointer to the publisher for String messages  
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;  
    // Shared pointer to the timer that triggers message publishing  
    rclcpp::TimerBase::SharedPtr timer_;  
};  

// Main function to initialize the ROS 2 node and start publishing  
int main(int argc, char **argv) {  
    rclcpp::init(argc, argv); // Initialize the ROS 2 environment  
    auto node = make_shared<StringPublisher>(); // Create an instance of StringPublisher  
    rclcpp::spin(node); // Start spinning the node to handle callbacks  
    rclcpp::shutdown(); // Shutdown the ROS 2 environment  
    return 0; // Return success  
}  