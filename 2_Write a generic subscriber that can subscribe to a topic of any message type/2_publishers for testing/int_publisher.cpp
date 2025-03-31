#include <rclcpp/rclcpp.hpp>  
#include <std_msgs/msg/int32.hpp>  

// Use namespaces for easier readability  
using namespace std;  

// Class for publishing integer messages to a ROS 2 topic  
class IntPublisher : public rclcpp::Node {  
public:  
    // Constructor for the IntPublisher node  
    IntPublisher() : Node("int_publisher") {  
        // Create a publisher to send Int32 messages on the "/int_topic" topic  
        publisher_ = this->create_publisher<std_msgs::msg::Int32>("/int_topic", 10);  
        
        // Create a timer that calls the publish_message function every 2 seconds  
        timer_ = this->create_wall_timer(chrono::seconds(2), bind(&IntPublisher::publish_message, this));  
    }  

private:  
    // Function to publish an integer message  
    void publish_message() {  
        // Create a message and set its data to a fixed value (42)  
        auto msg = std_msgs::msg::Int32();  
        msg.data = 42; // Example integer value to publish  
        
        // Log the message being published  
        RCLCPP_INFO(this->get_logger(), "Publishing: %d", msg.data);  
        
        // Publish the message to the topic  
        publisher_->publish(msg);  
    }  

    // Shared pointer to the publisher for Int32 messages  
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;  
    // Shared pointer to the timer that triggers message publishing  
    rclcpp::TimerBase::SharedPtr timer_;  
};  

// Main function to initialize the ROS 2 node and start publishing  
int main(int argc, char **argv) {  
    rclcpp::init(argc, argv); // Initialize the ROS 2 environment  
    auto node = make_shared<IntPublisher>(); // Create an instance of IntPublisher  
    rclcpp::spin(node); // Start spinning the node to handle callbacks  
    rclcpp::shutdown(); // Shutdown the ROS 2 environment  
    return 0; // Return success  
}  