#include <rclcpp/rclcpp.hpp>  
#include <std_msgs/msg/int32.hpp>  

// Use namespaces to simplify code readability  
using namespace std;  

// Class for publishing goal messages to a topic  
class GoalPublisher : public rclcpp::Node {  
public:  
    // Constructor for the GoalPublisher node  
    GoalPublisher() : Node("goal_publisher"), order_(5) {  
        // Create a publisher to send Int32 messages on the "/goal_topic" topic  
        publisher_ = this->create_publisher<std_msgs::msg::Int32>("/goal_topic", 10);  
        
        // Create a timer that triggers the publish_goal function every 2 seconds  
        timer_ = this->create_wall_timer(std::chrono::seconds(2), bind(&GoalPublisher::publish_goal, this));  
    }  

private:  
    // Function to publish the goal  
    void publish_goal() {  
        // Create a message and set its data to the current order value  
        auto msg = std_msgs::msg::Int32();  
        msg.data = order_++;  
        
        // Log the message being published  
        RCLCPP_INFO(this->get_logger(), "Publishing goal: %d", msg.data);  
        
        // Publish the message to the topic  
        publisher_->publish(msg);  
    }  

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_; // Shared pointer to the publisher  
    rclcpp::TimerBase::SharedPtr timer_; // Shared pointer to the timer  
    int order_; // Counter for the goal value, starts at 5  
};  

// Main function to initialize the ROS 2 node and start publishing  
int main(int argc, char **argv) {  
    rclcpp::init(argc, argv); // Initialize the ROS 2 environment  
    auto node = make_shared<GoalPublisher>(); // Create an instance of the GoalPublisher  
    rclcpp::spin(node); // Start spinning the node to handle callbacks  
    rclcpp::shutdown(); // Shutdown the ROS 2 environment  
    return 0; // Return success  
}  