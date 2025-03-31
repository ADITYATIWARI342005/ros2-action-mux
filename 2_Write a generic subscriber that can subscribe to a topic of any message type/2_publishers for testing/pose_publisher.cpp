#include <rclcpp/rclcpp.hpp>  
#include <geometry_msgs/msg/pose.hpp>  

// Use namespaces to simplify code readability  
using namespace std;  

// Class for publishing pose messages to a ROS 2 topic  
class PosePublisher : public rclcpp::Node {  
public:  
    // Constructor for the PosePublisher node  
    PosePublisher() : Node("pose_publisher") {  
        // Create a publisher for "Pose" messages on the "/pose_topic"  
        publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("/pose_topic", 10);  
        
        // Create a timer that calls the publish_message function every 3 seconds  
        timer_ = this->create_wall_timer(chrono::seconds(3), bind(&PosePublisher::publish_message, this));  
    }  

private:  
    // Function to publish a pose message  
    void publish_message() {  
        // Create a new Pose message  
        auto msg = geometry_msgs::msg::Pose();  
        
        // Set the position coordinates of the pose  
        msg.position.x = 1.0; // X position  
        msg.position.y = 2.0; // Y position  
        msg.position.z = 3.0; // Z position  
        
        // Set the orientation of the pose (w component of the quaternion)  
        msg.orientation.w = 1.0; // No rotation in this example  
        
        // Log the published pose information  
        RCLCPP_INFO(this->get_logger(), "Publishing Pose: (%.1f, %.1f, %.1f)",   
                     msg.position.x, msg.position.y, msg.position.z);  
        
        // Publish the pose message to the topic  
        publisher_->publish(msg);  
    }  

    // Shared pointer to the publisher for Pose messages  
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_;  
    // Shared pointer to the timer that triggers message publishing  
    rclcpp::TimerBase::SharedPtr timer_;  
};  

// Main function to initialize the ROS 2 node and start publishing  
int main(int argc, char **argv) {  
    rclcpp::init(argc, argv); // Initialize the ROS 2 environment  
    auto node = make_shared<PosePublisher>(); // Create an instance of PosePublisher  
    rclcpp::spin(node); // Start spinning the node to handle callbacks  
    rclcpp::shutdown(); // Shutdown the ROS 2 environment  
    return 0; // Return success  
}  