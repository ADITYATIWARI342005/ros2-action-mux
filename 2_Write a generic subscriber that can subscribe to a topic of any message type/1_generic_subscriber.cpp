#include <rclcpp/rclcpp.hpp>  
#include <rclcpp/any_subscription_callback.hpp>  
#include <std_msgs/msg/string.hpp>  
#include <std_msgs/msg/int32.hpp>  
#include <geometry_msgs/msg/pose.hpp>  
#include <memory>  
#include <vector>  

// Use namespaces to simplify code readability  
using namespace std;  

// Class for subscribing to multiple topics of different message types  
class GenericSubscriber : public rclcpp::Node {  
public:  
    // Constructor for the GenericSubscriber node  
    GenericSubscriber() : Node("generic_subscriber") {  
        // Subscribe to different topics with various message types  
        subscribe<std_msgs::msg::String>("/string_topic");  
        subscribe<std_msgs::msg::Int32>("/int_topic");  
        subscribe<geometry_msgs::msg::Pose>("/pose_topic");  
    }  

private:  
    // Template function to handle subscriptions for different message types  
    template <typename MessageType>  
    void subscribe(const string &topic_name) {  
        // Lambda function to handle incoming messages  
        auto callback = [this, topic_name](consttypenameMessageType::SharedPtrmsg) {  
            // Log the received message type and topic name  
            RCLCPP_INFO(this->get_logger(), "Received on [%s]: %s",   
                        topic_name.c_str(), typeid(MessageType).name());  
        };  

        // Create the subscription and store it in the vector  
        auto sub = this->create_subscription<MessageType>(topic_name, 10, callback);  
        subscriptions_.push_back(sub); // Store the subscription in the vector  
    }  

    // Vector to hold all active subscriptions  
    vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;  
};  

// Main function to initialize the ROS 2 node and start subscribing  
int main(int argc, char **argv) {  
    rclcpp::init(argc, argv); // Initialize the ROS 2 environment  
    auto node = make_shared<GenericSubscriber>(); // Create an instance of GenericSubscriber  
    rclcpp::spin(node); // Start spinning the node to handle callbacks  
    rclcpp::shutdown(); // Shutdown the ROS 2 environment  
    return 0; // Return success  
}  