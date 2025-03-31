#include <rclcpp/rclcpp.hpp>  
#include <rclcpp/any_subscription_callback.hpp>  
#include <std_msgs/msg/string.hpp>  
#include <std_msgs/msg/int32.hpp>  
#include <geometry_msgs/msg/pose.hpp>  
#include <std_srvs/srv/trigger.hpp>  
#include <map>  
#include <string>  
#include <memory>  
#include <iostream>  

class DynamicGenericSubscriber : public rclcpp::Node {  
public:  
    // Constructor for the DynamicGenericSubscriber node  
    DynamicGenericSubscriber() : Node("dynamic_generic_subscriber") {  
        // Create a service to dynamically add subscriptions  
        add_topic_service_ = this->create_service<std_srvs::srv::Trigger>(  
            "/add_topic", std::bind(&DynamicGenericSubscriber::add_topic_callback, this, std::placeholders::_1, std::placeholders::_2)  
        );  

        RCLCPP_INFO(this->get_logger(), "Dynamic Generic Subscriber Ready. Use /add_topic service to subscribe to new topics.");  
    }  

private:  
    std::map<std::string, rclcpp::SubscriptionBase::SharedPtr> subscriptions_; // Store subscriptions  
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr add_topic_service_; // Service for adding topics  

    // Callback for adding topics dynamically  
    void add_topic_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,  
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response) {  
        (void)request;  // Unused parameter  
        
        std::string topic_name;  
        RCLCPP_INFO(this->get_logger(), "Enter topic name:");  
        std::cin >> topic_name; // Read topic name from the user  
        
        // Check if the topic is already subscribed  
        if (subscriptions_.find(topic_name) != subscriptions_.end()) {  
            response->success = false;  
            response->message = "Topic already subscribed!";  
            return;  
        }  

        // Try subscribing to various message types  
        if (subscribe<std_msgs::msg::String>(topic_name) ||  
            subscribe<std_msgs::msg::Int32>(topic_name) ||  
            subscribe<geometry_msgs::msg::Pose>(topic_name)) {  
            response->success = true;  
            response->message = "Successfully subscribed to " + topic_name;  
        } else {  
            response->success = false;  
            response->message = "Failed to subscribe. Unknown message type.";  
        }  
    }  

    // Template function for subscribing to a topic with a given message type  
    template <typename MessageType>  
    bool subscribe(const std::string &topic_name) {  
        try {  
            // Create a callback to handle incoming messages  
            auto callback = [this, topic_name](typenameMessageType::SharedPtrmsg) {  
                RCLCPP_INFO(this->get_logger(), "Received on [%s]: (Type: %s)",   
                            topic_name.c_str(), typeid(MessageType).name());  
            };  

            // Create the subscription and store it in the map  
            auto sub = this->create_subscription<MessageType>(topic_name, 10, callback);  
            subscriptions_[topic_name] = sub; // Store the subscription  
            RCLCPP_INFO(this->get_logger(), "Subscribed to [%s] as [%s]", topic_name.c_str(), typeid(MessageType).name());  
            return true;  
        } catch (const std::exception &e) {  
            RCLCPP_ERROR(this->get_logger(), "Error subscribing to [%s]: %s", topic_name.c_str(), e.what());  
            return false;  
        }  
    }  
};  

int main(int argc, char **argv) {  
    rclcpp::init(argc, argv); // Initialize the ROS 2 environment  
    auto node = std::make_shared<DynamicGenericSubscriber>(); // Create an instance of DynamicGenericSubscriber  
    rclcpp::spin(node); // Spin the node to handle callbacks  
    rclcpp::shutdown(); // Shutdown the ROS 2 environment  
    return 0; // Return success  
}  