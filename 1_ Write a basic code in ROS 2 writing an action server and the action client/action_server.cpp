#include <rclcpp/rclcpp.hpp>  
#include <rclcpp_action/rclcpp_action.hpp>  
#include <std_msgs/msg/int32.hpp>  
#include "my_action_pkg/action/fibonacci.hpp"  

// Use namespaces to simplify code readability  
using namespace std;  

// Alias for easier access to action types  
using Fibonacci = my_action_pkg::action::Fibonacci;  
using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;  

class FibonacciActionClient : public rclcpp::Node {  
public:  
    // Constructor for the action client node  
    FibonacciActionClient() : Node("fibonacci_action_client") {  
        // Create the action client for the Fibonacci action  
        action_client_ = rclcpp_action::create_client<Fibonacci>(this, "fibonacci");  

        // Create a subscription to listen for goal orders from a topic  
        subscription_ = this->create_subscription<std_msgs::msg::Int32>(  
            "/goal_topic", 10,   
            bind(&FibonacciActionClient::topic_callback, this, placeholders::_1));  
    }  

private:  
    rclcpp_action::Client<Fibonacci>::SharedPtr action_client_; // Action client for sending goals  
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_; // Subscription for incoming goals  
    GoalHandleFibonacci::SharedPtr active_goal_handle_; // Handle for the currently active goal  

    // Callback for processing messages received from the topic  
    void topic_callback(const std_msgs::msg::Int32::SharedPtr msg) {  
        RCLCPP_INFO(this->get_logger(), "Received goal from topic: %d", msg->data);  

        // Cancel any existing goal if one is active  
        if (active_goal_handle_) {  
            RCLCPP_INFO(this->get_logger(), "Cancelling previous goal...");  
            action_client_->async_cancel_goal(active_goal_handle_);  
        }  

        // Send the new goal based on the received message  
        send_goal(msg->data);  
    }  

    // Send a goal to the Fibonacci action server  
    void send_goal(int order) {  
        // Wait for the action server to become available  
        if (!action_client_->wait_for_action_server(chrono::seconds(5))) {  
            RCLCPP_ERROR(this->get_logger(), "Action server not available!");  
            return;  
        }  

        // Create the goal message to send  
        auto goal_msg = Fibonacci::Goal();  
        goal_msg.order = order; // Set the order specified by the incoming message  

        // Set up options for sending the goal  
        auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();  
        
        // Set the feedback and result callbacks  
        send_goal_options.feedback_callback = bind(&FibonacciActionClient::feedback_callback, this, placeholders::_1, placeholders::_2);  
        send_goal_options.result_callback = bind(&FibonacciActionClient::result_callback, this, placeholders::_1);  

        // Log that we are sending a new goal  
        RCLCPP_INFO(this->get_logger(), "Sending new goal: %d", order);  
        
        // Send the goal asynchronously  
        auto goal_future = action_client_->async_send_goal(goal_msg, send_goal_options);  
        
        // Wait for the goal to be accepted  
        goal_future.wait();   
        active_goal_handle_ = goal_future.get(); // Store the active goal handle  
    }  

    // Callback for feedback received from the action server  
    void feedback_callback(GoalHandleFibonacci::SharedPtr, const shared_ptr<const Fibonacci::Feedback> feedback) {  
        RCLCPP_INFO(this->get_logger(), "Feedback received: [%ld]", feedback->sequence.size());  
    }  

    // Callback for processing the final result from the action server  
    void result_callback(const GoalHandleFibonacci::WrappedResult &result) {  
        RCLCPP_INFO(this->get_logger(), "Final result: %s", result.result->status.c_str());  
    }  
};  

// Main function to initialize the ROS 2 node and start spinning  
int main(int argc, char **argv) {  
    rclcpp::init(argc, argv); // Initialize the ROS 2 environment  
    auto node = make_shared<FibonacciActionClient>(); // Create an instance of the Fibonacci action client  
    rclcpp::spin(node); // Start spinning the node to handle callbacks  
    rclcpp::shutdown(); // Shutdown the ROS 2 environment  
    return 0; // Return success  
}  