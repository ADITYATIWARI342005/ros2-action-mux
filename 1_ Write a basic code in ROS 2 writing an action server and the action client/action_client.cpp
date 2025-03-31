#include <rclcpp/rclcpp.hpp>  
#include <rclcpp_action/rclcpp_action.hpp>  
#include "my_action_pkg/action/fibonacci.hpp"  

// Use namespaces to simplify code readability  
using namespace std;  

// Alias for easier access to action types  
using Fibonacci = my_action_pkg::action::Fibonacci;  
using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;  

class FibonacciActionServer : public rclcpp::Node {  
public:  
    // Constructor for the action server node  
    FibonacciActionServer() : Node("fibonacci_action_server") {  
        // Create the action server to handle Fibonacci action requests  
        this->action_server_ = rclcpp_action::create_server<Fibonacci>(  
            this,  
            "fibonacci",  // Name of the action  
            bind(&FibonacciActionServer::handle_goal, this, placeholders::_1, placeholders::_2),  
            bind(&FibonacciActionServer::handle_cancel, this, placeholders::_1),  
            bind(&FibonacciActionServer::handle_accepted, this, placeholders::_1));  
    }  

private:  
    rclcpp_action::Server<Fibonacci>::SharedPtr action_server_; // Shared pointer to the action server  

    // Handle incoming goal requests  
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid,   
                                            shared_ptr<const Fibonacci::Goal> goal) {  
        // Log the received goal order  
        RCLCPP_INFO(this->get_logger(), "Received goal request: order = %d", goal->order);  
        // Accept the goal and execute it  
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;  
    }  

    // Handle cancellation of a goal  
    rclcpp_action::CancelResponse handle_cancel(const shared_ptr<GoalHandleFibonacci> goal_handle) {  
        // Log that a cancellation request was received  
        RCLCPP_INFO(this->get_logger(), "Goal cancellation requested.");  
        // Accept the cancellation request  
        return rclcpp_action::CancelResponse::ACCEPT;  
    }  

    // Handle the accepted goal execution  
    void handle_accepted(const shared_ptr<GoalHandleFibonacci> goal_handle) {  
        // Start the goal execution in a separate thread to avoid blocking  
        thread([this, goal_handle]() { this->execute(goal_handle); }).detach();  
    }  

    // Execute the Fibonacci sequence generation  
    void execute(const shared_ptr<GoalHandleFibonacci> goal_handle) {  
        // Log the start of goal execution  
        RCLCPP_INFO(this->get_logger(), "Executing goal...");  

        // Prepare feedback and result objects  
        auto feedback = make_shared<Fibonacci::Feedback>();  
        auto result = make_shared<Fibonacci::Result>();  
        vector<int32_t> sequence = {0, 1}; // Initialize sequence with the first two Fibonacci numbers  

        // Generate Fibonacci sequence up to the specified order  
        for (int i = 2; i < goal_handle->get_goal()->order; ++i) {  
            // Calculate the next Fibonacci number  
            sequence.push_back(sequence[i - 1] + sequence[i - 2]);  

            // Check if the client has canceled the goal  
            if (goal_handle->is_canceling()) {  
                // Log the abortion of the goal  
                RCLCPP_INFO(this->get_logger(), "Goal aborted!");  
                result->status = "Aborted"; // Set result status to Aborted  
                goal_handle->canceled(result); // Notify cancellation to the client  
                return; // Exit the execution  
            }  

            // Send the current sequence as feedback to the client  
            feedback->sequence = sequence;  
            goal_handle->publish_feedback(feedback);  

            // Simulate processing time (1 second delay)  
            this_thread::sleep_for(chrono::seconds(1));  
        }  

        // Set the final result and mark the goal as succeeded  
        result->sequence = sequence; // Store the complete sequence in the result  
        result->status = "Succeeded"; // Set result status to Succeeded  
        goal_handle->succeed(result); // Notify success to the client  
        // Log that the goal has been completed successfully  
        RCLCPP_INFO(this->get_logger(), "Goal succeeded.");  
    }  
};  

// Main function to initialize the ROS 2 node  
int main(int argc, char **argv) {  
    // Initialize the ROS 2 environment  
    rclcpp::init(argc, argv);  
    // Create an instance of the Fibonacci action server  
    auto node = make_shared<FibonacciActionServer>();  
    // Start spinning the node's executor to handle callbacks  
    rclcpp::spin(node);  
    // Shutdown the ROS 2 environment  
    rclcpp::shutdown();  
    return 0; // Return success  
}  