#include "ros2_action_mux/action_client.hpp"  

using namespace std; // Use the std namespace for convenience  

namespace ros2_action_mux  
{  

ActionClient::ActionClient(const rclcpp::NodeOptions & options)  
: Node("action_client", options)  
{  
  // Declare and retrieve the trigger topic parameter  
  this->declare_parameter("trigger_topic", "/trigger_topic");  
  trigger_topic_ = this->get_parameter("trigger_topic").as_string();  

  RCLCPP_INFO(this->get_logger(), "Starting Action Client, subscribing to: %s",   
              trigger_topic_.c_str());  

  // Initialize the action client for the specified action  
  action_client_ = rclcpp_action::create_client<ProcessRequest>(this, "process_request");  

  // Subscribe to the trigger topic  
  trigger_subscription_ = this->create_subscription<std_msgs::msg::String>(  
    trigger_topic_, 10,  
    bind(&ActionClient::trigger_callback, this, placeholders::_1));  

  // Wait for the action server to be available  
  RCLCPP_INFO(this->get_logger(), "Waiting for action server...");  
  if (!action_client_->wait_for_action_server(chrono::seconds(10))) {  
    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");  
    return;  
  }  
  RCLCPP_INFO(this->get_logger(), "Action server found!");  
}  

ActionClient::~ActionClient()  
{  
  // Cancel any active goals on destruction  
  lock_guard<mutex> lock(client_mutex_);  
  if (current_goal_handle_) {  
    RCLCPP_INFO(this->get_logger(), "Canceling goal on shutdown");  
    action_client_->async_cancel_goal(current_goal_handle_);  
  }  
}  

void ActionClient::trigger_callback(const std_msgs::msg::String::SharedPtr msg)  
{  
  RCLCPP_INFO(this->get_logger(), "Received trigger message: %s", msg->data.c_str());  
  send_goal(msg->data);  // Send a new goal based on the received trigger message  
}  

void ActionClient::send_goal(const std::string & data)  
{  
  lock_guard<mutex> lock(client_mutex_);  

  // Cancel any current goal first  
  if (current_goal_handle_) {  
    RCLCPP_INFO(this->get_logger(), "Canceling previous goal");  
    action_client_->async_cancel_goal(current_goal_handle_);  
    // Proceed with the new goal regardless of the cancel result  
  }  

  // Create a new goal and populate its data  
  auto goal_msg = ProcessRequest::Goal();  
  goal_msg.request_data = data;  

  RCLCPP_INFO(this->get_logger(), "Sending new goal with data: %s", data.c_str());  

  auto send_goal_options = rclcpp_action::Client<ProcessRequest>::SendGoalOptions();  
  send_goal_options.goal_response_callback =  
    bind(&ActionClient::goal_response_callback, this, placeholders::_1);  
  send_goal_options.feedback_callback =  
    bind(&ActionClient::feedback_callback, this, placeholders::_1, placeholders::_2);  
  send_goal_options.result_callback =  
    bind(&ActionClient::result_callback, this, placeholders::_1);  

  // Send the goal to the action server  
  action_client_->async_send_goal(goal_msg, send_goal_options);  
}  

void ActionClient::goal_response_callback(  
  const rclcpp_action::ClientGoalHandle<ProcessRequest>::SharedPtr & goal_handle)  
{  
  lock_guard<mutex> lock(client_mutex_);  

  // Check if the goal was accepted by the action server  
  if (!goal_handle) {  
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");  
    return;  
  }  

  RCLCPP_INFO(this->get_logger(), "Goal accepted by server");  
  current_goal_handle_ = goal_handle;  // Store the current goal handle  
}  

void ActionClient::feedback_callback(  
  GoalHandleProcessRequest::SharedPtr goal_handle,  
  const shared_ptr<const ProcessRequest::Feedback> feedback)  
{  
  (void)goal_handle;  // Suppress unused parameter warning  
  RCLCPP_INFO(this->get_logger(), "Received feedback: %.1f%% complete", feedback->progress * 100.0);  
}  

void ActionClient::result_callback(  
  const GoalHandleProcessRequest::WrappedResult & result)  
{  
  lock_guard<mutex> lock(client_mutex_);  

  // Handle the result of the action goal  
  switch (result.code) {  
    case rclcpp_action::ResultCode::SUCCEEDED:  
      RCLCPP_INFO(this->get_logger(), "Goal succeeded with result: %s",   
                  result.result->result_message.c_str());  
      break;  
    case rclcpp_action::ResultCode::ABORTED:  
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");  
      break;  
    case rclcpp_action::ResultCode::CANCELED:  
      RCLCPP_INFO(this->get_logger(), "Goal was canceled");  
      break;  
    default:  
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");  
      break;  
  }  

  // Clear the current goal handle if it's the one that just finished  
  if (current_goal_handle_ && current_goal_handle_->get_goal_id() == result.goal_id) {  
    current_goal_handle_.reset();  
  }  
}  

}  // namespace ros2_action_mux  

int main(int argc, char ** argv)  
{  
  rclcpp::init(argc, argv);  // Initialize the ROS 2 communication  
  auto node = make_shared<ros2_action_mux::ActionClient>();  // Create the action client node  
  rclcpp::spin(node);  // Spin the node to process incoming messages and actions  
  rclcpp::shutdown();  // Shutdown ROS 2 communication cleanly  
  return 0;  
}  