#ifndef ROS2_ACTION_MUX__ACTION_CLIENT_HPP_  
#define ROS2_ACTION_MUX__ACTION_CLIENT_HPP_  

#include <memory>  
#include <string>  
#include <mutex>  

#include "rclcpp/rclcpp.hpp"  
#include "rclcpp_action/rclcpp_action.hpp"  
#include "std_msgs/msg/string.hpp"  
#include "ros2_action_mux/action/process_request.hpp"  

namespace ros2_action_mux  
{  

class ActionClient : public rclcpp::Node  
{  
public:  
  using ProcessRequest = ros2_action_mux::action::ProcessRequest; // Alias for ProcessRequest action type  
  using GoalHandleProcessRequest = rclcpp_action::ClientGoalHandle<ProcessRequest>; // Alias for the goal handle type  

  // Constructor for ActionClient  
  explicit ActionClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());  
  
  // Destructor for ActionClient  
  virtual ~ActionClient();  

private:  
  // Action client instance for sending ProcessRequest actions  
  rclcpp_action::Client<ProcessRequest>::SharedPtr action_client_;  
  
  // Subscription to trigger messages  
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr trigger_subscription_;  
  
  // Mutex to protect access to shared state across threads  
  std::mutex client_mutex_;  
  
  // Current goal handle to track ongoing actions  
  std::shared_ptr<GoalHandleProcessRequest> current_goal_handle_;  
  
  // Topic for listening to trigger messages  
  std::string trigger_topic_;  

  // Callback function for processing incoming trigger messages  
  void trigger_callback(const std_msgs::msg::String::SharedPtr msg);  
  
  // Sends a new goal to the action server  
  void send_goal(const std::string & data);  
  
  // Callback function for handling goal response from the action server  
  void goal_response_callback(  
    const rclcpp_action::ClientGoalHandle<ProcessRequest>::SharedPtr & goal_handle);  
  
  // Callback function for processing feedback from the action server  
  void feedback_callback(  
    GoalHandleProcessRequest::SharedPtr goal_handle,  
    const std::shared_ptr<const ProcessRequest::Feedback> feedback);  
  
  // Callback function for handling the result of the completed goal  
  void result_callback(  
    const GoalHandleProcessRequest::WrappedResult & result);  
};  

}  // namespace ros2_action_mux  

#endif  // ROS2_ACTION_MUX__ACTION_CLIENT_HPP_  