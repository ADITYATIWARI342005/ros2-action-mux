#ifndef ROS2_ACTION_MUX__ACTION_SERVER_HPP_  
#define ROS2_ACTION_MUX__ACTION_SERVER_HPP_  

#include <memory>  
#include <thread>  
#include <chrono>  
#include <functional>  
#include <future>  

#include "rclcpp/rclcpp.hpp"  
#include "rclcpp_action/rclcpp_action.hpp"  
#include "ros2_action_mux/action/process_request.hpp"  

namespace ros2_action_mux  
{  

class ActionServer : public rclcpp::Node  
{  
public:  
  // Type definitions for easier usage  
  using ProcessRequest = ros2_action_mux::action::ProcessRequest;  
  using GoalHandleProcessRequest = rclcpp_action::ServerGoalHandle<ProcessRequest>;  

  // Constructor for the ActionServer class  
  explicit ActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());  

private:  
  // Action server instance for handling ProcessRequest actions  
  rclcpp_action::Server<ProcessRequest>::SharedPtr action_server_;  

  /**  
   * @brief Handle incoming goal requests  
   *   
   * This function is called when a new goal is received.  
   *  
   * @param uuid Unique identifier for the goal  
   * @param goal The goal message  
   * @return The response to the goal request  
   */  
  rclcpp_action::GoalResponse handle_goal(  
    const rclcpp_action::GoalUUID & uuid,  
    std::shared_ptr<const ProcessRequest::Goal> goal);  

  /**  
   * @brief Handle cancel requests for ongoing goals  
   *   
   * This function is called when a cancel request is received for a goal.  
   *  
   * @param goal_handle Shared pointer to the goal handle  
   * @return The response to the cancel request  
   */  
  rclcpp_action::CancelResponse handle_cancel(  
    const std::shared_ptr<GoalHandleProcessRequest> goal_handle);  

  /**  
   * @brief Handle an accepted goal  
   *   
   * This function is called when a goal is accepted for processing.  
   *  
   * @param goal_handle Shared pointer to the accepted goal handle  
   */  
  void handle_accepted(  
    const std::shared_ptr<GoalHandleProcessRequest> goal_handle);  

  /**  
   * @brief Execute the goal processing  
   *   
   * This function runs the main action logic for the given goal.  
   *  
   * @param goal_handle Shared pointer to the goal handle  
   */  
  void execute_goal(const std::shared_ptr<GoalHandleProcessRequest> goal_handle);  
};  

} // namespace ros2_action_mux  

#endif // ROS2_ACTION_MUX__ACTION_SERVER_HPP_  