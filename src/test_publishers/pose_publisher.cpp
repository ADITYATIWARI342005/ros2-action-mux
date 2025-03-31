#include "rclcpp/rclcpp.hpp"  
#include "geometry_msgs/msg/pose.hpp"  
#include <cmath>  

using namespace std; // Use the std namespace for convenience  

class PosePublisher : public rclcpp::Node  
{  
public:  
  PosePublisher()  
  : Node("pose_publisher")  
  {  
    // Declare parameters for topic, publish rate, and motion type  
    this->declare_parameter("topic", "/pose_topic");  
    this->declare_parameter("publish_rate", 1.0);  
    this->declare_parameter("circular_motion", true);  
    
    // Retrieve parameter values  
    string topic = this->get_parameter("topic").as_string();  
    double publish_rate = this->get_parameter("publish_rate").as_double();  
    circular_motion_ = this->get_parameter("circular_motion").as_bool();  
    
    // Create a publisher for Pose messages on the specified topic  
    publisher_ = this->create_publisher<geometry_msgs::msg::Pose>(topic, 10);  
    
    // Create a timer that triggers the publish_message method at the specified rate  
    timer_ = this->create_wall_timer(  
      chrono::duration<double>(1.0 / publish_rate),  
      bind(&PosePublisher::publish_message, this));  
    
    RCLCPP_INFO(this->get_logger(), "Pose publisher started, publishing to '%s' at %.1f Hz",  
                topic.c_str(), publish_rate);  
  }  

private:  
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_; // Publisher for Pose messages  
  rclcpp::TimerBase::SharedPtr timer_; // Timer for publishing messages  
  bool circular_motion_; // Flag to control the type of motion (circular or random)  
  double angle_ = 0.0; // Current angle for circular motion  
  
  void publish_message()  
  {  
    auto message = geometry_msgs::msg::Pose();  
    
    if (circular_motion_) {  
      // Generate a circular path  
      message.position.x = 2.0 * cos(angle_);  
      message.position.y = 2.0 * sin(angle_);  
      message.position.z = 0.0; // Fixed height  
      
      // Calculate orientation to point along the circular path  
      message.orientation.x = 0.0;  
      message.orientation.y = 0.0;  
      message.orientation.z = sin(angle_ / 2.0);  
      message.orientation.w = cos(angle_ / 2.0);  
      
      // Update angle for the next publish  
      angle_ += 0.1;  
      // Keep angle within 0 to 2π range  
      if (angle_ > 2.0 * M_PI) {  
        angle_ -= 2.0 * M_PI;  
      }  
    } else {  
      // Generate a random pose  
      message.position.x = static_cast<double>(rand()) / RAND_MAX * 10.0 - 5.0; // Range: [-5, 5]  
      message.position.y = static_cast<double>(rand()) / RAND_MAX * 10.0 - 5.0; // Range: [-5, 5]  
      message.position.z = static_cast<double>(rand()) / RAND_MAX * 10.0 - 5.0; // Range: [-5, 5]  
      
      // Generate a random orientation (not necessarily a valid quaternion)  
      message.orientation.x = static_cast<double>(rand()) / RAND_MAX;  
      message.orientation.y = static_cast<double>(rand()) / RAND_MAX;  
      message.orientation.z = static_cast<double>(rand()) / RAND_MAX;  
      message.orientation.w = static_cast<double>(rand()) / RAND_MAX;  
      
      // Normalize the quaternion  
      double norm = sqrt(  
        message.orientation.x * message.orientation.x +  
        message.orientation.y * message.orientation.y +  
        message.orientation.z * message.orientation.z +  
        message.orientation.w * message.orientation.w);  
      
      message.orientation.x /= norm;  
      message.orientation.y /= norm;  
      message.orientation.z /= norm;  
      message.orientation.w /= norm;  
    }  
    
    RCLCPP_INFO(this->get_logger(), "Publishing pose: position (%.2f, %.2f, %.2f)",  
                message.position.x, message.position.y, message.position.z);  
    publisher_->publish(message); // Publish the generated Pose message  
  }  
};  

int main(int argc, char * argv[])  
{  
  rclcpp::init(argc, argv); // Initialize ROS 2 communication  
  auto node = make_shared<PosePublisher>(); // Create an instance of PosePublisher  
  rclcpp::spin(node); // Spin the node to keep it active  
  r