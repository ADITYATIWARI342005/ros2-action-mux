#include "ros2_action_mux/generic_subscriber.hpp"  

#include "std_msgs/msg/string.hpp"  
#include "std_msgs/msg/int64.hpp"  
#include "geometry_msgs/msg/pose.hpp"  

using namespace std; // Use the std namespace for convenience  

namespace ros2_action_mux  
{  

GenericSubscriber::GenericSubscriber(const rclcpp::NodeOptions & options)  
: Node("generic_subscriber", options)  
{  
  // Declare a parameter 'topics' as a vector of strings  
  this->declare_parameter("topics", vector<string>());  
  auto topics = this->get_parameter("topics").as_string_array();  

  if (topics.empty()) {  
    // No topics specified, subscribing to some default topics  
    RCLCPP_INFO(this->get_logger(), "No topics specified, subscribing to some default topics");  
    create_type_specific_subscriber("/trigger_topic", "std_msgs/msg/String");  
    create_type_specific_subscriber("/int_topic", "std_msgs/msg/Int64");  
    create_type_specific_subscriber("/pose_topic", "geometry_msgs/msg/Pose");  
  } else {  
    // For each specified topic, determine its type and create an appropriate subscriber  
    for (const auto & topic : topics) {  
      // Simplified approach to determine topic type based on the topic name  
      if (topic.find("string") != string::npos ||   
          topic.find("trigger") != string::npos) {  
        create_type_specific_subscriber(topic, "std_msgs/msg/String");  
      } else if (topic.find("int") != string::npos) {  
        create_type_specific_subscriber(topic, "std_msgs/msg/Int64");  
      } else if (topic.find("pose") != string::npos) {  
        create_type_specific_subscriber(topic, "geometry_msgs/msg/Pose");  
      } else {  
        // Default to string if the type cannot be determined  
        RCLCPP_WARN(this->get_logger(),   
                   "Could not determine type for topic '%s', defaulting to std_msgs/msg/String",  
                   topic.c_str());  
        create_type_specific_subscriber(topic, "std_msgs/msg/String");  
      }  
    }  
  }  
}  

void GenericSubscriber::create_type_specific_subscriber(const string & topic_name, const string & type_name)  
{  
  // Log the creation of a subscriber  
  RCLCPP_INFO(this->get_logger(), "Creating subscriber for topic '%s' with type '%s'",  
             topic_name.c_str(), type_name.c_str());  
  
  // Create and store the subscriber based on the type  
  if (type_name == "std_msgs/msg/String") {  
    subscribers_.push_back(  
      make_unique<TypedSubscriber<std_msgs::msg::String>>(this, topic_name, 10));  
  } else if (type_name == "std_msgs/msg/Int64") {  
    subscribers_.push_back(  
      make_unique<TypedSubscriber<std_msgs::msg::Int64>>(this, topic_name, 10));  
  } else if (type_name == "geometry_msgs/msg/Pose") {  
    subscribers_.push_back(  
      make_unique<TypedSubscriber<geometry_msgs::msg::Pose>>(this, topic_name, 10));  
  } else {  
    // Log an error if the message type is unsupported  
    RCLCPP_ERROR(this->get_logger(), "Unsupported message type: %s", type_name.c_str());  
  }  
}  

}  // namespace ros2_action_mux  

int main(int argc, char ** argv)  
{  
  rclcpp::init(argc, argv); // Initialize the ROS 2 communication  
  auto node = make_shared<ros2_action_mux::GenericSubscriber>(); // Create an instance of GenericSubscriber  
  rclcpp::spin(node); // Spin the node to process messages  
  rclcpp::shutdown(); // Shutdown ROS 2 communication cleanly  
  return 0;  
}  