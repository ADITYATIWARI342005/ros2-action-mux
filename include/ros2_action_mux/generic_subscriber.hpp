#ifndef ROS2_ACTION_MUX__GENERIC_SUBSCRIBER_HPP_  
#define ROS2_ACTION_MUX__GENERIC_SUBSCRIBER_HPP_  

#include <memory>  
#include <string>  
#include <vector>  
#include <type_traits>  
#include <typeinfo>  
#include <cxxabi.h>  

#include "rclcpp/rclcpp.hpp"  

namespace ros2_action_mux  
{  

// Base class for a generic subscriber  
class GenericSubscriberBase  
{  
public:  
  virtual ~GenericSubscriberBase() = default; // Virtual destructor for proper cleanup  
  virtual void print_message_type() const = 0; // Pure virtual function to print message type  
};  

// Template class for typed subscribers to handle specific message types  
template<typename MessageT>  
class TypedSubscriber : public GenericSubscriberBase  
{  
public:  
  // Constructor that creates a subscription for the given message type and topic  
  TypedSubscriber(  
    rclcpp::Node * node,  
    const std::string & topic_name,  
    size_t queue_size)  
  : topic_name_(topic_name)  // Initialize the topic name  
  {  
    // Create a subscription and provide a callback for incoming messages  
    subscription_ = node->create_subscription<MessageT>(  
      topic_name,  
      queue_size,  
      [this](consttypenameMessageT::SharedPtrmsg) {  
        this->callback(msg); // Call the member callback function  
      });  

    // Log the creation of the subscription  
    RCLCPP_INFO(  
      node->get_logger(),  
      "Created subscription to topic '%s' with message type '%s'",  
      topic_name.c_str(),  
      demangle_type_name(typeid(MessageT).name()).c_str());  
  }  

  // Print the type of message that this subscriber handles  
  void print_message_type() const override  
  {  
    RCLCPP_INFO(  
      rclcpp::get_logger("generic_subscriber"),  
      "Topic '%s' has message type: %s",  
      topic_name_.c_str(),  
      demangle_type_name(typeid(MessageT).name()).c_str());  
  }  

private:  
  std::string topic_name_; // The name of the topic  
  typename rclcpp::Subscription<MessageT>::SharedPtr subscription_; // Shared pointer to the subscription  

  // Callback function for processing received messages  
  void callback(const typename MessageT::SharedPtr msg)  
  {  
    RCLCPP_INFO(  
      rclcpp::get_logger("generic_subscriber"),  
      "Received message on topic '%s' of type '%s'",  
      topic_name_.c_str(),  
      demangle_type_name(typeid(MessageT).name()).c_str());  
  }  

  // Demangle C++ type names for readability  
  std::string demangle_type_name(const char* name) const  
  {  
    int status = -1;  
    // Create a unique pointer with custom deleter to manage memory  
    std::unique_ptr<char, void(*)(void*)> res {  
      abi::__cxa_demangle(name, nullptr, nullptr, &status),  
      std::free  
    };  
    // Return the demangled name or the original if demangling fails  
    return (status == 0) ? res.get() : name;  
  }  
};  

// Generic subscriber node to manage multiple typed subscribers  
class GenericSubscriber : public rclcpp::Node  
{  
public:  
  explicit GenericSubscriber(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());  

private:  
  std::vector<std::unique_ptr<GenericSubscriberBase>> subscribers_; // Store various subscribers  
  void create_type_specific_subscriber(const std::string & topic_name, const std::string & type_name); // Create typed subscriber  
};  

}  // namespace ros2_action_mux  

#endif  // ROS2_ACTION_MUX__GENERIC_SUBSCRIBER_HPP_  