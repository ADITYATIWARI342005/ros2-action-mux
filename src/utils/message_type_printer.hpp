#ifndef ROS2_ACTION_MUX__UTILS__MESSAGE_TYPE_PRINTER_HPP_  
#define ROS2_ACTION_MUX__UTILS__MESSAGE_TYPE_PRINTER_HPP_  

#include <string>  
#include <typeinfo>  
#include <cxxabi.h>  
#include <memory>  

namespace ros2_action_mux  
{  
namespace utils  
{  

/**  
 *   
 * @param name The mangled type name  
 * @return The demangled type name as a std::string  
 */  
inline std::string demangle_type_name(const char* name)  
{  
  int status = -1; // Variable to store the status of the demangling process  
  std::unique_ptr<char, void(*)(void*)> res {  
    abi::__cxa_demangle(name, nullptr, nullptr, &status), // Demangle the type name  
    std::free // Custom deleter to free the allocated memory  
  };  
  
  // Return the demangled name if successful, otherwise return the original name  
  return (status == 0) ? res.get() : name;   
}  

/**   
 * @tparam T The type to get the name for  
 * @return The demangled type name as a std::string  
 */  
template<typename T>  
inline std::string get_type_name()  
{  
  return demangle_type_name(typeid(T).name()); // Use typeid to get the mangled name and demangle it  
}  

}  // namespace utils  
}  // namespace ros2_action_mux  

#endif  // ROS2_ACTION_MUX__UTILS__MESSAGE_TYPE_PRINTER_HPP_  