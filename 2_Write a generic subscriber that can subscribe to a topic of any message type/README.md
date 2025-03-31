# ROS 2 Generic Subscriber

## **Overview**
This project demonstrates a **generic ROS 2 subscriber** that can listen to **any topic and message type**, dynamically detecting the type of message received. It also includes multiple publishers to test the functionality by publishing different message types.

## **Key Features**
✅ A **single subscriber** that listens to **multiple topics** and detects message types dynamically.
✅ Multiple publishers to send **String, Integer, and Pose messages**.
✅ The subscriber prints the **message type and content** when a message is received.

---

## **Project Structure**
```
my_generic_pkg/
├── src/
│   ├── generic_subscriber.cpp   # Generic subscriber node
│   ├── string_publisher.cpp     # Publishes std_msgs/String
│   ├── int_publisher.cpp        # Publishes std_msgs/Int32
│   ├── pose_publisher.cpp       # Publishes geometry_msgs/Pose
├── CMakeLists.txt               # Build configuration
├── package.xml                  # ROS 2 package dependencies
```

---

## **Implementation Details**
### **1. Generic Subscriber (`generic_subscriber.cpp`)**
- Subscribes to **three topics**: `/string_topic`, `/int_topic`, `/pose_topic`.
- Uses a **templated function** to handle different message types.
- Prints both the **topic name** and **message type** when a message is received.

### **2. Publishers**
- **`string_publisher.cpp`** → Publishes **std_msgs/String** messages.
- **`int_publisher.cpp`** → Publishes **std_msgs/Int32** messages.
- **`pose_publisher.cpp`** → Publishes **geometry_msgs/Pose** messages.

---

## **Installation & Setup**
### **1. Create the ROS 2 Package**
```sh
ros2 pkg create my_generic_pkg --build-type ament_cmake
cd my_generic_pkg
mkdir src
```

### **2. Add Dependencies**
Modify `package.xml` to include:
```xml
<depend>std_msgs</depend>
<depend>geometry_msgs</depend>
```

Modify `CMakeLists.txt` to compile all source files:
```cmake
add_executable(generic_subscriber src/generic_subscriber.cpp)
ament_target_dependencies(generic_subscriber rclcpp std_msgs geometry_msgs)

add_executable(string_publisher src/string_publisher.cpp)
ament_target_dependencies(string_publisher rclcpp std_msgs)

add_executable(int_publisher src/int_publisher.cpp)
ament_target_dependencies(int_publisher rclcpp std_msgs)

add_executable(pose_publisher src/pose_publisher.cpp)
ament_target_dependencies(pose_publisher rclcpp geometry_msgs)

install(TARGETS generic_subscriber string_publisher int_publisher pose_publisher
  DESTINATION lib/${PROJECT_NAME})
```

### **3. Build the Package**
```sh
colcon build --packages-select my_generic_pkg
source install/setup.bash
```

---

## **Running the Nodes**
### **1. Start the Generic Subscriber**
```sh
ros2 run my_generic_pkg generic_subscriber
```
### **2. Start the Publishers**
```sh
ros2 run my_generic_pkg string_publisher
ros2 run my_generic_pkg int_publisher
ros2 run my_generic_pkg pose_publisher
```
### **3. Publish Messages Manually (Optional)**
```sh
ros2 topic pub /string_topic std_msgs/msg/String "{data: 'Hello, ROS 2!'}"
ros2 topic pub /int_topic std_msgs/msg/Int32 "{data: 42}"
ros2 topic pub /pose_topic geometry_msgs/msg/Pose "{position: {x: 1.0, y: 2.0, z: 3.0}, orientation: {w: 1.0}}"
```

---

## **Expected Output**
### **Generic Subscriber Terminal Output**
```
[INFO] Received on [/string_topic]: std_msgs::msg::String
[INFO] Received on [/int_topic]: std_msgs::msg::Int32
[INFO] Received on [/pose_topic]: geometry_msgs::msg::Pose
```
### **Publishers Terminal Output**
```
[INFO] Publishing: Hello, ROS 2!
[INFO] Publishing: 42
[INFO] Publishing Pose: (1.0, 2.0, 3.0)
```

---

## **Conclusion & Next Steps**
✅ Successfully implemented a **generic subscriber** handling multiple message types.  
✅ Verified with multiple publishers.  
✅ Easily extendable to support **additional message types**.

**Next Steps:**
- Add support for **more message types dynamically**.
- Implement **QoS settings** for better reliability.
- Extend functionality to allow **dynamic topic subscription at runtime**.

🚀 **This project provides a strong foundation for flexible ROS 2 communication!**

