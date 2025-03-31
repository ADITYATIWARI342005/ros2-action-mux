# ROS 2 Action Mux: Handling Dynamic Goals via Topic Subscription

## **Overview**
This project demonstrates a **ROS 2 Action Server and Client** where the action client receives goals via a topic (`/goal_topic`). If a new goal arrives before the previous one completes, the client **cancels the ongoing goal** and sends the new one. The action server executes each goal for **5 seconds** before marking it as successful, while also handling **goal preemption (abortion)** correctly.

## **Thought Process & Approach**

### **1. Understanding the Requirements**
- A **ROS 2 action server** that executes goals for **5 seconds** before succeeding.
- A **ROS 2 action client** that listens to a topic (`/goal_topic`) for goal updates.
- If a new goal arrives **before completion**, the client **cancels the previous goal** and sends the new one.
- The action server must **handle goal abortion gracefully**.

### **2. Designing the Components**
The project consists of **three main components**:

| Component        | Role |
|----------------|------------------------|
| **Action Server** | Accepts goals, executes for 5 seconds, supports cancellations. |
| **Action Client** | Listens to `/goal_topic`, cancels previous goal if a new one arrives, and sends new goals dynamically. |
| **Goal Publisher** | Publishes integer values to `/goal_topic`, simulating goal requests. |

### **3. Steps to Implement the Solution**

#### **Step 1: Define the Action Interface**
We define a **custom action** (`Fibonacci.action`) to represent the Fibonacci sequence computation.

```plaintext
int32 order
---
int32[] sequence
---
string status
```

- The **goal request** contains an integer (`order`) indicating how many Fibonacci numbers to generate.
- The **feedback** is a growing sequence of numbers.
- The **result** contains the final sequence and a status message ("Succeeded" or "Aborted").

#### **Step 2: Implement the Action Server**
- **Accepts goals** and starts execution immediately.
- **Executes for 5 seconds**, computing Fibonacci numbers.
- **Sends feedback** after each computation step.
- **Handles preemption (cancellation)** by checking `goal_handle->is_canceling()` and terminating execution if requested.

#### **Step 3: Implement the Action Client**
- Subscribes to `/goal_topic`.
- Cancels the previous goal if a new goal message arrives.
- Sends a new goal dynamically to the action server.
- Waits for the server's response and handles feedback & result asynchronously.

#### **Step 4: Implement the Goal Publisher**
- Periodically **publishes integer values** to `/goal_topic`.
- Simulates dynamic goal changes.
- Helps test the system’s ability to cancel and replace goals efficiently.

---

## **Installation and Setup**

### **1. Create a ROS 2 Package**
```sh
ros2 pkg create my_action_pkg --build-type ament_cmake
cd my_action_pkg
mkdir action src include
```

### **2. Define the Action**
Create the action file:
```sh
touch action/Fibonacci.action
```
Paste the action definition inside.

Modify `CMakeLists.txt`:
```cmake
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Fibonacci.action"
  DEPENDENCIES builtin_interfaces
)
```
Modify `package.xml`:
```xml
<depend>rosidl_default_generators</depend>
```
Build the package:
```sh
colcon build --packages-select my_action_pkg
source install/setup.bash
```

---

## **Running the System**
### **1. Start the Action Server**
```sh
ros2 run my_action_pkg action_server
```
### **2. Start the Action Client**
```sh
ros2 run my_action_pkg action_client
```
### **3. Publish Goals**
```sh
ros2 run my_action_pkg goal_publisher
```
OR manually publish goals:
```sh
ros2 topic pub /goal_topic std_msgs/msg/Int32 "{data: 10}"
ros2 topic pub /goal_topic std_msgs/msg/Int32 "{data: 12}"
```

---

## **Expected Behavior**
- When a message is published on `/goal_topic`, the client cancels any ongoing goal and sends a new one.
- The action server executes goals for **5 seconds**, unless preempted.
- Logs confirm **goal acceptance, feedback updates, and final results**.

### **Example Output**
```
[INFO] Received goal from topic: 10
[INFO] Cancelling previous goal...
[INFO] Sending new goal: 10
[INFO] Goal accepted.
[INFO] Feedback received: [0, 1, 1, 2, 3]
[INFO] Received goal from topic: 12
[INFO] Cancelling previous goal...
[INFO] Sending new goal: 12
[INFO] Goal aborted!
```

---

## **Key Takeaways**
✅ **Topic-based goal triggering**
✅ **Automatic goal preemption**
✅ **Proper goal abortion handling**
✅ **Periodic feedback & success tracking** 🚀

---
### **Future Improvements**
- Extend the logic to handle **multiple action types**.
- Implement **dynamic goal priorities** based on external conditions.
- Introduce **logging and visualization tools** for better debugging.

🚀 **This project provides a solid foundation for action-based control systems in ROS 2!**

