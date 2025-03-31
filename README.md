# ROS 2 Action Mux

This repository demonstrates a ROS 2 action server and client implementation with a topic-based trigger mechanism. The action server waits for 5 seconds before completing a goal, and the client can abort previous goals based on incoming messages from a subscribed topic.

## Features

- Action server that waits 5 seconds before completing goals
- Action client that subscribes to a topic and sends goal requests based on incoming messages
- Ability to abort previous goals when new requests arrive
- Generic subscriber that can subscribe to topics of any message type
- Test publishers for different message types

## Package Structure

```
ros2_action_mux/
├── CMakeLists.txt
├── package.xml
├── src/
│   ├── action_server.cpp
│   ├── action_client.cpp
│   ├── generic_subscriber.cpp
│   ├── test_publishers/
│   │   ├── string_publisher.cpp
│   │   ├── int_publisher.cpp
│   │   └── pose_publisher.cpp
│   └── utils/
│       └── message_type_printer.hpp
├── include/
│   └── ros2_action_mux/
│       ├── action_server.hpp
│       ├── action_client.hpp
│       └── generic_subscriber.hpp
├── action/
│   └── ProcessRequest.action
└── launch/
    ├── action_demo.launch.py
    ├── string_test.launch.py
    ├── int_test.launch.py
    └── pose_test.launch.py
```

## Prerequisites

- ROS 2 Humble or newer
- C++17 compatible compiler
- colcon build tools

## Installation

1. Clone this repository into your ROS 2 workspace:

```bash
cd ~/ros2_ws/src/
git clone https://github.com/ADITYATIWARI342005/ros2_action_mux.git
```

2. Install dependencies:

```bash
cd ~/ros2_ws/
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the package:

```bash
colcon build --packages-select ros2_action_mux
```

4. Source the workspace:

```bash
source install/setup.bash
```

## Usage

### Running the complete demo

```bash
ros2 launch ros2_action_mux action_demo.launch.py
```

This launches the action server, action client, and a string publisher in separate nodes.

### Testing with different message types

```bash
# Test with String messages
ros2 launch ros2_action_mux string_test.launch.py

# Test with Int64 messages
ros2 launch ros2_action_mux int_test.launch.py

# Test with Pose messages
ros2 launch ros2_action_mux pose_test.launch.py
```

### Publishing test messages manually

```bash
# Publish string messages
ros2 topic pub /trigger_topic std_msgs/msg/String "data: 'New request'"

# Publish integer messages
ros2 topic pub /int_topic std_msgs/msg/Int64 "data: 42"

# Publish pose messages
ros2 topic pub /pose_topic geometry_msgs/msg/Pose "{position: {x: 1.0, y: 2.0, z: 3.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}"
```

## Architecture

### Action Definition

The `ProcessRequest.action` file defines the action interface:

```
# Goal
string request_data
---
# Result
bool success
string result_message
---
# Feedback
float32 progress
```

### Components

1. **Action Server**: Receives goal requests and processes them with a simulated 5-second delay.

2. **Action Client**: Subscribes to a trigger topic and sends action goals upon receiving messages. Automatically cancels previous goals when new messages arrive.

3. **Generic Subscriber**: Can subscribe to topics of any message type and executes a callback when messages arrive, printing the message type.

4. **Test Publishers**: Publishers for different message types (String, Int64, Pose) used to test the system.

