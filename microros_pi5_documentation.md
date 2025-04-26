# MicroROS-Pi5 ROS2 Robot Car Documentation

## 1. Introduction to ROS 2

### 1.1 What is ROS 2?

ROS 2 (Robot Operating System 2) is not an operating system in the traditional sense, but rather a framework for robot software development. It provides:

- **Communication Infrastructure**: Tools for different parts of a robot to exchange information
- **Hardware Abstraction**: Common interfaces for different robot hardware
- **Common Functionality**: Packages for navigation, vision, control, etc.
- **Development Tools**: Building, testing, and debugging robot applications

### 1.2 Core ROS 2 Concepts

#### 1.2.1 Nodes

Nodes are the basic computational units in ROS 2. Each node typically has a single, modular purpose (e.g., controlling a motor, processing sensor data, etc.). Nodes can be written in different programming languages (C++, Python, etc.).

#### 1.2.2 Topics

Topics are named buses over which nodes exchange messages. Nodes can **publish** messages to or **subscribe** to topics. This communication is anonymous and decoupled:

- Publishers don't know who is subscribing
- Multiple nodes can publish to and subscribe to the same topic
- Communication is asynchronous

Example: A camera node might publish images to a topic called `/camera/image`, and a computer vision node might subscribe to that topic to process those images.

#### 1.2.3 Messages

Messages are strictly typed data structures used for communication over topics. ROS 2 defines many standard message types (geometry, sensor data, etc.), and you can create custom ones.

#### 1.2.4 Services

While topics provide one-to-many communication, services provide request/response interactions between nodes.

#### 1.2.5 Actions

Actions are for long-running tasks, combining topics and services to provide goal setting, feedback, and result reporting.

### 1.3 ROS Computation Graph

The ROS computation graph describes how nodes, topics, services, and actions connect. A standard way to visualize this is using the `rqt_graph` tool.

## 2. Introduction to micro-ROS

### 2.1 What is micro-ROS?

micro-ROS is a version of ROS 2 designed for microcontrollers. It enables the same communication patterns and concepts as ROS 2 on resource-constrained hardware like Arduino, ESP32, or STM32 microcontrollers.

### 2.2 micro-ROS vs. ROS 2

Key differences:

- **Smaller Footprint**: Optimized for limited memory and processing power
- **Real-time Capabilities**: Designed for real-time operating systems (RTOS)
- **Client-Agent Architecture**: Often uses an agent to bridge to the full ROS 2 system

### 2.3 micro-ROS Components

- **Client Library**: The lightweight ROS 2 implementation running on the microcontroller
- **Agent**: A program running on the more powerful computing device (Raspberry Pi)
- **DDS (Data Distribution Service)**: The underlying middleware for communication
- **RMW (ROS Middleware)**: The layer adapting DDS to ROS abstractions

## 3. MicroROS-Pi5 Robot Car Architecture

### 3.1 Hardware Components

- **Raspberry Pi 5**: The main computing board
- **Microcontroller(s)**: Likely an Arduino, ESP32, or STM32 controlling motors and reading sensors
- **Motors**: For driving the wheels
- **Sensors**: Could include encoders, IMU, cameras, etc.

### 3.2 Software Components

- **ROS 2 Humble**: Running in the Docker container (`yahboomtechnology/ros-humble:3.6`)
- **micro-ROS Client**: Running on the microcontroller(s)
- **micro-ROS Agent**: Running on the Raspberry Pi, bridging microcontrollers to the ROS 2 system
- **Startup Scripts**: Automating the system setup

### 3.3 Communication Architecture

```
┌────────────────────────────────────────────┐
│              Raspberry Pi 5                │
│                                            │
│  ┌────────────────────────────────┐        │
│  │      Docker Container          │        │
│  │  (yahboomtechnology/ros-humble)│        │
│  │                                │        │
│  │     ROS 2 Humble System        │        │
│  │  (Nodes, Topics, Services)     │        │
│  └───────────────┬────────────────┘        │
│                  │                          │
│                  │ DDS Communication        │
│                  │                          │
│  ┌───────────────▼────────────────┐        │
│  │        micro-ROS Agent         │        │
│  └───────────────┬────────────────┘        │
│                  │                          │
└──────────────────┼──────────────────────────┘
                   │ UART/USB/Ethernet
┌──────────────────┼──────────────────────────┐
│                  │                          │
│  ┌───────────────▼────────────────┐        │
│  │      micro-ROS Client          │        │
│  │                                │        │
│  │  Microcontroller (Arduino/ESP) │        │
│  │                                │        │
│  └────────────────────────────────┘        │
│                                            │
│                Robot Hardware              │
└────────────────────────────────────────────┘
```

## 4. Communication Between micro-ROS and ROS 2

### 4.1 The Communication Flow

1. **Microcontroller Level**: The micro-ROS client runs on the microcontroller, interfacing with hardware (reading sensors, controlling motors)

2. **Transport Layer**: Communication between the microcontroller and Raspberry Pi occurs over a physical connection:
   - Serial (UART)
   - USB
   - Ethernet or WiFi (if the microcontroller supports it)

3. **Agent Level**: The micro-ROS agent translates between the microcontroller's messages and the ROS 2 DDS middleware

4. **Docker Level**: The containerized ROS 2 system communicates with the micro-ROS agent via DDS

### 4.2 Topic Propagation in Detail

#### Publishing from Microcontroller to ROS 2

1. A microcontroller node creates a publisher for a topic (e.g., `/wheel_encoders`)
2. The node publishes messages using the micro-ROS client library
3. Messages are serialized and sent to the micro-ROS agent
4. The agent deserializes the messages and publishes them to the ROS 2 DDS layer
5. ROS 2 nodes in the Docker container can subscribe to this topic

#### Publishing from ROS 2 to Microcontroller

1. A ROS 2 node in the Docker container publishes to a topic (e.g., `/cmd_vel`)
2. The micro-ROS agent subscribes to this topic
3. When messages arrive, the agent serializes them and sends them to the appropriate microcontroller
4. The micro-ROS client on the microcontroller deserializes the message
5. The microcontroller node's subscriber callback processes the message

### 4.3 Service and Action Communication

Similar to topics, but following the request/response or action protocol.

## 5. System Startup and Configuration

### 5.1 Docker Container Startup

The Docker image (`yahboomtechnology/ros-humble:3.6`) contains:
- ROS 2 Humble Desktop
- Camera-related packages
- GUI support

The container likely starts with:

```bash
docker run -it --network=host --privileged \
  -v /dev:/dev \
  -e DISPLAY=$DISPLAY \
  yahboomtechnology/ros-humble:3.6
```

Key aspects:
- **Network Host Mode**: Allows easy communication with the micro-ROS agent
- **Privileged Mode**: For access to hardware devices
- **Device Mapping**: Accessing physical devices like cameras and serial ports
- **Display**: For GUI applications if needed

### 5.2 micro-ROS Agent Configuration

The micro-ROS agent must be configured for the specific transport being used:

**Serial/UART Example**:
```bash
# Inside or outside the container
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
```

**UDP Example** (if using network communication):
```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

### 5.3 Startup Scripts

The Raspberry Pi likely has startup scripts that:
1. Launch the Docker container
2. Start the micro-ROS agent
3. Launch specific ROS 2 nodes for the robot's functionality

## 6. Common Topics and Messages

### 6.1 Standard Robot Topics

Common topics likely used in the robot car:

- **/cmd_vel** (geometry_msgs/Twist): Commands for velocity (linear and angular)
- **/odom** (nav_msgs/Odometry): Odometry information from wheel encoders
- **/imu** (sensor_msgs/Imu): Data from inertial measurement unit
- **/tf** and **/tf_static**: Coordinate transformations between robot parts
- **/camera/image_raw** (sensor_msgs/Image): Raw camera images
- **/scan** (sensor_msgs/LaserScan): If the robot has distance sensors

### 6.2 Example: Motor Control Flow

1. A joystick or autonomous navigation node publishes to `/cmd_vel`
2. The message contains desired linear and angular velocities
3. The micro-ROS agent receives this message and forwards it to the microcontroller
4. The microcontroller converts the velocities to motor commands
5. The motors move the robot
6. Wheel encoders measure actual movement
7. The microcontroller reads encoders and publishes to `/odom`
8. This data flows back to ROS 2 for localization and navigation

## 7. Development and Debugging

### 7.1 Viewing Topics

To see available topics:

```bash
# Inside the Docker container
ros2 topic list
```

To see messages on a topic:

```bash
ros2 topic echo /cmd_vel
```

### 7.2 Visualizing the System

```bash
# For a graph of nodes and topics
ros2 run rqt_graph rqt_graph
```

### 7.3 Debugging micro-ROS Communication

Check if the agent is running:

```bash
ps aux | grep micro_ros_agent
```

Monitor serial communication:

```bash
# Before starting the agent
cat /dev/ttyACM0
```

### 7.4 Common Issues

- **Connection Problems**: Verify physical connections and permissions for serial devices
- **Topic Mismatch**: Ensure publishers and subscribers use the same topic name and message type
- **QoS Mismatch**: Different Quality of Service settings can prevent communication
- **Resource Constraints**: Microcontrollers have limited memory and processing power

## 8. Extending the System

### 8.1 Adding New Nodes

New ROS 2 nodes can be added to the Docker container:

```bash
# Create a new package
ros2 pkg create my_robot_package --build-type ament_python --dependencies rclpy geometry_msgs

# Build packages
colcon build --packages-select my_robot_package

# Source the workspace
source install/setup.bash

# Run a node
ros2 run my_robot_package my_node
```

### 8.2 Adding micro-ROS Functionality

New functionality can be added to the microcontroller:

1. Use the micro-ROS build system to compile code for the specific microcontroller
2. Flash the updated firmware
3. Restart the micro-ROS agent

## 9. References and Further Reading

- [ROS 2 Documentation](https://docs.ros.org/en/humble/index.html)
- [micro-ROS Documentation](https://micro.ros.org/docs/overview/)
- [Yahboom Official Documentation](https://www.yahboom.net/)
- [DDS Middleware](https://www.omg.org/omg-dds-portal/)