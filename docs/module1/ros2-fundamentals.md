---
sidebar_position: 2
title: ROS 2 Fundamentals
---

# ROS 2 Fundamentals

## What is ROS 2?

ROS 2 (Robot Operating System 2) is an open-source middleware framework for building robot applications. Despite its name, it's not an operating system but rather a collection of tools, libraries, and conventions that simplify the task of creating complex robot behavior.

## Why ROS 2?

ROS 2 improves upon ROS 1 with:
- **Real-time capabilities** for safety-critical applications
- **Better security** with DDS (Data Distribution Service)
- **Multi-platform support** (Linux, Windows, macOS)
- **Improved communication** with Quality of Service (QoS) policies
- **Production-ready** for commercial applications

## Core Architecture

### The Computation Graph

ROS 2 creates a network of processes (nodes) that communicate with each other. This forms a computation graph where:

```
[Sensor Node] --topic--> [Processing Node] --topic--> [Actuator Node]
```

### Key Components

1. **Nodes** - Independent processes that perform computation
2. **Topics** - Named buses for streaming data
3. **Services** - Synchronous request-response communication
4. **Actions** - Asynchronous goal-oriented tasks with feedback
5. **Parameters** - Configuration values for nodes

## Installation

### Ubuntu 22.04 (Recommended)

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Add ROS 2 repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble
sudo apt update
sudo apt install ros-humble-desktop python3-colcon-common-extensions

# Source the setup file
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Your First ROS 2 Node

### Publisher Node (Python)

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Subscriber Node (Python)

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Running Your Nodes

```bash
# Terminal 1 - Run publisher
python3 publisher_node.py

# Terminal 2 - Run subscriber
python3 subscriber_node.py
```

## ROS 2 Command Line Tools

### Essential Commands

```bash
# List all nodes
ros2 node list

# Get node info
ros2 node info /node_name

# List all topics
ros2 topic list

# Echo topic messages
ros2 topic echo /topic_name

# Get topic info
ros2 topic info /topic_name

# Publish to a topic
ros2 topic pub /topic_name std_msgs/msg/String "data: 'Hello'"

# List all services
ros2 service list

# Call a service
ros2 service call /service_name std_srvs/srv/Empty
```

## Workspace Structure

```
ros2_ws/
├── src/
│   └── my_robot_package/
│       ├── my_robot_package/
│       │   ├── __init__.py
│       │   └── my_node.py
│       ├── package.xml
│       ├── setup.py
│       └── setup.cfg
├── build/
├── install/
└── log/
```

## Creating a Package

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Create Python package
ros2 pkg create --build-type ament_python my_robot_package

# Build the workspace
cd ~/ros2_ws
colcon build

# Source the workspace
source install/setup.bash
```

## Best Practices

1. **Use meaningful names** for nodes, topics, and services
2. **Follow naming conventions** (lowercase with underscores)
3. **Set appropriate QoS policies** for your use case
4. **Handle shutdown gracefully** with proper cleanup
5. **Log important information** using the built-in logger
6. **Use parameters** for configuration instead of hardcoding

## Common Patterns

### Timer-based Publishing

```python
self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
```

### Parameter Declaration

```python
self.declare_parameter('my_parameter', 'default_value')
param_value = self.get_parameter('my_parameter').value
```

### Quality of Service (QoS)

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

self.publisher_ = self.create_publisher(String, 'topic', qos_profile)
```

## Exercises

1. Create a node that publishes sensor data at 50 Hz
2. Build a subscriber that processes and filters incoming data
3. Implement a simple service for robot state queries
4. Create a launch file to start multiple nodes together

## Next Steps

Now that you understand ROS 2 basics, let's explore [Nodes, Topics, and Services](nodes-topics-services.md) in greater detail.
