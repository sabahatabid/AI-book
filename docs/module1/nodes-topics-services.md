---
sidebar_position: 3
title: Nodes, Topics, and Services
---

# Nodes, Topics, and Services

## Understanding the ROS 2 Communication Model

ROS 2 provides multiple communication patterns to suit different robotic needs. Let's explore each in depth.

## Nodes: The Building Blocks

### What is a Node?

A **node** is a process that performs computation. Robots typically have many nodes working together:
- Camera driver node
- Image processing node
- Motion planning node
- Motor control node

### Node Lifecycle

Nodes in ROS 2 can have managed lifecycles with states:
1. **Unconfigured** - Initial state
2. **Inactive** - Configured but not active
3. **Active** - Fully operational
4. **Finalized** - Shutdown

### Creating a Lifecycle Node

```python
from rclpy.lifecycle import Node, State, TransitionCallbackReturn

class LifecycleNode(Node):
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring...')
        # Setup resources
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Activating...')
        # Start operations
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Deactivating...')
        # Pause operations
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Cleaning up...')
        # Release resources
        return TransitionCallbackReturn.SUCCESS
```

## Topics: Streaming Data

### Publish-Subscribe Pattern

Topics use a **many-to-many** communication model:
- Multiple publishers can send to one topic
- Multiple subscribers can receive from one topic
- Asynchronous and decoupled

### When to Use Topics

- Continuous data streams (sensor readings)
- High-frequency updates
- Broadcasting information
- One-way communication

### Message Types

ROS 2 uses strongly-typed messages:

```python
from std_msgs.msg import String, Int32, Float64
from sensor_msgs.msg import Image, LaserScan, Imu
from geometry_msgs.msg import Twist, Pose, PoseStamped
```

### Advanced Publisher Example

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time

class RobotJointPublisher(Node):
    def __init__(self):
        super().__init__('robot_joint_publisher')
        
        self.publisher_ = self.create_publisher(
            JointState,
            'joint_states',
            10
        )
        
        self.timer = self.create_timer(0.01, self.publish_joint_states)  # 100 Hz
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        self.positions = [0.0, 0.0, 0.0, 0.0]
        
    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.positions
        
        # Simulate joint movement
        self.positions = [p + 0.01 for p in self.positions]
        
        self.publisher_.publish(msg)
```

### Advanced Subscriber Example

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )
        
        self.min_distance = 0.5  # meters
        
    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        
        # Filter out invalid readings
        valid_ranges = ranges[np.isfinite(ranges)]
        
        if len(valid_ranges) > 0:
            min_dist = np.min(valid_ranges)
            
            if min_dist < self.min_distance:
                self.get_logger().warn(
                    f'Obstacle detected at {min_dist:.2f}m!'
                )
```

## Services: Request-Response

### Client-Server Pattern

Services provide **synchronous** request-response communication:
- One server, multiple clients
- Blocking call (waits for response)
- Suitable for occasional operations

### When to Use Services

- Configuration changes
- State queries
- Triggering one-time operations
- Requesting computations

### Creating a Service Server

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AdditionServer(Node):
    def __init__(self):
        super().__init__('addition_server')
        
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_callback
        )
        
    def add_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(
            f'Incoming request: {request.a} + {request.b} = {response.sum}'
        )
        return response

def main():
    rclpy.init()
    server = AdditionServer()
    rclpy.spin(server)
    rclpy.shutdown()
```

### Creating a Service Client

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AdditionClient(Node):
    def __init__(self):
        super().__init__('addition_client')
        
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        
    def send_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        return future.result()

def main():
    rclpy.init()
    client = AdditionClient()
    result = client.send_request(5, 7)
    client.get_logger().info(f'Result: {result.sum}')
    rclpy.shutdown()
```

## Actions: Long-Running Tasks

### Goal-Feedback-Result Pattern

Actions are for **asynchronous** operations with feedback:
- Send a goal
- Receive periodic feedback
- Get final result
- Can be cancelled

### When to Use Actions

- Navigation to a goal
- Grasping objects
- Long computations
- Any task that takes time and needs progress updates

### Action Server Example

```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from action_tutorials_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback
        )
        
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]
        
        for i in range(1, goal_handle.request.order):
            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1]
            )
            
            self.get_logger().info(f'Feedback: {feedback_msg.sequence}')
            goal_handle.publish_feedback(feedback_msg)
            
        goal_handle.succeed()
        
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        return result
```

## Communication Patterns Comparison

| Feature | Topics | Services | Actions |
|---------|--------|----------|---------|
| Pattern | Pub-Sub | Request-Response | Goal-Feedback-Result |
| Synchronous | No | Yes | No |
| Feedback | No | No | Yes |
| Cancellable | No | No | Yes |
| Use Case | Streaming | Queries | Long tasks |

## Best Practices

### Topic Design

1. Use descriptive names: `/robot/camera/image_raw`
2. Namespace by robot: `/robot1/cmd_vel`, `/robot2/cmd_vel`
3. Choose appropriate message types
4. Set proper QoS policies

### Service Design

1. Keep services fast (< 1 second)
2. Use timeouts in clients
3. Handle service failures gracefully
4. Don't use services in control loops

### Action Design

1. Provide meaningful feedback
2. Allow cancellation
3. Set reasonable timeouts
4. Handle preemption properly

## Practical Exercise

Build a simple robot controller:

1. **Topic**: Subscribe to `/cmd_vel` for velocity commands
2. **Service**: Provide `/get_pose` to query robot position
3. **Action**: Implement `/navigate_to_goal` for autonomous navigation

## Next Steps

Learn how to integrate Python AI agents with ROS 2 in [Python and rclpy](python-rclpy.md).
