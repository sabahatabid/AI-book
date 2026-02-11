---
sidebar_position: 4
title: Python and rclpy
---

# Bridging Python AI Agents to ROS 2

## Introduction

The `rclpy` library is the Python client for ROS 2, enabling seamless integration between Python-based AI models and robotic systems. This is where your AI knowledge meets physical robotics.

## Why Python for Robotics?

- Rich AI/ML ecosystem (PyTorch, TensorFlow, scikit-learn)
- Rapid prototyping and development
- Extensive libraries for computer vision, NLP, and more
- Easy integration with ROS 2

## Setting Up Your Environment

```bash
# Install Python dependencies
pip install rclpy numpy opencv-python torch transformers

# Verify installation
python3 -c "import rclpy; print('rclpy installed successfully')"
```

## Integrating AI Models with ROS 2

### Example: Object Detection Node

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D
from cv_bridge import CvBridge
import cv2
import torch
from torchvision import models, transforms

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        
        # Initialize AI model
        self.model = models.detection.fasterrcnn_resnet50_fpn(pretrained=True)
        self.model.eval()
        
        # ROS 2 setup
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        self.publisher = self.create_publisher(
            Detection2DArray,
            '/detections',
            10
        )
        
        self.get_logger().info('Object Detection Node initialized')
    
    def image_callback(self, msg):
        # Convert ROS Image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Preprocess for model
        transform = transforms.Compose([
            transforms.ToTensor(),
        ])
        input_tensor = transform(cv_image).unsqueeze(0)
        
        # Run inference
        with torch.no_grad():
            predictions = self.model(input_tensor)
        
        # Publish detections
        detection_msg = self.create_detection_message(predictions[0])
        self.publisher.publish(detection_msg)
    
    def create_detection_message(self, predictions):
        msg = Detection2DArray()
        # Process predictions and populate message
        return msg

def main():
    rclpy.init()
    node = ObjectDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### Example: LLM-Powered Robot Controller

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from openai import OpenAI

class LLMRobotController(Node):
    def __init__(self):
        super().__init__('llm_robot_controller')
        
        # Initialize OpenAI client
        self.client = OpenAI()
        
        # ROS 2 publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.voice_sub = self.create_subscription(
            String,
            '/voice_command',
            self.voice_callback,
            10
        )
        
        self.system_prompt = """You are a robot controller. Convert natural 
        language commands into robot movements. Respond with JSON containing:
        {
            "linear_x": float,
            "angular_z": float,
            "duration": float
        }"""
        
    def voice_callback(self, msg):
        command = msg.data
        self.get_logger().info(f'Received command: {command}')
        
        # Get LLM interpretation
        response = self.client.chat.completions.create(
            model="gpt-4",
            messages=[
                {"role": "system", "content": self.system_prompt},
                {"role": "user", "content": command}
            ]
        )
        
        # Parse and execute
        import json
        movement = json.loads(response.choices[0].message.content)
        self.execute_movement(movement)
    
    def execute_movement(self, movement):
        twist = Twist()
        twist.linear.x = movement['linear_x']
        twist.angular.z = movement['angular_z']
        
        # Publish for specified duration
        duration = movement['duration']
        rate = self.create_rate(10)  # 10 Hz
        
        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < duration:
            self.cmd_vel_pub.publish(twist)
            rate.sleep()
        
        # Stop
        self.cmd_vel_pub.publish(Twist())

def main():
    rclpy.init()
    node = LLMRobotController()
    rclpy.spin(node)
    rclpy.shutdown()
```

## Advanced Patterns

### Multi-threaded Execution

```python
from rclpy.executors import MultiThreadedExecutor
import threading

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
        self.create_timer(0.1, self.fast_callback)
        self.create_timer(1.0, self.slow_callback)
    
    def fast_callback(self):
        # High-frequency sensor reading
        pass
    
    def slow_callback(self):
        # Low-frequency processing
        pass

def main():
    rclpy.init()
    node = SensorNode()
    
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
```

### Callback Groups for Parallel Processing

```python
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

class ParallelNode(Node):
    def __init__(self):
        super().__init__('parallel_node')
        
        # Different callback groups for parallel execution
        self.group1 = MutuallyExclusiveCallbackGroup()
        self.group2 = MutuallyExclusiveCallbackGroup()
        
        self.create_subscription(
            Image, '/camera1', self.camera1_callback, 10,
            callback_group=self.group1
        )
        
        self.create_subscription(
            Image, '/camera2', self.camera2_callback, 10,
            callback_group=self.group2
        )
    
    def camera1_callback(self, msg):
        # Process camera 1 (can run in parallel with camera2_callback)
        pass
    
    def camera2_callback(self, msg):
        # Process camera 2
        pass
```

## Integrating with Popular AI Frameworks

### PyTorch Integration

```python
import torch
import torch.nn as nn

class RobotVisionNode(Node):
    def __init__(self):
        super().__init__('robot_vision_node')
        
        # Load custom trained model
        self.model = torch.load('robot_model.pth')
        self.model.eval()
        
        # Use GPU if available
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model.to(self.device)
```

### TensorFlow Integration

```python
import tensorflow as tf

class TFRobotNode(Node):
    def __init__(self):
        super().__init__('tf_robot_node')
        
        # Load TensorFlow model
        self.model = tf.keras.models.load_model('robot_model.h5')
```

### Transformers for NLP

```python
from transformers import pipeline

class NLPRobotNode(Node):
    def __init__(self):
        super().__init__('nlp_robot_node')
        
        # Initialize NLP pipeline
        self.nlp = pipeline('text-classification', 
                           model='distilbert-base-uncased')
        
    def process_command(self, text):
        result = self.nlp(text)
        return result[0]['label']
```

## Best Practices

### 1. Separate AI Processing from ROS Communication

```python
class AIProcessor:
    """Pure Python AI logic"""
    def __init__(self):
        self.model = load_model()
    
    def process(self, data):
        return self.model.predict(data)

class ROSNode(Node):
    """ROS 2 communication wrapper"""
    def __init__(self):
        super().__init__('ros_node')
        self.ai_processor = AIProcessor()
        
    def callback(self, msg):
        result = self.ai_processor.process(msg.data)
        self.publish_result(result)
```

### 2. Handle Model Loading Efficiently

```python
class LazyModelNode(Node):
    def __init__(self):
        super().__init__('lazy_model_node')
        self._model = None
    
    @property
    def model(self):
        if self._model is None:
            self.get_logger().info('Loading model...')
            self._model = load_heavy_model()
        return self._model
```

### 3. Use Async for Non-Blocking Operations

```python
import asyncio

class AsyncAINode(Node):
    def __init__(self):
        super().__init__('async_ai_node')
        self.loop = asyncio.get_event_loop()
    
    async def async_inference(self, data):
        # Non-blocking AI inference
        result = await self.model.predict_async(data)
        return result
    
    def callback(self, msg):
        future = asyncio.run_coroutine_threadsafe(
            self.async_inference(msg.data),
            self.loop
        )
        result = future.result()
```

## Debugging Tips

### Logging

```python
self.get_logger().debug('Debug message')
self.get_logger().info('Info message')
self.get_logger().warn('Warning message')
self.get_logger().error('Error message')
self.get_logger().fatal('Fatal message')
```

### Profiling

```python
import cProfile
import pstats

def profile_callback(self, msg):
    profiler = cProfile.Profile()
    profiler.enable()
    
    # Your code here
    self.process_data(msg)
    
    profiler.disable()
    stats = pstats.Stats(profiler)
    stats.sort_stats('cumulative')
    stats.print_stats(10)
```

## Practical Exercise

Create an AI-powered obstacle avoidance system:

1. Subscribe to laser scan data
2. Use a neural network to classify obstacles
3. Publish velocity commands to avoid obstacles
4. Log AI decisions for debugging

## Next Steps

Learn how to describe robot structures in [URDF for Humanoids](urdf-humanoids.md).
