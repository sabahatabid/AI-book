---
sidebar_position: 2
title: Gazebo Simulation
---

# Gazebo Simulation

## Introduction

Gazebo is the most widely used robot simulator in the ROS ecosystem. It provides accurate physics simulation, sensor simulation, and seamless ROS 2 integration.

## Installation

### Gazebo Classic (Recommended for ROS 2 Humble)

```bash
# Install Gazebo 11
sudo apt install ros-humble-gazebo-ros-pkgs

# Verify installation
gazebo --version
```

### Gazebo Ignition (Next Generation)

```bash
# Install Gazebo Fortress
sudo apt install ros-humble-ros-gz
```

## Gazebo Architecture

### Core Components

1. **Server (gzserver)** - Physics simulation engine
2. **Client (gzclient)** - Visualization and GUI
3. **Plugins** - Extend functionality
4. **World files** - Define environments
5. **Model files** - Define objects and robots

## Creating Your First World

### Basic World File

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="default">
    
    <!-- Physics -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    
    <!-- Lighting -->
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    
    <!-- Ground Plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <!-- Simple Box Obstacle -->
    <model name="box">
      <pose>2 0 0.5 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
  </world>
</sdf>
```

### Launch Gazebo with Your World

```bash
gazebo my_world.world
```

## Spawning Robots in Gazebo

### Launch File for Robot Spawning

```python
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Path to URDF
    urdf_file = os.path.join(
        os.getcwd(),
        'urdf',
        'humanoid.urdf'
    )
    
    # Read URDF
    with open(urdf_file, 'r') as file:
        robot_desc = file.read()
    
    return LaunchDescription([
        # Start Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', 'my_world.world'],
            output='screen'
        ),
        
        # Spawn robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'humanoid',
                '-topic', 'robot_description',
                '-x', '0', '-y', '0', '-z', '1.0'
            ],
            output='screen'
        ),
        
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
            output='screen'
        ),
    ])
```

## Physics Configuration

### Choosing a Physics Engine

```xml
<!-- ODE (Default - Fast, stable) -->
<physics type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1</real_time_factor>
</physics>

<!-- Bullet (Better collision detection) -->
<physics type="bullet">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1</real_time_factor>
</physics>

<!-- Simbody (More accurate, slower) -->
<physics type="simbody">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1</real_time_factor>
</physics>
```

### Tuning Physics Parameters

```xml
<physics type="ode">
  <!-- Time step -->
  <max_step_size>0.001</max_step_size>
  
  <!-- Real-time factor (1.0 = real-time) -->
  <real_time_factor>1.0</real_time_factor>
  
  <!-- Update rate -->
  <real_time_update_rate>1000</real_time_update_rate>
  
  <!-- Solver iterations (higher = more accurate) -->
  <ode>
    <solver>
      <type>quick</type>
      <iters>50</iters>
      <sor>1.3</sor>
    </solver>
    <constraints>
      <cfm>0.0</cfm>
      <erp>0.2</erp>
      <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
      <contact_surface_layer>0.001</contact_surface_layer>
    </constraints>
  </ode>
</physics>
```

## Creating Complex Environments

### Warehouse Environment

```xml
<world name="warehouse">
  <!-- Floor -->
  <model name="floor">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <plane>
            <normal>0 0 1</normal>
            <size>100 100</size>
          </plane>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>100</mu>
              <mu2>50</mu2>
            </ode>
          </friction>
        </surface>
      </collision>
      <visual name="visual">
        <geometry>
          <plane>
            <normal>0 0 1</normal>
            <size>100 100</size>
          </plane>
        </geometry>
        <material>
          <ambient>0.8 0.8 0.8 1</ambient>
        </material>
      </visual>
    </link>
  </model>
  
  <!-- Shelving Units -->
  <model name="shelf_1">
    <pose>5 0 0 0 0 0</pose>
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <box>
            <size>2 0.5 3</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>2 0.5 3</size>
          </box>
        </geometry>
        <material>
          <ambient>0.5 0.3 0.1 1</ambient>
        </material>
      </visual>
    </link>
  </model>
  
  <!-- Boxes on shelves -->
  <model name="box_1">
    <pose>5 0 1.5 0 0 0</pose>
    <link name="link">
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.083</ixx>
          <iyy>0.083</iyy>
          <izz>0.083</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.3 0.3 0.3</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.3 0.3 0.3</size>
          </box>
        </geometry>
        <material>
          <ambient>0.8 0.5 0.2 1</ambient>
        </material>
      </visual>
    </link>
  </model>
</world>
```

## Gazebo Plugins

### Camera Plugin

```xml
<gazebo reference="camera_link">
  <sensor type="camera" name="camera1">
    <update_rate>30.0</update_rate>
    <camera name="head">
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>800</width>
        <height>600</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.02</near>
        <far>300</far>
      </clip>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.007</stddev>
      </noise>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <alwaysOn>true</alwaysOn>
      <updateRate>0.0</updateRate>
      <cameraName>robot/camera1</cameraName>
      <imageTopicName>image_raw</imageTopicName>
      <cameraInfoTopicName>camera_info</cameraInfoTopicName>
      <frameName>camera_link</frameName>
    </plugin>
  </sensor>
</gazebo>
```

### Differential Drive Plugin

```xml
<gazebo>
  <plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">
    <update_rate>50</update_rate>
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
    <wheel_separation>0.4</wheel_separation>
    <wheel_diameter>0.2</wheel_diameter>
    <max_wheel_torque>20</max_wheel_torque>
    <command_topic>cmd_vel</command_topic>
    <publish_odom>true</publish_odom>
    <publish_odom_tf>true</publish_odom_tf>
    <publish_wheel_tf>true</publish_wheel_tf>
    <odometry_topic>odom</odometry_topic>
    <odometry_frame>odom</odometry_frame>
    <robot_base_frame>base_link</robot_base_frame>
  </plugin>
</gazebo>
```

## Controlling Robots in Gazebo

### Velocity Control

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.move_robot)
        
    def move_robot(self):
        msg = Twist()
        msg.linear.x = 0.5  # Forward velocity
        msg.angular.z = 0.2  # Rotation
        self.publisher.publish(msg)

def main():
    rclpy.init()
    controller = RobotController()
    rclpy.spin(controller)
    rclpy.shutdown()
```

## Performance Optimization

### Tips for Faster Simulation

1. **Reduce visual complexity** - Use simple geometries
2. **Adjust physics rate** - Lower update rate if possible
3. **Use static models** - Mark non-moving objects as static
4. **Disable shadows** - For better performance
5. **Run headless** - Use gzserver only without GUI

```bash
# Headless mode
gzserver my_world.world
```

## Debugging Tools

### Gazebo GUI Tools

- **View → Transparent** - See through objects
- **View → Wireframe** - See collision geometries
- **View → Contacts** - Visualize contact points
- **View → Joints** - See joint axes

### Command Line Tools

```bash
# List models
gz model --list

# Get model info
gz model --model-name=my_robot --info

# Set model pose
gz model --model-name=my_robot --pose "1 2 0.5 0 0 0"

# Delete model
gz model --model-name=my_robot --delete
```

## Next Steps

Learn about high-fidelity rendering in [Unity Rendering](unity-rendering.md).
