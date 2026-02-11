---
sidebar_position: 5
title: URDF for Humanoids
---

# URDF: Unified Robot Description Format

## Introduction

URDF (Unified Robot Description Format) is an XML format for representing robot models in ROS. For humanoid robots, URDF defines the kinematic and dynamic properties that make human-like movement possible.

## Why URDF Matters for Humanoids

Humanoid robots are complex systems with:
- 20+ degrees of freedom
- Bipedal locomotion requirements
- Precise balance control
- Human-like joint ranges

URDF provides the foundation for simulating and controlling these sophisticated machines.

## URDF Structure

### Basic Components

1. **Links** - Rigid bodies (bones)
2. **Joints** - Connections between links (joints)
3. **Visual** - How the robot looks
4. **Collision** - For physics simulation
5. **Inertial** - Mass and inertia properties

## Simple Humanoid Example

### Basic Structure

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  
  <!-- Base Link (Torso) -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.4" ixy="0.0" ixz="0.0"
               iyy="0.5" iyz="0.0" izz="0.3"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
      <material name="skin">
        <color rgba="0.9 0.7 0.6 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0"
               iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Neck Joint -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
  </joint>

</robot>
```

## Complete Humanoid Arm

```xml
<!-- Upper Arm -->
<link name="upper_arm_right">
  <visual>
    <geometry>
      <cylinder length="0.3" radius="0.05"/>
    </geometry>
    <origin xyz="0 0 -0.15" rpy="0 0 0"/>
  </visual>
  <collision>
    <geometry>
      <cylinder length="0.3" radius="0.05"/>
    </geometry>
    <origin xyz="0 0 -0.15" rpy="0 0 0"/>
  </collision>
  <inertial>
    <mass value="1.5"/>
    <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    <inertia ixx="0.01" ixy="0" ixz="0"
             iyy="0.01" iyz="0" izz="0.001"/>
  </inertial>
</link>

<!-- Shoulder Joint -->
<joint name="shoulder_right" type="revolute">
  <parent link="torso"/>
  <child link="upper_arm_right"/>
  <origin xyz="0 -0.2 0.2" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>
  <limit lower="-3.14" upper="3.14" effort="50" velocity="2.0"/>
  <dynamics damping="0.7" friction="0.0"/>
</joint>

<!-- Forearm -->
<link name="forearm_right">
  <visual>
    <geometry>
      <cylinder length="0.25" radius="0.04"/>
    </geometry>
    <origin xyz="0 0 -0.125" rpy="0 0 0"/>
  </visual>
  <collision>
    <geometry>
      <cylinder length="0.25" radius="0.04"/>
    </geometry>
    <origin xyz="0 0 -0.125" rpy="0 0 0"/>
  </collision>
  <inertial>
    <mass value="1.0"/>
    <origin xyz="0 0 -0.125" rpy="0 0 0"/>
    <inertia ixx="0.005" ixy="0" ixz="0"
             iyy="0.005" iyz="0" izz="0.0005"/>
  </inertial>
</link>

<!-- Elbow Joint -->
<joint name="elbow_right" type="revolute">
  <parent link="upper_arm_right"/>
  <child link="forearm_right"/>
  <origin xyz="0 0 -0.3" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="0" upper="2.5" effort="30" velocity="2.0"/>
  <dynamics damping="0.5" friction="0.0"/>
</joint>
```

## Humanoid Leg with Bipedal Support

```xml
<!-- Thigh -->
<link name="thigh_right">
  <visual>
    <geometry>
      <cylinder length="0.4" radius="0.06"/>
    </geometry>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
  </visual>
  <inertial>
    <mass value="3.0"/>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    <inertia ixx="0.04" ixy="0" ixz="0"
             iyy="0.04" iyz="0" izz="0.002"/>
  </inertial>
</link>

<!-- Hip Joint (3 DOF using multiple joints) -->
<joint name="hip_yaw_right" type="revolute">
  <parent link="torso"/>
  <child link="hip_pitch_link_right"/>
  <origin xyz="0 -0.1 -0.25" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-0.5" upper="0.5" effort="100" velocity="1.0"/>
</joint>

<joint name="hip_pitch_right" type="revolute">
  <parent link="hip_pitch_link_right"/>
  <child link="hip_roll_link_right"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
</joint>

<joint name="hip_roll_right" type="revolute">
  <parent link="hip_roll_link_right"/>
  <child link="thigh_right"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>
  <limit lower="-0.5" upper="0.5" effort="100" velocity="1.0"/>
</joint>

<!-- Shin -->
<link name="shin_right">
  <visual>
    <geometry>
      <cylinder length="0.4" radius="0.05"/>
    </geometry>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
  </visual>
  <inertial>
    <mass value="2.0"/>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    <inertia ixx="0.027" ixy="0" ixz="0"
             iyy="0.027" iyz="0" izz="0.001"/>
  </inertial>
</link>

<!-- Knee Joint -->
<joint name="knee_right" type="revolute">
  <parent link="thigh_right"/>
  <child link="shin_right"/>
  <origin xyz="0 0 -0.4" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="0" upper="2.5" effort="80" velocity="1.5"/>
</joint>

<!-- Foot -->
<link name="foot_right">
  <visual>
    <geometry>
      <box size="0.2 0.1 0.05"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <box size="0.2 0.1 0.05"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.5"/>
    <inertia ixx="0.001" ixy="0" ixz="0"
             iyy="0.002" iyz="0" izz="0.002"/>
  </inertial>
</link>

<!-- Ankle Joint -->
<joint name="ankle_right" type="revolute">
  <parent link="shin_right"/>
  <child link="foot_right"/>
  <origin xyz="0 0 -0.4" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-0.7" upper="0.7" effort="50" velocity="1.0"/>
</joint>
```

## Joint Types

### Revolute (Rotational)
```xml
<joint name="elbow" type="revolute">
  <limit lower="-3.14" upper="3.14" effort="100" velocity="2.0"/>
</joint>
```

### Continuous (Unlimited Rotation)
```xml
<joint name="wheel" type="continuous">
  <limit effort="100" velocity="10.0"/>
</joint>
```

### Prismatic (Linear)
```xml
<joint name="slider" type="prismatic">
  <limit lower="0" upper="1.0" effort="100" velocity="1.0"/>
</joint>
```

### Fixed
```xml
<joint name="camera_mount" type="fixed">
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
</joint>
```

## Adding Sensors

### Camera

```xml
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.05 0.05 0.05"/>
    </geometry>
  </visual>
</link>

<joint name="camera_joint" type="fixed">
  <parent link="head"/>
  <child link="camera_link"/>
  <origin xyz="0.1 0 0" rpy="0 0 0"/>
</joint>

<!-- Gazebo plugin for camera -->
<gazebo reference="camera_link">
  <sensor type="camera" name="head_camera">
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
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <alwaysOn>true</alwaysOn>
      <updateRate>0.0</updateRate>
      <cameraName>head_camera</cameraName>
      <imageTopicName>image_raw</imageTopicName>
      <cameraInfoTopicName>camera_info</cameraInfoTopicName>
      <frameName>camera_link</frameName>
    </plugin>
  </sensor>
</gazebo>
```

### IMU (Inertial Measurement Unit)

```xml
<gazebo reference="torso">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <plugin filename="libgazebo_ros_imu_sensor.so" name="imu_plugin">
      <ros>
        <namespace>/humanoid</namespace>
        <remapping>~/out:=imu</remapping>
      </ros>
      <initial_orientation_as_reference>false</initial_orientation_as_reference>
    </plugin>
  </sensor>
</gazebo>
```

## Visualizing URDF

### Using RViz2

```bash
# Install URDF tools
sudo apt install ros-humble-urdf-tutorial

# Check URDF validity
check_urdf my_humanoid.urdf

# Visualize in RViz2
ros2 launch urdf_tutorial display.launch.py model:=my_humanoid.urdf
```

### Python Script to Load URDF

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class HumanoidController(Node):
    def __init__(self):
        super().__init__('humanoid_controller')
        
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_joint_states)
        
        self.joint_names = [
            'neck_joint',
            'shoulder_right', 'elbow_right',
            'shoulder_left', 'elbow_left',
            'hip_pitch_right', 'knee_right', 'ankle_right',
            'hip_pitch_left', 'knee_left', 'ankle_left'
        ]
        
        self.angle = 0.0
    
    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        
        # Simulate walking motion
        self.angle += 0.1
        msg.position = [
            0.0,  # neck
            math.sin(self.angle) * 0.5, math.cos(self.angle) * 0.3,  # right arm
            math.sin(self.angle) * 0.5, math.cos(self.angle) * 0.3,  # left arm
            math.sin(self.angle) * 0.3, math.cos(self.angle) * 0.5, 0.0,  # right leg
            -math.sin(self.angle) * 0.3, math.cos(self.angle) * 0.5, 0.0,  # left leg
        ]
        
        self.publisher.publish(msg)

def main():
    rclpy.init()
    controller = HumanoidController()
    rclpy.spin(controller)
    rclpy.shutdown()
```

## Best Practices for Humanoid URDF

1. **Accurate Mass Properties** - Critical for balance
2. **Realistic Joint Limits** - Match human range of motion
3. **Proper Collision Geometry** - Simpler than visual
4. **Center of Mass** - Place correctly for stability
5. **Damping and Friction** - Add to joints for realism
6. **Modular Design** - Use xacro for reusability

## Using Xacro for Modularity

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid">
  
  <xacro:property name="arm_length" value="0.3"/>
  <xacro:property name="leg_length" value="0.4"/>
  
  <xacro:macro name="arm" params="side reflect">
    <link name="upper_arm_${side}">
      <!-- arm definition -->
    </link>
    
    <joint name="shoulder_${side}" type="revolute">
      <origin xyz="0 ${reflect * 0.2} 0.2" rpy="0 0 0"/>
      <!-- joint definition -->
    </joint>
  </xacro:macro>
  
  <!-- Instantiate both arms -->
  <xacro:arm side="right" reflect="-1"/>
  <xacro:arm side="left" reflect="1"/>
  
</robot>
```

## Next Steps

You've completed Module 1! Move on to [Module 2: The Digital Twin](../module2/overview.md) to learn about simulation environments.
