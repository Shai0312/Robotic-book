---
title: Practical Examples - Gazebo Simulation
sidebar_label: Practical Examples
sidebar_position: 5
description: Practical examples and hands-on exercises for Gazebo simulation in digital twin applications
---

# Practical Examples - Gazebo Simulation

This section provides practical examples and hands-on exercises to help you apply the concepts learned about Gazebo simulation for digital twin applications. These examples will guide you through creating realistic humanoid robot simulations with physics and sensors.

## Example 1: Simple Humanoid Robot Model

Let's create a basic humanoid robot model with essential joints and physics properties.

### URDF Robot Definition

Create a file called `simple_humanoid.urdf`:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="neck_joint" type="revolute">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="100" velocity="1"/>
  </joint>

  <!-- Left Arm -->
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="left_shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.2 0 0.1" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="1"/>
  </joint>

  <!-- Right Arm -->
  <link name="right_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="right_shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="right_upper_arm"/>
    <origin xyz="-0.2 0 0.1" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="1"/>
  </joint>

  <!-- Left Leg -->
  <link name="left_upper_leg">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.02"/>
    </inertial>
  </link>

  <joint name="left_hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_upper_leg"/>
    <origin xyz="0.1 0 -0.2" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <!-- Right Leg -->
  <link name="right_upper_leg">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.02"/>
    </inertial>
  </link>

  <joint name="right_hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="right_upper_leg"/>
    <origin xyz="-0.1 0 -0.2" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <!-- Gazebo-specific configurations -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
  </gazebo>

  <gazebo reference="head">
    <material>Gazebo/White</material>
  </gazebo>

  <gazebo reference="left_upper_arm">
    <material>Gazebo/Red</material>
  </gazebo>

  <gazebo reference="right_upper_arm">
    <material>Gazebo/Red</material>
  </gazebo>

  <gazebo reference="left_upper_leg">
    <material>Gazebo/Green</material>
  </gazebo>

  <gazebo reference="right_upper_leg">
    <material>Gazebo/Green</material>
  </gazebo>
</robot>
```

### Launching the Robot in Gazebo

Create a launch file `simple_humanoid.launch`:

```xml
<launch>
  <!-- Load the URDF into the parameter server -->
  <param name="robot_description" textfile="$(find your_package)/urdf/simple_humanoid.urdf" />

  <!-- Start Gazebo with an empty world -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="paused" value="false"/>
    <arg name="use_sim_time" value="true"/>
    <arg name="gui" value="true"/>
    <arg name="headless" value="false"/>
    <arg name="debug" value="false"/>
  </include>

  <!-- Spawn the robot in Gazebo -->
  <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model"
        args="-param robot_description -urdf -model simple_humanoid" />

  <!-- Robot State Publisher -->
  <node name="robot_state_publisher" pkg="robot_state_publisher"
        type="robot_state_publisher" />
</launch>
```

## Example 2: Adding Sensors to the Robot

Now let's enhance our robot with sensors for digital twin capabilities.

### Enhanced URDF with LIDAR

Update the URDF to include a LIDAR sensor:

```xml
<!-- Add this to the head link definition -->
<link name="lidar_link">
  <visual>
    <geometry>
      <cylinder radius="0.05" length="0.04"/>
    </geometry>
    <material name="black">
      <color rgba="0 0 0 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <cylinder radius="0.05" length="0.04"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.1"/>
    <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
  </inertial>
</link>

<joint name="lidar_joint" type="fixed">
  <parent link="head"/>
  <child link="lidar_link"/>
  <origin xyz="0 0 0.05" rpy="0 0 0"/>
</joint>

<!-- Gazebo sensor configuration -->
<gazebo reference="lidar_link">
  <sensor name="lidar_sensor" type="ray">
    <always_on>true</always_on>
    <visualize>true</visualize>
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>10.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="lidar_controller" filename="libgazebo_ros_laser.so">
      <topicName>/laser_scan</topicName>
      <frameName>lidar_link</frameName>
    </plugin>
  </sensor>
</gazebo>
```

### Adding an IMU Sensor

Add an IMU to the torso for orientation data:

```xml
<!-- Add this inside the base_link gazebo block -->
<gazebo reference="base_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.0017</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.0017</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.0017</stddev>
          </noise>
        </z>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>
    <plugin name="imu_controller" filename="libgazebo_ros_imu.so">
      <topicName>/imu/data</topicName>
      <bodyName>base_link</bodyName>
      <frameName>base_link</frameName>
    </plugin>
  </sensor>
</gazebo>
```

## Example 3: Creating a Custom World

Create a custom world file `humanoid_test.world`:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="humanoid_test_world">
    <!-- Physics configuration -->
    <physics type="ode">
      <gravity>0 0 -9.8</gravity>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Lighting -->
    <light name="sun" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.3 0.3 -1</direction>
    </light>

    <!-- Ground plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.7 0.7 0.7 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
            <specular>0.0 0.0 0.0 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- Obstacles -->
    <model name="box_obstacle_1">
      <pose>2 0 0.5 0 0 0</pose>
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
            <ambient>0.8 0.3 0.3 1</ambient>
            <diffuse>0.8 0.3 0.3 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>1.0</mass>
          <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
        </inertial>
      </link>
    </model>

    <model name="box_obstacle_2">
      <pose>-2 2 0.3 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.6 0.6 0.6</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.6 0.6 0.6</size>
            </box>
          </geometry>
          <material>
            <ambient>0.3 0.3 0.8 1</ambient>
            <diffuse>0.3 0.3 0.8 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>1.0</mass>
          <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
        </inertial>
      </link>
    </model>

    <!-- Furniture for humanoid testing -->
    <model name="table">
      <pose>0 -3 0.4 0 0 0</pose>
      <link name="table_top">
        <collision name="collision">
          <geometry>
            <box>
              <size>1.5 0.8 0.02</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1.5 0.8 0.02</size>
            </box>
          </geometry>
          <material>
            <ambient>0.6 0.4 0.2 1</ambient>
            <diffuse>0.6 0.4 0.2 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>10.0</mass>
          <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
        </inertial>
      </link>
      <link name="leg1">
        <pose>0.7 0.35 -0.39 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.05 0.05 0.8</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.05 0.05 0.8</size>
            </box>
          </geometry>
          <material>
            <ambient>0.3 0.2 0.1 1</ambient>
            <diffuse>0.3 0.2 0.1 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>1.0</mass>
          <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
        </inertial>
      </link>
      <link name="leg2">
        <pose>-0.7 0.35 -0.39 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.05 0.05 0.8</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.05 0.05 0.8</size>
            </box>
          </geometry>
          <material>
            <ambient>0.3 0.2 0.1 1</ambient>
            <diffuse>0.3 0.2 0.1 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>1.0</mass>
          <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
        </inertial>
      </link>
      <link name="leg3">
        <pose>0.7 -0.35 -0.39 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.05 0.05 0.8</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.05 0.05 0.8</size>
            </box>
          </geometry>
          <material>
            <ambient>0.3 0.2 0.1 1</ambient>
            <diffuse>0.3 0.2 0.1 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>1.0</mass>
          <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
        </inertial>
      </link>
      <link name="leg4">
        <pose>-0.7 -0.35 -0.39 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.05 0.05 0.8</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.05 0.05 0.8</size>
            </box>
          </geometry>
          <material>
            <ambient>0.3 0.2 0.1 1</ambient>
            <diffuse>0.3 0.2 0.1 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>1.0</mass>
          <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
        </inertial>
      </link>
      <joint name="top_to_leg1" type="fixed">
        <parent>table_top</parent>
        <child>leg1</child>
      </joint>
      <joint name="top_to_leg2" type="fixed">
        <parent>table_top</parent>
        <child>leg2</child>
      </joint>
      <joint name="top_to_leg3" type="fixed">
        <parent>table_top</parent>
        <child>leg3</child>
      </joint>
      <joint name="top_to_leg4" type="fixed">
        <parent>table_top</parent>
        <child>leg4</child>
      </joint>
    </model>
  </world>
</sdf>
```

## Example 4: Running Basic Controllers

Create a simple controller to move the robot's joints:

### Python Controller Script

Create `simple_controller.py`:

```python
#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

class SimpleHumanoidController:
    def __init__(self):
        rospy.init_node('simple_humanoid_controller')

        # Publishers for joint positions
        self.left_shoulder_pub = rospy.Publisher('/simple_humanoid/left_shoulder_joint_position_controller/command',
                                                Float64, queue_size=1)
        self.right_shoulder_pub = rospy.Publisher('/simple_humanoid/right_shoulder_joint_position_controller/command',
                                                 Float64, queue_size=1)
        self.left_hip_pub = rospy.Publisher('/simple_humanoid/left_hip_joint_position_controller/command',
                                           Float64, queue_size=1)
        self.right_hip_pub = rospy.Publisher('/simple_humanoid/right_hip_joint_position_controller/command',
                                            Float64, queue_size=1)

        # Timer for periodic control updates
        self.timer = rospy.Timer(rospy.Duration(0.1), self.control_loop)

        # Time variable for oscillating motion
        self.time = 0.0

        rospy.loginfo("Simple Humanoid Controller initialized")

    def control_loop(self, event):
        # Update time
        self.time += 0.1

        # Create oscillating motion for joints
        left_shoulder_pos = 0.5 * math.sin(self.time)
        right_shoulder_pos = 0.5 * math.sin(self.time + math.pi)
        left_hip_pos = 0.3 * math.sin(self.time * 0.5)
        right_hip_pos = 0.3 * math.sin(self.time * 0.5 + math.pi)

        # Publish joint commands
        self.left_shoulder_pub.publish(Float64(left_shoulder_pos))
        self.right_shoulder_pub.publish(Float64(right_shoulder_pos))
        self.left_hip_pub.publish(Float64(left_hip_pos))
        self.right_hip_pub.publish(Float64(right_hip_pos))

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    controller = SimpleHumanoidController()
    controller.run()
```

## Example 5: Sensor Data Processing

Create a simple sensor data processor:

### Python Sensor Processor

Create `sensor_processor.py`:

```python
#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan, Imu
import numpy as np

class SensorProcessor:
    def __init__(self):
        rospy.init_node('sensor_processor')

        # Subscribe to sensor topics
        rospy.Subscriber('/laser_scan', LaserScan, self.laser_callback)
        rospy.Subscriber('/imu/data', Imu, self.imu_callback)

        # Store latest sensor data
        self.latest_scan = None
        self.latest_imu = None

        rospy.loginfo("Sensor Processor initialized")

    def laser_callback(self, scan_msg):
        self.latest_scan = scan_msg
        self.process_laser_data(scan_msg)

    def imu_callback(self, imu_msg):
        self.latest_imu = imu_msg
        self.process_imu_data(imu_msg)

    def process_laser_data(self, scan_msg):
        # Calculate minimum distance to obstacles
        valid_ranges = [r for r in scan_msg.ranges if not (r < scan_msg.range_min or r > scan_msg.range_max)]

        if valid_ranges:
            min_distance = min(valid_ranges)
            if min_distance < 1.0:  # If obstacle is within 1 meter
                rospy.logwarn(f"Obstacle detected at {min_distance:.2f} meters!")

    def process_imu_data(self, imu_msg):
        # Extract orientation (simplified)
        orientation = imu_msg.orientation
        rospy.loginfo_throttle(1.0, f"Orientation: ({orientation.x:.2f}, {orientation.y:.2f}, {orientation.z:.2f}, {orientation.w:.2f})")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    processor = SensorProcessor()
    processor.run()
```

## Exercise: Creating Your Own Digital Twin

### Step-by-Step Exercise

1. **Create a new robot model** based on the simple humanoid example
2. **Add at least 2 different sensors** (LIDAR, IMU, or camera)
3. **Create a custom environment** with obstacles
4. **Implement a simple controller** to move the robot
5. **Process sensor data** to detect obstacles or navigate

### Exercise Requirements

- Robot must have at least 6 degrees of freedom
- Include realistic physics properties
- Add at least 2 sensor types
- Create a world with at least 3 obstacles
- Implement basic obstacle avoidance behavior

### Validation Steps

1. **Visual Inspection**: Check that the robot appears correctly in Gazebo
2. **Physics Test**: Verify that the robot responds to gravity and collisions appropriately
3. **Sensor Test**: Confirm that sensors are publishing data on correct topics
4. **Control Test**: Verify that joint controllers respond to commands
5. **Integration Test**: Test the complete system with controller and sensor processing

## Troubleshooting Common Issues

### Robot Falls Through Ground
- Check that `base_link` has proper mass and inertia
- Verify that collision geometries are defined for all links
- Ensure the robot is spawned above ground level

### Sensors Not Publishing Data
- Verify that sensor plugins are loaded correctly
- Check topic names and connections
- Confirm that the simulation is running

### Joint Controllers Not Working
- Ensure controller configuration files are properly set up
- Check that joint names match between URDF and controller config
- Verify that the controller manager is running

## Advanced Exercise: Multi-Robot Simulation

For an advanced challenge, try creating a simulation with multiple humanoid robots that can interact with each other and the environment. This involves:

1. Creating unique namespaces for each robot
2. Implementing communication between robots
3. Adding coordination algorithms
4. Testing multi-robot scenarios

## Summary

These practical examples demonstrate the key concepts of creating digital twins in Gazebo:
- Building realistic robot models with proper physics
- Adding sensors for perception capabilities
- Creating appropriate environments for testing
- Implementing controllers and data processing
- Validating the complete system

By working through these examples, you'll gain hands-on experience with Gazebo simulation that's essential for digital twin applications in robotics.

## Exercises

1. Modify the simple humanoid to include more joints and make it more anthropomorphic
2. Create a navigation scenario where the robot must reach a goal while avoiding obstacles
3. Implement a sensor fusion algorithm that combines LIDAR and IMU data
4. Design a custom world that represents a real environment you want to simulate