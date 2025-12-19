---
sidebar_position: 5
---

# URDF as the Robot's Physical Schema

## Understanding Physical Schema in Robotics

A **physical schema** in robotics is a comprehensive representation that defines the complete physical characteristics of a robot. URDF (Unified Robot Description Format) serves as the primary physical schema in ROS 2, providing a standardized way to describe a robot's structure, properties, and relationships. This schema acts as the foundational reference for all robot-related operations, from simulation to real-world control.

### The Role of Physical Schema

The physical schema serves as the "single source of truth" for robot properties, ensuring consistency across:
- Simulation environments
- Visualization tools
- Control systems
- Planning algorithms
- Perception systems

## URDF as Comprehensive Physical Representation

### Structural Information

URDF captures the complete structural hierarchy of a robot:

```xml
<robot name="humanoid_robot">
  <!-- Base structure -->
  <link name="base_link"/>

  <!-- Torso -->
  <joint name="torso_joint" type="fixed">
    <parent link="base_link"/>
    <child link="torso_link"/>
  </joint>
  <link name="torso_link"/>

  <!-- Limbs (arms and legs) -->
  <joint name="left_arm_joint" type="revolute">
    <parent link="torso_link"/>
    <child link="left_upper_arm_link"/>
  </joint>
  <!-- More links and joints... -->
</robot>
```

### Physical Properties

URDF defines the complete physical properties needed for accurate simulation:

- **Geometric properties**: Shape, size, and visual appearance
- **Mass properties**: Mass distribution and center of mass
- **Inertial properties**: Moments of inertia for dynamic simulation
- **Collision properties**: Shapes for collision detection

### Kinematic Properties

The schema defines how robot parts move relative to each other:

- **Joint types**: Revolute, prismatic, continuous, etc.
- **Joint limits**: Range of motion constraints
- **Kinematic chains**: How parts are connected and move together

## Integration with ROS 2 Ecosystem

### Robot State Publisher

The `robot_state_publisher` uses URDF to publish TF transforms:

```python
# robot_state_publisher node
import rclpy
from rclpy.node import Node
from robot_state_publisher import RobotStatePublisher

class MyRobotStatePublisher(Node):
    def __init__(self):
        super().__init__('robot_state_publisher')
        # Reads URDF from parameter server
        # Listens to joint_states topic
        # Publishes TF transforms based on URDF structure
```

### Joint State Management

URDF defines the joint structure that ROS 2 uses for state management:

```python
# Example joint state publisher
import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        # Joint names come from URDF joint definitions
        self.joint_names = [
            'left_shoulder_pitch_joint',
            'left_shoulder_yaw_joint',
            'left_elbow_joint',
            # ... all joints defined in URDF
        ]
```

### TF (Transforms) System

URDF provides the static transform relationships for the TF tree:

```xml
<!-- URDF defines static relationships -->
<joint name="camera_mount_joint" type="fixed">
  <parent link="head_link"/>
  <child link="camera_link"/>
  <origin xyz="0.1 0 0.05" rpy="0 0 0"/>
</joint>
```

The TF system then uses these relationships to maintain coordinate frame transformations.

## Simulation Integration

### Gazebo Physics Engine

Gazebo uses URDF to create physics bodies and simulation properties:

```xml
<!-- Gazebo-specific extensions in URDF -->
<gazebo reference="left_foot_link">
  <mu1>0.9</mu1>
  <mu2>0.9</mu2>
  <kp>1000000.0</kp>
  <kd>100.0</kd>
  <material>Gazebo/Blue</material>
</gazebo>
```

### Collision and Contact Simulation

URDF collision elements define how the robot interacts physically:

```xml
<link name="torso_link">
  <collision>
    <geometry>
      <box size="0.3 0.2 0.5"/>
    </geometry>
    <!-- Used by physics engine for collision detection -->
  </collision>
  <inertial>
    <mass value="10.0"/>
    <inertia ixx="0.5" ixy="0" ixz="0" iyy="0.8" iyz="0" izz="0.6"/>
    <!-- Used by physics engine for dynamics simulation -->
  </inertial>
</link>
```

## Control System Integration

### Joint Controllers

URDF joint definitions inform the control system about available joints:

```xml
<!-- URDF defines joint properties -->
<joint name="left_knee_joint" type="revolute">
  <parent link="left_thigh_link"/>
  <child link="left_shin_link"/>
  <limit lower="0" upper="2.0" effort="100" velocity="2"/>
  <axis xyz="1 0 0"/>
</joint>
```

### Hardware Interface Mapping

URDF can include hardware interface information:

```xml
<!-- Using ros2_control plugin for hardware interface -->
<gazebo>
  <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
    <parameters>$(find my_robot_description)/config/robot_controllers.yaml</parameters>
  </plugin>
</gazebo>
```

## Perception System Integration

### Sensor Integration

URDF defines where sensors are mounted on the robot:

```xml
<!-- IMU sensor mounted in torso -->
<joint name="imu_joint" type="fixed">
  <parent link="torso_link"/>
  <child link="imu_link"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
</joint>
<link name="imu_link"/>

<!-- Camera mounted on head -->
<joint name="camera_joint" type="fixed">
  <parent link="head_link"/>
  <child link="camera_link"/>
  <origin xyz="0.05 0 0.1" rpy="0 0 0"/>
</joint>
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.05 0.05 0.05"/>
    </geometry>
  </visual>
</link>
```

### Coordinate Frame Consistency

URDF ensures all perception systems use consistent coordinate frames:

```xml
<!-- All sensor data is interpreted relative to these frames -->
<!-- LIDAR data in base_laser_frame -->
<!-- Camera data in camera_frame -->
<!-- IMU data in imu_frame -->
```

## Dynamic Properties and Constraints

### Inertial Properties

Accurate inertial properties are crucial for dynamics simulation:

```xml
<link name="left_upper_arm_link">
  <inertial>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <mass value="0.8"/>
    <!-- 3x3 inertia tensor (symmetric) -->
    <inertia ixx="0.006" ixy="0" ixz="0"
             iyy="0.0008" iyz="0" izz="0.006"/>
  </inertial>
</link>
```

### Joint Dynamics

Joint dynamics properties affect simulation realism:

```xml
<joint name="left_shoulder_joint" type="revolute">
  <parent link="torso_link"/>
  <child link="left_upper_arm_link"/>
  <dynamics damping="0.5" friction="0.1"/>
  <!-- Damping and friction affect joint movement in simulation -->
</joint>
```

## URDF Extensions and Specialized Elements

### Gazebo-Specific Extensions

```xml
<!-- Gazebo plugins for sensors -->
<gazebo reference="camera_link">
  <sensor name="camera" type="camera">
    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so"/>
  </sensor>
</gazebo>
```

### Transmission Elements

For hardware interface definitions:

```xml
<transmission name="left_shoulder_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="left_shoulder_joint">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
  </joint>
  <actuator name="left_shoulder_motor">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

## Validation and Quality Assurance

### URDF Validation Tools

Use ROS 2 tools to validate your physical schema:

```bash
# Check URDF syntax and structure
check_urdf /path/to/your/robot.urdf

# Visualize the kinematic tree
urdf_to_graphiz /path/to/your/robot.urdf
```

### Physical Plausibility Checks

Validate that your physical schema makes sense:

```xml
<!-- Check that masses are realistic -->
<!-- Humanoid arm links should have reasonable mass values -->
<!-- Check that inertial tensors are physically valid -->
<!-- Verify joint limits are within reasonable ranges -->
```

## Practical Example: Complete Humanoid Schema

```xml
<?xml version="1.0"?>
<robot name="nao_like_humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Root link -->
  <link name="base_link">
    <inertial>
      <mass value="0.001"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- Torso -->
  <joint name="torso_joint" type="fixed">
    <parent link="base_link"/>
    <child link="torso_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>

  <link name="torso_link">
    <visual>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.15 0.4"/>
      </geometry>
      <material name="light_gray">
        <color rgba="0.7 0.7 0.7 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.15 0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <inertia ixx="0.04" ixy="0" ixz="0" iyy="0.08" iyz="0" izz="0.06"/>
    </inertial>
  </link>

  <!-- Head -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso_link"/>
    <child link="head_link"/>
    <origin xyz="0 0 0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="1"/>
  </joint>

  <link name="head_link">
    <visual>
      <geometry>
        <sphere radius="0.08"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.08"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.0011" ixy="0" ixz="0" iyy="0.0011" iyz="0" izz="0.0011"/>
    </inertial>
  </link>

  <!-- Left Arm -->
  <joint name="left_shoulder_pitch_joint" type="revolute">
    <parent link="torso_link"/>
    <child link="left_upper_arm_link"/>
    <origin xyz="0.05 0.1 0.3" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="2"/>
  </joint>

  <link name="left_upper_arm_link">
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <inertia ixx="0.006" ixy="0" ixz="0" iyy="0.006" iyz="0" izz="0.0008"/>
    </inertial>
  </link>

  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_upper_arm_link"/>
    <child link="left_lower_arm_link"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.5" effort="40" velocity="2"/>
  </joint>

  <link name="left_lower_arm_link">
    <visual>
      <origin xyz="0 0 -0.125" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.25"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.125" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.25"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.6"/>
      <origin xyz="0 0 -0.125" rpy="0 0 0"/>
      <inertia ixx="0.003" ixy="0" ixz="0" iyy="0.003" iyz="0" izz="0.0005"/>
    </inertial>
  </link>

  <!-- Left Hand -->
  <joint name="left_wrist_joint" type="revolute">
    <parent link="left_lower_arm_link"/>
    <child link="left_hand_link"/>
    <origin xyz="0 0 -0.25" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="20" velocity="3"/>
  </joint>

  <link name="left_hand_link">
    <visual>
      <geometry>
        <box size="0.1 0.08 0.05"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.08 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.0002" ixy="0" ixz="0" iyy="0.0003" iyz="0" izz="0.0002"/>
    </inertial>
  </link>

  <!-- Similar definitions for right arm, legs, etc. -->

</robot>
```

## Best Practices for Physical Schema Design

### 1. Realistic Physical Properties

Ensure all physical properties are realistic and consistent:

```xml
<!-- Good: Realistic mass distribution -->
<inertial>
  <mass value="2.0"/>  <!-- Realistic for torso -->
  <inertia ixx="0.04" iyy="0.08" izz="0.06"/>  <!-- Plausible ratios -->
</inertial>

<!-- Avoid: Unrealistic values -->
<inertial>
  <mass value="1000"/>  <!-- Too heavy -->
  <inertia ixx="0" iyy="0" izz="0"/>  <!-- Invalid -->
</inertial>
```

### 2. Consistent Units

Always use consistent SI units:

- Length: meters
- Mass: kilograms
- Angles: radians
- Time: seconds

### 3. Proper Inertial Tensors

Ensure inertia tensors are physically valid (positive definite):

```xml
<!-- Valid inertia tensor -->
<inertia ixx="0.04" ixy="0" ixz="0" iyy="0.08" iyz="0" izz="0.06"/>

<!-- Check: izz + iyy > ixx, ixx + izz > iyy, ixx + iyy > izz -->
<!-- 0.06 + 0.08 > 0.04 ✓ -->
<!-- 0.04 + 0.06 > 0.08 ✓ -->
<!-- 0.04 + 0.08 > 0.06 ✓ -->
```

### 4. Hierarchical Organization

Structure your URDF logically with clear parent-child relationships:

```xml
<!-- Logical hierarchy: base -> torso -> limbs -->
<!-- All limbs branch from appropriate body parts -->
<!-- Sensors are attached to the correct links -->
```

## Validation Strategies

### 1. Kinematic Validation

Verify the kinematic structure is correct:

```bash
# Check kinematic tree
ros2 run tf2_tools view_frames

# Verify transforms
ros2 run tf2_ros tf2_echo base_link left_hand_link
```

### 2. Physical Validation

Check that physical properties are realistic:

```bash
# Validate URDF structure
check_urdf /path/to/robot.urdf

# Check for physical plausibility
# - Mass values are reasonable
# - Inertial tensors are valid
# - Joint limits make sense
```

### 3. Simulation Validation

Test the physical schema in simulation:

```bash
# Launch in Gazebo
ros2 launch gazebo_ros gazebo.launch.py

# Spawn robot and verify behavior
# - Check for unstable joints
# - Verify collision detection
# - Test balance and stability
```

## Common Schema Issues and Solutions

### 1. Inconsistent Mass Distribution

Problem: Robot tips over due to incorrect center of mass.

Solution: Verify mass values and center of mass locations:

```xml
<!-- Ensure center of mass is physically realistic -->
<inertial>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>  <!-- Center of mass offset if needed -->
  <mass value="2.0"/>
  <inertia ixx="0.04" iyy="0.08" izz="0.06"/>
</inertial>
```

### 2. Invalid Inertial Tensors

Problem: Simulation instability due to invalid inertia values.

Solution: Use physically valid tensors:

```xml
<!-- Ensure triangle inequality holds for principal moments -->
<inertia ixx="0.04" iyy="0.08" izz="0.06"/>
<!-- 0.04 + 0.08 > 0.06, 0.04 + 0.06 > 0.08, 0.08 + 0.06 > 0.04 -->
```

### 3. Joint Limit Issues

Problem: Robot self-collides due to excessive joint ranges.

Solution: Set realistic joint limits:

```xml
<!-- Human-like joint limits -->
<limit lower="-1.57" upper="1.57" effort="50" velocity="2"/>
```

## Tools for Schema Development

### 1. URDF Validators

```bash
# Basic validation
check_urdf robot.urdf

# Visualization
urdf_to_graphiz robot.urdf
```

### 2. Simulation Testing

```bash
# Test in Gazebo
ros2 launch gazebo_ros spawn_entity.launch.py -entity robot -file robot.urdf

# Visualize in RViz
ros2 run rviz2 rviz2
```

### 3. TF Analysis

```bash
# Analyze transform tree
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo frame1 frame2
```

## Next Steps

In the next section, we'll explore practical humanoid model examples, demonstrating how to apply all these concepts to create complete, functional humanoid robot models. We'll walk through building a complete robot from scratch and examine real-world examples of humanoid robots implemented in URDF.