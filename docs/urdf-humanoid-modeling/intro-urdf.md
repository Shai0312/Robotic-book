---
sidebar_position: 2
---

# Introduction to URDF

## What is URDF?

**URDF (Unified Robot Description Format)** is an XML-based format used to describe robot models in ROS. It provides a comprehensive way to define a robot's physical structure, kinematic properties, visual appearance, and collision characteristics.

### Purpose of URDF

URDF serves several critical functions in robotics:

1. **Physical Representation**: Defines the robot's mechanical structure
2. **Kinematic Model**: Specifies how parts move relative to each other
3. **Visualization**: Provides visual representation for tools like RViz
4. **Simulation**: Enables physics simulation in tools like Gazebo
5. **Control Integration**: Maps physical structure to control systems

### Why URDF is Essential for Humanoid Robots

Humanoid robots have complex structures that make URDF particularly valuable:

- **Multi-limb systems**: Arms, legs, head, torso with many joints
- **Balance requirements**: Accurate mass distribution for stability control
- **Collision detection**: Complex self-collision scenarios
- **Sensor integration**: Proper placement of cameras, IMUs, etc.

## URDF Structure Overview

### Basic XML Structure

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Links define rigid bodies -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Joints connect links -->
  <joint name="joint_name" type="revolute">
    <parent link="base_link"/>
    <child link="child_link"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="1"/>
  </joint>

  <link name="child_link">
    <!-- Link definition -->
  </link>
</robot>
```

### Core Components

1. **Robot Element**: Root element containing the entire robot description
2. **Links**: Rigid bodies that make up the robot structure
3. **Joints**: Connections between links that allow relative motion
4. **Visual Elements**: Define how the robot appears in visualization tools
5. **Collision Elements**: Define collision properties for simulation
6. **Inertial Elements**: Define mass properties for dynamics simulation

## Key URDF Elements for Humanoid Robots

### Robot Element

```xml
<robot name="humanoid_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Robot definition -->
</robot>
```

The robot element defines the root of the robot model and can include namespace declarations for advanced features like Xacro.

### Link Element

A link represents a rigid body in the robot:

```xml
<link name="head_link">
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <sphere radius="0.1"/>
    </geometry>
    <material name="white">
      <color rgba="1 1 1 1"/>
    </material>
  </visual>
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <sphere radius="0.1"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="1.0"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
  </inertial>
</link>
```

#### Link Sub-elements:

- **Visual**: Defines appearance in visualization
- **Collision**: Defines collision properties in simulation
- **Inertial**: Defines mass properties for dynamics

### Joint Element

A joint connects two links and defines their relative motion:

```xml
<joint name="neck_joint" type="revolute">
  <parent link="torso_link"/>
  <child link="head_link"/>
  <origin xyz="0 0 0.25" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-0.5" upper="0.5" effort="10" velocity="1"/>
  <dynamics damping="0.1" friction="0.0"/>
</joint>
```

#### Joint Attributes:

- **name**: Unique identifier for the joint
- **type**: Type of motion allowed (revolute, continuous, prismatic, etc.)

#### Joint Types for Humanoid Robots:

1. **Revolute**: Rotational joint with limited range
2. **Continuous**: Rotational joint with unlimited range
3. **Prismatic**: Linear sliding joint
4. **Fixed**: No relative motion between links
5. **Floating**: 6-DOF motion (rarely used)
6. **Planar**: Motion on a plane (rarely used)

## URDF in the ROS Ecosystem

### Robot State Publisher

The `robot_state_publisher` package uses URDF to:

- Compute forward kinematics
- Publish TF transforms between links
- Visualize robot configuration in RViz

### TF (Transforms)

URDF defines the static transforms between robot links that TF uses to:

- Maintain coordinate system relationships
- Transform data between different robot frames
- Enable spatial reasoning

### Gazebo Integration

Gazebo simulation uses URDF (via the `gazebo_ros` plugins) to:

- Create physics bodies for simulation
- Define collision properties
- Set up sensors and actuators

## Humanoid-Specific Considerations

### Multi-Tree Structure

Humanoid robots often have multiple kinematic chains (arms, legs) that can be:

- **Connected**: Through the torso/chest
- **Independent**: When hands are grasping different objects

### Balance and Stability

URDF models for humanoid robots must accurately represent:

- **Center of Mass**: Critical for balance control
- **Inertia Tensors**: Important for dynamic simulation
- **Foot Geometry**: For contact simulation and balance

### Sensor Placement

Humanoid robots typically include sensors in their URDF:

```xml
<gazebo reference="head_link">
  <sensor name="camera" type="camera">
    <!-- Camera configuration -->
  </sensor>
</gazebo>
```

## URDF Best Practices

### 1. Meaningful Naming

```xml
<!-- Good -->
<link name="left_upper_arm_link"/>
<joint name="left_shoulder_pitch_joint"/>

<!-- Avoid -->
<link name="link1"/>
<joint name="joint1"/>
```

### 2. Proper Tree Structure

URDF must form a tree (no loops), so plan your robot structure carefully:

```xml
<!-- Root -->
base_link
├── torso_link
│   ├── head_link
│   ├── left_shoulder_link
│   │   ├── left_elbow_link
│   │   └── left_wrist_link
│   └── right_shoulder_link
│       ├── right_elbow_link
│       └── right_wrist_link
└── left_hip_link
    ├── left_knee_link
    └── left_ankle_link
```

### 3. Units Consistency

- Length: meters
- Angles: radians
- Mass: kilograms
- Time: seconds

### 4. Origin Conventions

- Place origins at joint centers where possible
- Use consistent axis directions
- Consider how transformations will be used

## Tools for Working with URDF

### 1. check_urdf

Command-line tool to validate URDF files:

```bash
check_urdf /path/to/robot.urdf
```

### 2. urdf_to_graphiz

Visualize URDF structure:

```bash
urdf_to_graphiz /path/to/robot.urdf
```

### 3. RViz

Visualize robot model with joint state control.

### 4. Gazebo

Physics simulation and visualization.

## Common URDF Patterns for Humanoid Robots

### Simplified Humanoid Torso

```xml
<robot name="simple_humanoid">
  <link name="base_link"/>  <!-- Often coincides with world frame -->

  <joint name="torso_joint" type="fixed">
    <parent link="base_link"/>
    <child link="torso_link"/>
    <origin xyz="0 0 0.8" rpy="0 0 0"/>
  </joint>

  <link name="torso_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.5" ixy="0" ixz="0" iyy="0.8" iyz="0" izz="0.6"/>
    </inertial>
  </link>
</robot>
```

## Next Steps

In the next section, we'll dive deep into creating links and joints specifically for humanoid robots, exploring the various joint types and their applications in human-like robotic structures.