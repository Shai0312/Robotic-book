---
sidebar_position: 3
---

# Links and Joints in Robot Modeling

## Understanding Links and Joints

In URDF (Unified Robot Description Format), **links** and **joints** form the fundamental building blocks of robot models. Together, they create the kinematic structure that defines how a robot moves and functions.

### Links: The Rigid Bodies

A **link** represents a rigid body in the robot - a physical component that doesn't deform. Each link contains information about:

- **Visual properties**: How the link appears in visualization tools
- **Collision properties**: How the link interacts in physics simulation
- **Inertial properties**: Mass, center of mass, and inertia tensor for dynamics

### Joints: The Connections

A **joint** connects two links and defines their relative motion. Joints specify:
- How two links are connected (parent-child relationship)
- The type of motion allowed between them
- Physical parameters like limits, damping, and friction

## Link Structure and Components

### Basic Link Definition

```xml
<link name="link_name">
  <visual>
    <!-- Visual appearance definition -->
  </visual>
  <collision>
    <!-- Collision properties definition -->
  </collision>
  <inertial>
    <!-- Mass and inertia properties -->
  </inertial>
</link>
```

### Visual Elements

The visual element defines how the link appears in visualization tools like RViz:

```xml
<visual>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <!-- Shape definition -->
    <cylinder length="0.1" radius="0.05"/>
  </geometry>
  <material name="blue">
    <color rgba="0 0 1 1"/>
  </material>
</visual>
```

#### Visual Sub-elements:
- **origin**: Position and orientation relative to the link frame
- **geometry**: Shape of the visual representation
- **material**: Color and appearance properties

### Collision Elements

The collision element defines the collision properties for physics simulation:

```xml
<collision>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <!-- Collision shape -->
    <cylinder length="0.1" radius="0.05"/>
  </geometry>
</collision>
```

#### Collision Sub-elements:
- **origin**: Position and orientation relative to the link frame
- **geometry**: Shape used for collision detection

### Inertial Elements

The inertial element defines mass properties for dynamics simulation:

```xml
<inertial>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <mass value="1.0"/>
  <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
</inertial>
```

#### Inertial Sub-elements:
- **origin**: Center of mass position relative to the link frame
- **mass**: Mass of the link in kilograms
- **inertia**: 3x3 inertia tensor (symmetric matrix)

## Joint Structure and Components

### Basic Joint Definition

```xml
<joint name="joint_name" type="joint_type">
  <parent link="parent_link_name"/>
  <child link="child_link_name"/>
  <origin xyz="x y z" rpy="roll pitch yaw"/>
  <!-- Additional joint properties -->
</joint>
```

### Joint Attributes

- **name**: Unique identifier for the joint
- **type**: Type of motion allowed (revolute, continuous, prismatic, etc.)

### Joint Types for Humanoid Robots

#### 1. Revolute Joints
Rotational joints with limited range of motion:

```xml
<joint name="elbow_joint" type="revolute">
  <parent link="upper_arm_link"/>
  <child link="lower_arm_link"/>
  <origin xyz="0 0 -0.3" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-2.0" upper="1.5" effort="100" velocity="2"/>
  <dynamics damping="0.1" friction="0.0"/>
</joint>
```

#### 2. Continuous Joints
Rotational joints with unlimited range of motion:

```xml
<joint name="continuous_joint" type="continuous">
  <parent link="base_link"/>
  <child link="rotating_part_link"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <dynamics damping="0.1"/>
</joint>
```

#### 3. Prismatic Joints
Linear sliding joints:

```xml
<joint name="prismatic_joint" type="prismatic">
  <parent link="base_link"/>
  <child link="slider_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="0" upper="0.5" effort="100" velocity="1"/>
</joint>
```

#### 4. Fixed Joints
Joints with no relative motion (used for attaching parts):

```xml
<joint name="fixed_joint" type="fixed">
  <parent link="main_body_link"/>
  <child link="sensor_mount_link"/>
  <origin xyz="0.1 0 0.2" rpy="0 0 0"/>
</joint>
```

## Geometry Types for Links

### Primitive Shapes

#### Box
```xml
<geometry>
  <box size="0.1 0.2 0.3"/>  <!-- width x depth x height -->
</geometry>
```

#### Cylinder
```xml
<geometry>
  <cylinder radius="0.05" length="0.2"/>
</geometry>
```

#### Sphere
```xml
<geometry>
  <sphere radius="0.05"/>
</geometry>
```

### Mesh Geometry
For complex shapes using external mesh files:

```xml
<geometry>
  <mesh filename="package://robot_description/meshes/part.stl"/>
</geometry>
```

## Practical Examples for Humanoid Robots

### Humanoid Arm Structure

```xml
<!-- Upper Arm Link -->
<link name="left_upper_arm_link">
  <visual>
    <geometry>
      <cylinder radius="0.04" length="0.3"/>
    </geometry>
    <material name="gray">
      <color rgba="0.5 0.5 0.5 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <cylinder radius="0.04" length="0.3"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.8"/>
    <inertia ixx="0.006" ixy="0" ixz="0" iyy="0.0008" iyz="0" izz="0.006"/>
  </inertial>
</link>

<!-- Shoulder Joint -->
<joint name="left_shoulder_pitch_joint" type="revolute">
  <parent link="torso_link"/>
  <child link="left_upper_arm_link"/>
  <origin xyz="0.1 0.2 0.3" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>  <!-- Rotate around X-axis -->
  <limit lower="-1.57" upper="1.57" effort="50" velocity="3"/>
</joint>

<!-- Lower Arm Link -->
<link name="left_lower_arm_link">
  <visual>
    <geometry>
      <cylinder radius="0.03" length="0.25"/>
    </geometry>
    <material name="gray">
      <color rgba="0.5 0.5 0.5 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <cylinder radius="0.03" length="0.25"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.6"/>
    <inertia ixx="0.003" ixy="0" ixz="0" iyy="0.0005" iyz="0" izz="0.003"/>
  </inertial>
</link>

<!-- Elbow Joint -->
<joint name="left_elbow_joint" type="revolute">
  <parent link="left_upper_arm_link"/>
  <child link="left_lower_arm_link"/>
  <origin xyz="0 0 -0.3" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>  <!-- Rotate around Y-axis -->
  <limit lower="0" upper="2.5" effort="40" velocity="3"/>
</joint>
```

### Humanoid Leg Structure

```xml
<!-- Thigh Link -->
<link name="left_thigh_link">
  <visual>
    <geometry>
      <cylinder radius="0.05" length="0.4"/>
    </geometry>
    <material name="black">
      <color rgba="0 0 0 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <cylinder radius="0.05" length="0.4"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="1.2"/>
    <inertia ixx="0.016" ixy="0" ixz="0" iyy="0.0012" iyz="0" izz="0.016"/>
  </inertial>
</link>

<!-- Hip Joint -->
<joint name="left_hip_pitch_joint" type="revolute">
  <parent link="torso_link"/>
  <child link="left_thigh_link"/>
  <origin xyz="0 -0.1 -0.1" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>
  <limit lower="-1.0" upper="0.5" effort="100" velocity="2"/>
</joint>

<!-- Shin Link -->
<link name="left_shin_link">
  <visual>
    <geometry>
      <cylinder radius="0.04" length="0.4"/>
    </geometry>
    <material name="black">
      <color rgba="0 0 0 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <cylinder radius="0.04" length="0.4"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="1.0"/>
    <inertia ixx="0.013" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.013"/>
  </inertial>
</link>

<!-- Knee Joint -->
<joint name="left_knee_joint" type="revolute">
  <parent link="left_thigh_link"/>
  <child link="left_shin_link"/>
  <origin xyz="0 0 -0.4" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>
  <limit lower="0" upper="2.0" effort="100" velocity="2"/>
</joint>
```

## Joint Limitations and Constraints

### Joint Limits
Define the operational range of revolute and prismatic joints:

```xml
<limit lower="-3.14" upper="3.14" effort="100" velocity="1"/>
```

- **lower/upper**: Position limits (radians for revolute, meters for prismatic)
- **effort**: Maximum effort (torque for revolute, force for prismatic)
- **velocity**: Maximum velocity

### Dynamics Properties
Define damping and friction characteristics:

```xml
<dynamics damping="0.1" friction="0.0"/>
```

## Best Practices for Links and Joints

### 1. Consistent Naming Conventions
Use descriptive names that indicate function and location:

```xml
<!-- Good naming -->
<link name="left_upper_arm_link"/>
<joint name="left_shoulder_pitch_joint"/>

<!-- Avoid generic names -->
<link name="link1"/>
<joint name="joint1"/>
```

### 2. Proper Mass Distribution
Ensure realistic inertial properties for stable simulation:

```xml
<!-- Example of realistic humanoid link -->
<inertial>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>  <!-- Center of mass offset -->
  <mass value="2.5"/>                   <!-- Realistic mass -->
  <inertia ixx="0.02" ixy="0" ixz="0"   <!-- Realistic inertia values -->
          iyy="0.02" iyz="0" izz="0.01"/>
</inertial>
```

### 3. Appropriate Joint Limits
Set realistic limits based on human anatomy or robot specifications:

```xml
<!-- Human-like shoulder joint limits -->
<joint name="left_shoulder_pitch_joint" type="revolute">
  <!-- ... -->
  <limit lower="-1.57" upper="1.57" effort="80" velocity="2"/>
</joint>
```

### 4. Collision Avoidance Planning
Consider self-collision scenarios when designing joint ranges:

```xml
<!-- Elbow joint with limits to prevent self-collision -->
<joint name="left_elbow_joint" type="revolute">
  <!-- ... -->
  <limit lower="0.1" upper="2.3" effort="60" velocity="3"/>
</joint>
```

## Common Pitfalls and Solutions

### 1. Incorrect Joint Axes
Ensure joint axes are correctly oriented for intended motion:

```xml
<!-- For shoulder pitch (forward/back movement) -->
<axis xyz="1 0 0"/>  <!-- Rotate around X-axis -->

<!-- For shoulder yaw (side-to-side movement) -->
<axis xyz="0 0 1"/>  <!-- Rotate around Z-axis -->
```

### 2. Misaligned Origins
Joint origins should be placed at the physical joint center:

```xml
<!-- Correct: Joint origin at shoulder joint center -->
<joint name="shoulder_joint">
  <origin xyz="0.1 0.2 0.3" rpy="0 0 0"/>  <!-- At joint location -->
  <!-- ... -->
</joint>
```

### 3. Unstable Inertial Properties
Use realistic mass and inertia values:

```xml
<!-- Avoid zero or negative values -->
<inertial>
  <mass value="0.1"/>  <!-- Must be positive -->
  <inertia ixx="0.001" iyy="0.001" izz="0.001"/>  <!-- Positive definite -->
</inertial>
```

## Tools for Validation

### URDF Validation
Use the `check_urdf` tool to validate your model:

```bash
check_urdf /path/to/your/robot.urdf
```

### Visualization
Use RViz or Gazebo to visualize and test your robot model:

```bash
# Launch with robot state publisher
ros2 launch urdf_tutorial display.launch.py model:=path/to/your/model.urdf
```

## Next Steps

In the next section, we'll explore frames and transformations in detail, learning how coordinate systems work in URDF and how they enable proper spatial relationships between robot components. We'll cover the TF (Transforms) system and how it relates to URDF links and joints.