---
sidebar_position: 6
---

# Practical Humanoid Model Examples

## Building Complete Humanoid Models

In this section, we'll work through practical examples of creating complete humanoid robot models. We'll start with simple models and gradually build up to more complex examples, demonstrating best practices and common patterns used in humanoid robotics.

## Example 1: Simple Biped Robot

Let's start with a basic bipedal robot that has the essential components of a humanoid: torso, head, arms, and legs.

```xml
<?xml version="1.0"?>
<robot name="simple_biped">

  <!-- Base/Fixed frame -->
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
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
  </joint>

  <link name="torso_link">
    <visual>
      <geometry>
        <box size="0.2 0.15 0.4"/>
      </geometry>
      <material name="light_gray">
        <color rgba="0.7 0.7 0.7 1"/>
      </material>
    </visual>
    <collision>
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
  <joint name="left_shoulder_joint" type="revolute">
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

  <!-- Right Arm (symmetric to left) -->
  <joint name="right_shoulder_joint" type="revolute">
    <parent link="torso_link"/>
    <child link="right_upper_arm_link"/>
    <origin xyz="0.05 -0.1 0.3" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="2"/>
  </joint>

  <link name="right_upper_arm_link">
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

  <joint name="right_elbow_joint" type="revolute">
    <parent link="right_upper_arm_link"/>
    <child link="right_lower_arm_link"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.5" effort="40" velocity="2"/>
  </joint>

  <link name="right_lower_arm_link">
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

  <!-- Left Leg -->
  <joint name="left_hip_joint" type="revolute">
    <parent link="torso_link"/>
    <child link="left_thigh_link"/>
    <origin xyz="0 0.05 0" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.0" upper="0.5" effort="100" velocity="2"/>
  </joint>

  <link name="left_thigh_link">
    <visual>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.2"/>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <inertia ixx="0.016" ixy="0" ixz="0" iyy="0.016" iyz="0" izz="0.0012"/>
    </inertial>
  </link>

  <joint name="left_knee_joint" type="revolute">
    <parent link="left_thigh_link"/>
    <child link="left_shin_link"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="0" upper="2.0" effort="100" velocity="2"/>
  </joint>

  <link name="left_shin_link">
    <visual>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.4"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <inertia ixx="0.013" ixy="0" ixz="0" iyy="0.013" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Left Foot -->
  <joint name="left_ankle_joint" type="revolute">
    <parent link="left_shin_link"/>
    <child link="left_foot_link"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="50" velocity="1"/>
  </joint>

  <link name="left_foot_link">
    <visual>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.0003" ixy="0" ixz="0" iyy="0.0008" iyz="0" izz="0.0008"/>
    </inertial>
  </link>

  <!-- Right Leg (symmetric to left) -->
  <joint name="right_hip_joint" type="revolute">
    <parent link="torso_link"/>
    <child link="right_thigh_link"/>
    <origin xyz="0 -0.05 0" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.0" upper="0.5" effort="100" velocity="2"/>
  </joint>

  <link name="right_thigh_link">
    <visual>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.2"/>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <inertia ixx="0.016" ixy="0" ixz="0" iyy="0.016" iyz="0" izz="0.0012"/>
    </inertial>
  </link>

  <joint name="right_knee_joint" type="revolute">
    <parent link="right_thigh_link"/>
    <child link="right_shin_link"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="0" upper="2.0" effort="100" velocity="2"/>
  </joint>

  <link name="right_shin_link">
    <visual>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.4"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <inertia ixx="0.013" ixy="0" ixz="0" iyy="0.013" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Right Foot -->
  <joint name="right_ankle_joint" type="revolute">
    <parent link="right_shin_link"/>
    <child link="right_foot_link"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="50" velocity="1"/>
  </joint>

  <link name="right_foot_link">
    <visual>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.0003" ixy="0" ixz="0" iyy="0.0008" iyz="0" izz="0.0008"/>
    </inertial>
  </link>

</robot>
```

## Example 2: NAO-like Humanoid with Sensors

Now let's create a more advanced model similar to the popular NAO humanoid robot, including sensors:

```xml
<?xml version="1.0"?>
<robot name="nao_like_robot">

  <!-- Base link -->
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
      <geometry>
        <box size="0.15 0.12 0.3"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.15 0.12 0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.03" iyz="0" izz="0.025"/>
    </inertial>
  </link>

  <!-- Head -->
  <joint name="head_yaw_joint" type="revolute">
    <parent link="torso_link"/>
    <child link="head_link"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.0" upper="1.0" effort="10" velocity="1"/>
  </joint>

  <link name="head_link">
    <visual>
      <geometry>
        <sphere radius="0.07"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.07"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.4"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.0007" ixy="0" ixz="0" iyy="0.0007" iyz="0" izz="0.0007"/>
    </inertial>
  </link>

  <!-- Camera in head -->
  <joint name="camera_joint" type="fixed">
    <parent link="head_link"/>
    <child link="camera_link"/>
    <origin xyz="0.05 0 0.02" rpy="0 0 0"/>
  </joint>

  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.03 0.03 0.02"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <inertial>
      <mass value="0.05"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.000001" ixy="0" ixz="0" iyy="0.000001" iyz="0" izz="0.000001"/>
    </inertial>
  </link>

  <!-- IMU in torso -->
  <joint name="imu_joint" type="fixed">
    <parent link="torso_link"/>
    <child link="imu_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>

  <link name="imu_link"/>

  <!-- Left Arm with 5 DOF -->
  <joint name="l_shoulder_pitch_joint" type="revolute">
    <parent link="torso_link"/>
    <child link="l_shoulder_link"/>
    <origin xyz="0.04 0.08 0.2" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.5" upper="1.5" effort="50" velocity="2"/>
  </joint>

  <link name="l_shoulder_link">
    <visual>
      <origin xyz="0 0 -0.07" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.14"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.07" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.14"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 -0.07" rpy="0 0 0"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.0002"/>
    </inertial>
  </link>

  <joint name="l_shoulder_roll_joint" type="revolute">
    <parent link="l_shoulder_link"/>
    <child link="l_upper_arm_link"/>
    <origin xyz="0 0 -0.14" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.0" effort="50" velocity="2"/>
  </joint>

  <link name="l_upper_arm_link">
    <visual>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.025" length="0.2"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.025" length="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.6"/>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <inertia ixx="0.002" ixy="0" ixz="0" iyy="0.002" iyz="0" izz="0.0003"/>
    </inertial>
  </link>

  <joint name="l_elbow_yaw_joint" type="revolute">
    <parent link="l_upper_arm_link"/>
    <child link="l_lower_arm_link"/>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.5" upper="1.5" effort="40" velocity="2"/>
  </joint>

  <link name="l_lower_arm_link">
    <visual>
      <origin xyz="0 0 -0.08" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.02" length="0.16"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.08" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.02" length="0.16"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.4"/>
      <origin xyz="0 0 -0.08" rpy="0 0 0"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.0002"/>
    </inertial>
  </link>

  <!-- Right Arm (symmetric) -->
  <joint name="r_shoulder_pitch_joint" type="revolute">
    <parent link="torso_link"/>
    <child link="r_shoulder_link"/>
    <origin xyz="0.04 -0.08 0.2" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.5" upper="1.5" effort="50" velocity="2"/>
  </joint>

  <link name="r_shoulder_link">
    <visual>
      <origin xyz="0 0 -0.07" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.14"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.07" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.14"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 -0.07" rpy="0 0 0"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.0002"/>
    </inertial>
  </link>

  <joint name="r_shoulder_roll_joint" type="revolute">
    <parent link="r_shoulder_link"/>
    <child link="r_upper_arm_link"/>
    <origin xyz="0 0 -0.14" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.0" effort="50" velocity="2"/>
  </joint>

  <link name="r_upper_arm_link">
    <visual>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.025" length="0.2"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.025" length="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.6"/>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <inertia ixx="0.002" ixy="0" ixz="0" iyy="0.002" iyz="0" izz="0.0003"/>
    </inertial>
  </link>

  <joint name="r_elbow_yaw_joint" type="revolute">
    <parent link="r_upper_arm_link"/>
    <child link="r_lower_arm_link"/>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.5" upper="1.5" effort="40" velocity="2"/>
  </joint>

  <link name="r_lower_arm_link">
    <visual>
      <origin xyz="0 0 -0.08" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.02" length="0.16"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.08" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.02" length="0.16"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.4"/>
      <origin xyz="0 0 -0.08" rpy="0 0 0"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.0002"/>
    </inertial>
  </link>

  <!-- Left Leg with 6 DOF -->
  <joint name="l_hip_yaw_joint" type="revolute">
    <parent link="torso_link"/>
    <child link="l_hip_link"/>
    <origin xyz="0 0.04 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.0" upper="1.0" effort="100" velocity="1"/>
  </joint>

  <link name="l_hip_link">
    <visual>
      <origin xyz="0 0 -0.07" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.14"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.07" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.14"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <origin xyz="0 0 -0.07" rpy="0 0 0"/>
      <inertia ixx="0.002" ixy="0" ixz="0" iyy="0.002" iyz="0" izz="0.0008"/>
    </inertial>
  </link>

  <joint name="l_hip_roll_joint" type="revolute">
    <parent link="l_hip_link"/>
    <child link="l_thigh_link"/>
    <origin xyz="0 0 -0.14" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="100" velocity="1"/>
  </joint>

  <link name="l_thigh_link">
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.035" length="0.3"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.035" length="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <inertia ixx="0.008" ixy="0" ixz="0" iyy="0.008" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="l_knee_pitch_joint" type="revolute">
    <parent link="l_thigh_link"/>
    <child link="l_shin_link"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="0" upper="2.0" effort="100" velocity="1"/>
  </joint>

  <link name="l_shin_link">
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.3"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <inertia ixx="0.006" ixy="0" ixz="0" iyy="0.006" iyz="0" izz="0.0008"/>
    </inertial>
  </link>

  <joint name="l_ankle_pitch_joint" type="revolute">
    <parent link="l_shin_link"/>
    <child link="l_foot_link"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-0.5" upper="0.5" effort="50" velocity="1"/>
  </joint>

  <link name="l_foot_link">
    <visual>
      <geometry>
        <box size="0.12 0.06 0.04"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.12 0.06 0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.4"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.0004" ixy="0" ixz="0" iyy="0.0008" iyz="0" izz="0.0008"/>
    </inertial>
  </link>

  <!-- Right Leg (symmetric) -->
  <joint name="r_hip_yaw_joint" type="revolute">
    <parent link="torso_link"/>
    <child link="r_hip_link"/>
    <origin xyz="0 -0.04 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.0" upper="1.0" effort="100" velocity="1"/>
  </joint>

  <link name="r_hip_link">
    <visual>
      <origin xyz="0 0 -0.07" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.14"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.07" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.14"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <origin xyz="0 0 -0.07" rpy="0 0 0"/>
      <inertia ixx="0.002" ixy="0" ixz="0" iyy="0.002" iyz="0" izz="0.0008"/>
    </inertial>
  </link>

  <joint name="r_hip_roll_joint" type="revolute">
    <parent link="r_hip_link"/>
    <child link="r_thigh_link"/>
    <origin xyz="0 0 -0.14" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="100" velocity="1"/>
  </joint>

  <link name="r_thigh_link">
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.035" length="0.3"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.035" length="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <inertia ixx="0.008" ixy="0" ixz="0" iyy="0.008" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="r_knee_pitch_joint" type="revolute">
    <parent link="r_thigh_link"/>
    <child link="r_shin_link"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="0" upper="2.0" effort="100" velocity="1"/>
  </joint>

  <link name="r_shin_link">
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.3"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <inertia ixx="0.006" ixy="0" ixz="0" iyy="0.006" iyz="0" izz="0.0008"/>
    </inertial>
  </link>

  <joint name="r_ankle_pitch_joint" type="revolute">
    <parent link="r_shin_link"/>
    <child link="r_foot_link"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-0.5" upper="0.5" effort="50" velocity="1"/>
  </joint>

  <link name="r_foot_link">
    <visual>
      <geometry>
        <box size="0.12 0.06 0.04"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.12 0.06 0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.4"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.0004" ixy="0" ixz="0" iyy="0.0008" iyz="0" izz="0.0008"/>
    </inertial>
  </link>

</robot>
```

## Example 3: Modular Humanoid Design with Xacro

For more complex robots, we can use Xacro to create modular designs that are easier to maintain:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="modular_humanoid">

  <!-- Define constants -->
  <xacro:property name="M_PI" value="3.1415926535897931" />

  <!-- Materials -->
  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>
  <material name="black">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>
  <material name="white">
    <color rgba="1.0 1.0 1.0 1.0"/>
  </material>
  <material name="gray">
    <color rgba="0.5 0.5 0.5 1.0"/>
  </material>

  <!-- Macro for a simple cylinder link -->
  <xacro:macro name="cylinder_link" params="name radius length mass xyz rpy material">
    <link name="${name}">
      <visual>
        <origin xyz="${xyz}" rpy="${rpy}"/>
        <geometry>
          <cylinder radius="${radius}" length="${length}"/>
        </geometry>
        <material name="${material}"/>
      </visual>
      <collision>
        <origin xyz="${xyz}" rpy="${rpy}"/>
        <geometry>
          <cylinder radius="${radius}" length="${length}"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="${mass}"/>
        <origin xyz="${xyz}" rpy="${rpy}"/>
        <inertia
          ixx="${0.0833333 * mass * (3*radius*radius + length*length)}"
          ixy="0" ixz="0"
          iyy="${0.0833333 * mass * (3*radius*radius + length*length)}"
          iyz="0"
          izz="${0.5 * mass * radius * radius}"/>
      </inertial>
    </link>
  </xacro:macro>

  <!-- Macro for a simple joint -->
  <xacro:macro name="joint" params="name type parent child xyz rpy axis lower upper effort velocity">
    <joint name="${name}" type="${type}">
      <parent link="${parent}"/>
      <child link="${child}"/>
      <origin xyz="${xyz}" rpy="${rpy}"/>
      <axis xyz="${axis}"/>
      <xacro:if value="${type != 'fixed'}">
        <limit lower="${lower}" upper="${upper}" effort="${effort}" velocity="${velocity}"/>
      </xacro:if>
    </joint>
  </xacro:macro>

  <!-- Base link -->
  <link name="base_link">
    <inertial>
      <mass value="0.001"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- Torso -->
  <xacro:cylinder_link name="torso_link" radius="0.08" length="0.4" mass="2.0"
                      xyz="0 0 0.2" rpy="0 0 0" material="white"/>
  <xacro:joint name="torso_joint" type="fixed" parent="base_link" child="torso_link"
               xyz="0 0 0.1" rpy="0 0 0" axis="1 0 0"
               lower="0" upper="0" effort="0" velocity="0"/>

  <!-- Head -->
  <link name="head_link">
    <visual>
      <geometry>
        <sphere radius="0.08"/>
      </geometry>
      <material name="white"/>
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
  <xacro:joint name="neck_joint" type="revolute" parent="torso_link" child="head_link"
               xyz="0 0 0.4" rpy="0 0 0" axis="0 1 0"
               lower="-0.5" upper="0.5" effort="10" velocity="1"/>

  <!-- Left Arm Macro -->
  <xacro:macro name="left_arm" params="prefix">
    <!-- Shoulder -->
    <xacro:cylinder_link name="${prefix}_shoulder_link" radius="0.04" length="0.1"
                        mass="0.5" xyz="0 0 -0.05" rpy="0 0 0" material="gray"/>
    <xacro:joint name="${prefix}_shoulder_joint" type="revolute"
                 parent="torso_link" child="${prefix}_shoulder_link"
                 xyz="0.05 0.1 0.3" rpy="0 0 0" axis="1 0 0"
                 lower="-1.57" upper="1.57" effort="50" velocity="2"/>

    <!-- Upper Arm -->
    <xacro:cylinder_link name="${prefix}_upper_arm_link" radius="0.035" length="0.3"
                        mass="0.8" xyz="0 0 -0.15" rpy="0 0 0" material="gray"/>
    <xacro:joint name="${prefix}_elbow_joint" type="revolute"
                 parent="${prefix}_shoulder_link" child="${prefix}_upper_arm_link"
                 xyz="0 0 -0.1" rpy="0 0 0" axis="0 1 0"
                 lower="0" upper="2.5" effort="40" velocity="2"/>

    <!-- Lower Arm -->
    <xacro:cylinder_link name="${prefix}_lower_arm_link" radius="0.03" length="0.25"
                        mass="0.6" xyz="0 0 -0.125" rpy="0 0 0" material="gray"/>
    <xacro:joint name="${prefix}_wrist_joint" type="revolute"
                 parent="${prefix}_upper_arm_link" child="${prefix}_lower_arm_link"
                 xyz="0 0 -0.3" rpy="0 0 0" axis="0 0 1"
                 lower="-1.57" upper="1.57" effort="20" velocity="3"/>
  </xacro:macro>

  <!-- Right Arm Macro -->
  <xacro:macro name="right_arm" params="prefix">
    <!-- Shoulder -->
    <xacro:cylinder_link name="${prefix}_shoulder_link" radius="0.04" length="0.1"
                        mass="0.5" xyz="0 0 -0.05" rpy="0 0 0" material="gray"/>
    <xacro:joint name="${prefix}_shoulder_joint" type="revolute"
                 parent="torso_link" child="${prefix}_shoulder_link"
                 xyz="0.05 -0.1 0.3" rpy="0 0 0" axis="1 0 0"
                 lower="-1.57" upper="1.57" effort="50" velocity="2"/>

    <!-- Upper Arm -->
    <xacro:cylinder_link name="${prefix}_upper_arm_link" radius="0.035" length="0.3"
                        mass="0.8" xyz="0 0 -0.15" rpy="0 0 0" material="gray"/>
    <xacro:joint name="${prefix}_elbow_joint" type="revolute"
                 parent="${prefix}_shoulder_link" child="${prefix}_upper_arm_link"
                 xyz="0 0 -0.1" rpy="0 0 0" axis="0 1 0"
                 lower="0" upper="2.5" effort="40" velocity="2"/>

    <!-- Lower Arm -->
    <xacro:cylinder_link name="${prefix}_lower_arm_link" radius="0.03" length="0.25"
                        mass="0.6" xyz="0 0 -0.125" rpy="0 0 0" material="gray"/>
    <xacro:joint name="${prefix}_wrist_joint" type="revolute"
                 parent="${prefix}_upper_arm_link" child="${prefix}_lower_arm_link"
                 xyz="0 0 -0.3" rpy="0 0 0" axis="0 0 1"
                 lower="-1.57" upper="1.57" effort="20" velocity="3"/>
  </xacro:macro>

  <!-- Instantiate arms -->
  <xacro:left_arm prefix="l_arm"/>
  <xacro:right_arm prefix="r_arm"/>

  <!-- Left Leg Macro -->
  <xacro:macro name="left_leg" params="prefix">
    <!-- Thigh -->
    <xacro:cylinder_link name="${prefix}_thigh_link" radius="0.05" length="0.4"
                        mass="1.2" xyz="0 0 -0.2" rpy="0 0 0" material="black"/>
    <xacro:joint name="${prefix}_hip_joint" type="revolute"
                 parent="torso_link" child="${prefix}_thigh_link"
                 xyz="0 0.05 0" rpy="0 0 0" axis="1 0 0"
                 lower="-1.0" upper="0.5" effort="100" velocity="2"/>

    <!-- Shin -->
    <xacro:cylinder_link name="${prefix}_shin_link" radius="0.04" length="0.4"
                        mass="1.0" xyz="0 0 -0.2" rpy="0 0 0" material="black"/>
    <xacro:joint name="${prefix}_knee_joint" type="revolute"
                 parent="${prefix}_thigh_link" child="${prefix}_shin_link"
                 xyz="0 0 -0.4" rpy="0 0 0" axis="1 0 0"
                 lower="0" upper="2.0" effort="100" velocity="2"/>

    <!-- Foot -->
    <link name="${prefix}_foot_link">
      <visual>
        <geometry>
          <box size="0.15 0.08 0.05"/>
        </geometry>
        <material name="black"/>
      </visual>
      <collision>
        <geometry>
          <box size="0.15 0.08 0.05"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="0.3"/>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <inertia ixx="0.0003" ixy="0" ixz="0" iyy="0.0008" iyz="0" izz="0.0008"/>
      </inertial>
    </link>
    <xacro:joint name="${prefix}_ankle_joint" type="revolute"
                 parent="${prefix}_shin_link" child="${prefix}_foot_link"
                 xyz="0 0 -0.4" rpy="0 0 0" axis="0 1 0"
                 lower="-0.5" upper="0.5" effort="50" velocity="1"/>
  </xacro:macro>

  <!-- Right Leg Macro -->
  <xacro:macro name="right_leg" params="prefix">
    <!-- Thigh -->
    <xacro:cylinder_link name="${prefix}_thigh_link" radius="0.05" length="0.4"
                        mass="1.2" xyz="0 0 -0.2" rpy="0 0 0" material="black"/>
    <xacro:joint name="${prefix}_hip_joint" type="revolute"
                 parent="torso_link" child="${prefix}_thigh_link"
                 xyz="0 -0.05 0" rpy="0 0 0" axis="1 0 0"
                 lower="-1.0" upper="0.5" effort="100" velocity="2"/>

    <!-- Shin -->
    <xacro:cylinder_link name="${prefix}_shin_link" radius="0.04" length="0.4"
                        mass="1.0" xyz="0 0 -0.2" rpy="0 0 0" material="black"/>
    <xacro:joint name="${prefix}_knee_joint" type="revolute"
                 parent="${prefix}_thigh_link" child="${prefix}_shin_link"
                 xyz="0 0 -0.4" rpy="0 0 0" axis="1 0 0"
                 lower="0" upper="2.0" effort="100" velocity="2"/>

    <!-- Foot -->
    <link name="${prefix}_foot_link">
      <visual>
        <geometry>
          <box size="0.15 0.08 0.05"/>
        </geometry>
        <material name="black"/>
      </visual>
      <collision>
        <geometry>
          <box size="0.15 0.08 0.05"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="0.3"/>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <inertia ixx="0.0003" ixy="0" ixz="0" iyy="0.0008" iyz="0" izz="0.0008"/>
      </inertial>
    </link>
    <xacro:joint name="${prefix}_ankle_joint" type="revolute"
                 parent="${prefix}_shin_link" child="${prefix}_foot_link"
                 xyz="0 0 -0.4" rpy="0 0 0" axis="0 1 0"
                 lower="-0.5" upper="0.5" effort="50" velocity="1"/>
  </xacro:macro>

  <!-- Instantiate legs -->
  <xacro:left_leg prefix="l_leg"/>
  <xacro:right_leg prefix="r_leg"/>

</robot>
```

## Best Practices for Humanoid Design

### 1. Mass Distribution

Realistic mass distribution is crucial for stable simulation:

```xml
<!-- Good mass distribution example -->
<link name="torso_link">
  <inertial>
    <mass value="2.0"/>  <!-- Realistic for humanoid torso -->
    <origin xyz="0 0 0.15" rpy="0 0 0"/>  <!-- COM offset realistic -->
    <inertia ixx="0.04" ixy="0" ixz="0"    <!-- Moments physically plausible -->
             iyy="0.08" iyz="0" izz="0.06"/>
  </inertial>
</link>
```

### 2. Joint Limit Planning

Plan joint limits based on human anatomy or robot specifications:

```xml
<!-- Human-like shoulder limits -->
<joint name="left_shoulder_pitch_joint" type="revolute">
  <limit lower="-2.0" upper="1.5" effort="80" velocity="2"/>
  <!-- Forward movement limited, backward more restricted -->
</joint>

<!-- Human-like knee limits -->
<joint name="left_knee_joint" type="revolute">
  <limit lower="0" upper="2.5" effort="100" velocity="2"/>
  <!-- Can't bend knee backwards (0 lower limit) -->
</joint>
```

### 3. Sensor Integration Planning

Plan for sensor integration from the beginning:

```xml
<!-- Plan for cameras in head -->
<joint name="front_camera_joint" type="fixed">
  <parent link="head_link"/>
  <child link="front_camera_frame"/>
  <origin xyz="0.05 0 0.02" rpy="0 0 0"/>
</joint>

<!-- Plan for IMU in torso for balance -->
<joint name="imu_joint" type="fixed">
  <parent link="torso_link"/>
  <child link="imu_frame"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
</joint>
```

### 4. Collision Avoidance

Consider self-collision when setting joint limits:

```xml
<!-- Elbow limits to prevent collision with torso -->
<joint name="left_elbow_joint" type="revolute">
  <limit lower="0.1" upper="2.4" effort="40" velocity="2"/>
  <!-- Prevents arm from wrapping too far around body -->
</joint>
```

## Common Design Patterns

### 1. Symmetric Limbs

Use symmetry for arms and legs:

```xml
<!-- Left and right arms should be mirror images -->
<!-- Same joint types, similar ranges, mirrored positions -->
<joint name="left_shoulder_joint" type="revolute">
  <origin xyz="0.05 0.1 0.3" rpy="0 0 0"/>
  <!-- ... -->
</joint>

<joint name="right_shoulder_joint" type="revolute">
  <origin xyz="0.05 -0.1 0.3" rpy="0 0 0"/>
  <!-- ... -->
</joint>
```

### 2. Progressive Joint Complexity

Start simple and add complexity as needed:

```xml
<!-- Start with basic 6-DOF legs -->
<!-- Add more DOF as needed for specific behaviors -->
<!-- 6-DOF: hip yaw, hip roll, hip pitch, knee pitch, ankle pitch, ankle roll -->
```

### 3. Functional Grouping

Group joints by function:

```xml
<!-- Manipulation joints (arms) -->
<!-- Locomotion joints (legs) -->
<!-- Balance joints (torso, neck) -->
<!-- Each group can have different control strategies -->
```

## Validation and Testing

### 1. URDF Validation

```bash
# Validate the URDF file
check_urdf my_humanoid.urdf

# Visualize the kinematic tree
urdf_to_graphiz my_humanoid.urdf
dot -Tpng my_humanoid.gv -o my_humanoid.png
```

### 2. Simulation Testing

```bash
# Test in Gazebo
ros2 launch gazebo_ros spawn_entity.launch.py -entity humanoid -file my_humanoid.urdf

# Check for stability, collisions, and proper kinematics
```

### 3. TF Tree Analysis

```bash
# Check the TF tree
ros2 run tf2_tools view_frames

# Verify all expected frames exist
ros2 run tf2_ros tf2_echo base_link left_hand_link
```

## Troubleshooting Common Issues

### 1. Unstable Simulation

If the robot is unstable in simulation:

```xml
<!-- Check that all masses are positive and realistic -->
<!-- Verify inertial tensors are physically valid -->
<!-- Ensure joint limits are reasonable -->
<!-- Check that origins are properly aligned -->
```

### 2. Kinematic Issues

If forward/inverse kinematics don't work properly:

```xml
<!-- Verify joint axes are correctly oriented -->
<!-- Check that joint origins are at physical joint centers -->
<!-- Ensure the kinematic chain is properly connected -->
```

### 3. Collision Problems

If there are unexpected collisions:

```xml
<!-- Check collision geometries are appropriate -->
<!-- Verify joint limits prevent self-collision -->
<!-- Consider adding safety margins to collision shapes -->
```

## Next Steps

In the following sections, we'll add XML code examples, visual aids, exercises, and validation rules to complete our understanding of URDF for humanoid modeling. We'll also explore how to validate URDF models and best practices for maintaining complex humanoid robot descriptions.