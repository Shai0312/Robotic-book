---
sidebar_position: 7
---

# URDF XML Code Examples

## Complete Humanoid Robot URDF

This page provides comprehensive XML code examples for creating humanoid robot models in URDF. These examples demonstrate best practices and common patterns used in humanoid robotics.

### Basic Humanoid Robot with All Components

```xml
<?xml version="1.0"?>
<robot name="complete_humanoid_robot">

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
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <box size="0.25 0.15 0.3"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <box size="0.25 0.15 0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <inertia ixx="0.03" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.04"/>
    </inertial>
  </link>

  <!-- Head -->
  <joint name="head_joint" type="revolute">
    <parent link="torso_link"/>
    <child link="head_link"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
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

  <!-- Camera in head -->
  <joint name="camera_joint" type="fixed">
    <parent link="head_link"/>
    <child link="camera_frame"/>
    <origin xyz="0.05 0 0.02" rpy="0 0 0"/>
  </joint>

  <link name="camera_frame">
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

  <!-- Left Arm -->
  <joint name="l_shoulder_pitch_joint" type="revolute">
    <parent link="torso_link"/>
    <child link="l_shoulder_link"/>
    <origin xyz="0.05 0.1 0.25" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="2"/>
  </joint>

  <link name="l_shoulder_link">
    <visual>
      <origin xyz="0 0 -0.05" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.1"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.05" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.4"/>
      <origin xyz="0 0 -0.05" rpy="0 0 0"/>
      <inertia ixx="0.0005" ixy="0" ixz="0" iyy="0.0005" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="l_shoulder_roll_joint" type="revolute">
    <parent link="l_shoulder_link"/>
    <child link="l_upper_arm_link"/>
    <origin xyz="0 0 -0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.0" effort="50" velocity="2"/>
  </joint>

  <link name="l_upper_arm_link">
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.035" length="0.3"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.035" length="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <inertia ixx="0.002" ixy="0" ixz="0" iyy="0.002" iyz="0" izz="0.0003"/>
    </inertial>
  </link>

  <joint name="l_elbow_joint" type="revolute">
    <parent link="l_upper_arm_link"/>
    <child link="l_lower_arm_link"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.5" effort="40" velocity="2"/>
  </joint>

  <link name="l_lower_arm_link">
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
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.0002"/>
    </inertial>
  </link>

  <!-- Left Hand -->
  <joint name="l_wrist_joint" type="revolute">
    <parent link="l_lower_arm_link"/>
    <child link="l_hand_link"/>
    <origin xyz="0 0 -0.25" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="20" velocity="3"/>
  </joint>

  <link name="l_hand_link">
    <visual>
      <geometry>
        <box size="0.1 0.06 0.04"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.06 0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0002" iyz="0" izz="0.0002"/>
    </inertial>
  </link>

  <!-- Right Arm (symmetric to left) -->
  <joint name="r_shoulder_pitch_joint" type="revolute">
    <parent link="torso_link"/>
    <child link="r_shoulder_link"/>
    <origin xyz="0.05 -0.1 0.25" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="2"/>
  </joint>

  <link name="r_shoulder_link">
    <visual>
      <origin xyz="0 0 -0.05" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.1"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.05" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.4"/>
      <origin xyz="0 0 -0.05" rpy="0 0 0"/>
      <inertia ixx="0.0005" ixy="0" ixz="0" iyy="0.0005" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="r_shoulder_roll_joint" type="revolute">
    <parent link="r_shoulder_link"/>
    <child link="r_upper_arm_link"/>
    <origin xyz="0 0 -0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.0" effort="50" velocity="2"/>
  </joint>

  <link name="r_upper_arm_link">
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.035" length="0.3"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.035" length="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <inertia ixx="0.002" ixy="0" ixz="0" iyy="0.002" iyz="0" izz="0.0003"/>
    </inertial>
  </link>

  <joint name="r_elbow_joint" type="revolute">
    <parent link="r_upper_arm_link"/>
    <child link="r_lower_arm_link"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.5" effort="40" velocity="2"/>
  </joint>

  <link name="r_lower_arm_link">
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
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.0002"/>
    </inertial>
  </link>

  <!-- Right Hand -->
  <joint name="r_wrist_joint" type="revolute">
    <parent link="r_lower_arm_link"/>
    <child link="r_hand_link"/>
    <origin xyz="0 0 -0.25" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="20" velocity="3"/>
  </joint>

  <link name="r_hand_link">
    <visual>
      <geometry>
        <box size="0.1 0.06 0.04"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.06 0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0002" iyz="0" izz="0.0002"/>
    </inertial>
  </link>

  <!-- Left Leg -->
  <joint name="l_hip_yaw_joint" type="revolute">
    <parent link="torso_link"/>
    <child link="l_hip_link"/>
    <origin xyz="0 0.05 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.5" upper="0.5" effort="100" velocity="1"/>
  </joint>

  <link name="l_hip_link">
    <visual>
      <origin xyz="0 0 -0.05" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.1"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.05" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.6"/>
      <origin xyz="0 0 -0.05" rpy="0 0 0"/>
      <inertia ixx="0.0008" ixy="0" ixz="0" iyy="0.0008" iyz="0" izz="0.0004"/>
    </inertial>
  </link>

  <joint name="l_hip_roll_joint" type="revolute">
    <parent link="l_hip_link"/>
    <child link="l_thigh_link"/>
    <origin xyz="0 0 -0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="100" velocity="1"/>
  </joint>

  <link name="l_thigh_link">
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

  <joint name="l_knee_joint" type="revolute">
    <parent link="l_thigh_link"/>
    <child link="l_shin_link"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="0" upper="2.0" effort="100" velocity="1"/>
  </joint>

  <link name="l_shin_link">
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.045" length="0.3"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.045" length="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <inertia ixx="0.011" ixy="0" ixz="0" iyy="0.011" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Left Foot -->
  <joint name="l_ankle_joint" type="revolute">
    <parent link="l_shin_link"/>
    <child link="l_foot_link"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-0.5" upper="0.5" effort="50" velocity="1"/>
  </joint>

  <link name="l_foot_link">
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
  <joint name="r_hip_yaw_joint" type="revolute">
    <parent link="torso_link"/>
    <child link="r_hip_link"/>
    <origin xyz="0 -0.05 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.5" upper="0.5" effort="100" velocity="1"/>
  </joint>

  <link name="r_hip_link">
    <visual>
      <origin xyz="0 0 -0.05" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.1"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.05" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.6"/>
      <origin xyz="0 0 -0.05" rpy="0 0 0"/>
      <inertia ixx="0.0008" ixy="0" ixz="0" iyy="0.0008" iyz="0" izz="0.0004"/>
    </inertial>
  </link>

  <joint name="r_hip_roll_joint" type="revolute">
    <parent link="r_hip_link"/>
    <child link="r_thigh_link"/>
    <origin xyz="0 0 -0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="100" velocity="1"/>
  </joint>

  <link name="r_thigh_link">
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

  <joint name="r_knee_joint" type="revolute">
    <parent link="r_thigh_link"/>
    <child link="r_shin_link"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="0" upper="2.0" effort="100" velocity="1"/>
  </joint>

  <link name="r_shin_link">
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.045" length="0.3"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.045" length="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <inertia ixx="0.011" ixy="0" ixz="0" iyy="0.011" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Right Foot -->
  <joint name="r_ankle_joint" type="revolute">
    <parent link="r_shin_link"/>
    <child link="r_foot_link"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-0.5" upper="0.5" effort="50" velocity="1"/>
  </joint>

  <link name="r_foot_link">
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

## URDF with Gazebo Integration

```xml
<?xml version="1.0"?>
<robot name="humanoid_with_gazebo" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Include Gazebo-specific plugins -->
  <gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
      <robotNamespace>/humanoid</robotNamespace>
    </plugin>
  </gazebo>

  <!-- Base link -->
  <link name="base_link">
    <inertial>
      <mass value="0.001"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- Torso with Gazebo properties -->
  <joint name="torso_joint" type="fixed">
    <parent link="base_link"/>
    <child link="torso_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>

  <link name="torso_link">
    <visual>
      <geometry>
        <box size="0.25 0.15 0.3"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.25 0.15 0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <inertia ixx="0.03" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.04"/>
    </inertial>
  </link>

  <!-- Gazebo-specific properties for torso -->
  <gazebo reference="torso_link">
    <material>Gazebo/White</material>
    <mu1>0.9</mu1>
    <mu2>0.9</mu2>
    <kp>1000000.0</kp>
    <kd>100.0</kd>
  </gazebo>

  <!-- Head with camera sensor -->
  <joint name="head_joint" type="revolute">
    <parent link="torso_link"/>
    <child link="head_link"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
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

  <!-- Camera sensor in head -->
  <joint name="camera_joint" type="fixed">
    <parent link="head_link"/>
    <child link="camera_link"/>
    <origin xyz="0.06 0 0.02" rpy="0 0 0"/>
  </joint>

  <link name="camera_link"/>

  <gazebo reference="camera_link">
    <sensor name="camera" type="camera">
      <always_on>true</always_on>
      <update_rate>30.0</update_rate>
      <camera name="head">
        <horizontal_fov>1.3962634</horizontal_fov>
        <image>
          <width>800</width>
          <height>600</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>100</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <camera_name>humanoid/camera1</camera_name>
        <image_topic_name>image_raw</image_topic_name>
        <camera_info_topic_name>camera_info</camera_info_topic_name>
        <frame_name>camera_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>

  <!-- IMU sensor in torso -->
  <joint name="imu_joint" type="fixed">
    <parent link="torso_link"/>
    <child link="imu_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>

  <link name="imu_link"/>

  <gazebo reference="imu_link">
    <sensor name="imu_sensor" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <visualize>true</visualize>
      <imu>
        <angular_velocity>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2e-4</stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2e-4</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2e-4</stddev>
            </noise>
          </z>
        </angular_velocity>
        <linear_acceleration>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
            </noise>
          </z>
        </linear_acceleration>
      </imu>
    </sensor>
  </gazebo>

  <!-- Left Arm -->
  <joint name="l_shoulder_pitch_joint" type="revolute">
    <parent link="torso_link"/>
    <child link="l_shoulder_link"/>
    <origin xyz="0.05 0.1 0.25" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="2"/>
    <dynamics damping="0.5" friction="0.1"/>
  </joint>

  <link name="l_shoulder_link">
    <visual>
      <origin xyz="0 0 -0.05" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.1"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.05" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.4"/>
      <origin xyz="0 0 -0.05" rpy="0 0 0"/>
      <inertia ixx="0.0005" ixy="0" ixz="0" iyy="0.0005" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- Gazebo properties for left shoulder -->
  <gazebo reference="l_shoulder_link">
    <material>Gazebo/Grey</material>
  </gazebo>

  <joint name="l_shoulder_roll_joint" type="revolute">
    <parent link="l_shoulder_link"/>
    <child link="l_upper_arm_link"/>
    <origin xyz="0 0 -0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.0" effort="50" velocity="2"/>
    <dynamics damping="0.5" friction="0.1"/>
  </joint>

  <link name="l_upper_arm_link">
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.035" length="0.3"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.035" length="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <inertia ixx="0.002" ixy="0" ixz="0" iyy="0.002" iyz="0" izz="0.0003"/>
    </inertial>
  </link>

  <!-- More joints and links would continue in the same pattern... -->

</robot>
```

## Xacro Macros for Modular Design

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="xacro_humanoid">

  <!-- Define constants -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="ARM_UPPER_LENGTH" value="0.3" />
  <xacro:property name="ARM_LOWER_LENGTH" value="0.25" />
  <xacro:property name="LEG_THIGH_LENGTH" value="0.4" />
  <xacro:property name="LEG_SHIN_LENGTH" value="0.3" />

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

  <!-- Macro for a simple cylinder link with proper inertial properties -->
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

  <!-- Macro for a box link with proper inertial properties -->
  <xacro:macro name="box_link" params="name size_x size_y size_z mass xyz rpy material">
    <link name="${name}">
      <visual>
        <origin xyz="${xyz}" rpy="${rpy}"/>
        <geometry>
          <box size="${size_x} ${size_y} ${size_z}"/>
        </geometry>
        <material name="${material}"/>
      </visual>
      <collision>
        <origin xyz="${xyz}" rpy="${rpy}"/>
        <geometry>
          <box size="${size_x} ${size_y} ${size_z}"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="${mass}"/>
        <origin xyz="${xyz}" rpy="${rpy}"/>
        <inertia
          ixx="${0.0833333 * mass * (size_y*size_y + size_z*size_z)}"
          ixy="0" ixz="0"
          iyy="${0.0833333 * mass * (size_x*size_x + size_z*size_z)}"
          iyz="0"
          izz="${0.0833333 * mass * (size_x*size_x + size_y*size_y)}"/>
      </inertial>
    </link>
  </xacro:macro>

  <!-- Macro for a spherical link with proper inertial properties -->
  <xacro:macro name="sphere_link" params="name radius mass xyz rpy material">
    <link name="${name}">
      <visual>
        <origin xyz="${xyz}" rpy="${rpy}"/>
        <geometry>
          <sphere radius="${radius}"/>
        </geometry>
        <material name="${material}"/>
      </visual>
      <collision>
        <origin xyz="${xyz}" rpy="${rpy}"/>
        <geometry>
          <sphere radius="${radius}"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="${mass}"/>
        <origin xyz="${xyz}" rpy="${rpy}"/>
        <inertia
          ixx="${0.4 * mass * radius * radius}"
          ixy="0" ixz="0"
          iyy="${0.4 * mass * radius * radius}"
          iyz="0"
          izz="${0.4 * mass * radius * radius}"/>
      </inertial>
    </link>
  </xacro:macro>

  <!-- Macro for a joint -->
  <xacro:macro name="joint" params="name type parent child xyz rpy axis lower upper effort velocity">
    <joint name="${name}" type="${type}">
      <parent link="${parent}"/>
      <child link="${child}"/>
      <origin xyz="${xyz}" rpy="${rpy}"/>
      <axis xyz="${axis}"/>
      <xacro:if value="${type != 'fixed'}">
        <limit lower="${lower}" upper="${upper}" effort="${effort}" velocity="${velocity}"/>
        <dynamics damping="0.5" friction="0.1"/>
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
  <xacro:box_link name="torso_link" size_x="0.25" size_y="0.15" size_z="0.3"
                  mass="2.0" xyz="0 0 0.15" rpy="0 0 0" material="white"/>
  <xacro:joint name="torso_joint" type="fixed" parent="base_link" child="torso_link"
               xyz="0 0 0.1" rpy="0 0 0" axis="1 0 0"
               lower="0" upper="0" effort="0" velocity="0"/>

  <!-- Head -->
  <xacro:sphere_link name="head_link" radius="0.08" mass="0.5"
                     xyz="0 0 0" rpy="0 0 0" material="white"/>
  <xacro:joint name="head_joint" type="revolute" parent="torso_link" child="head_link"
               xyz="0 0 0.3" rpy="0 0 0" axis="0 1 0"
               lower="-0.5" upper="0.5" effort="10" velocity="1"/>

  <!-- Left Arm Macro -->
  <xacro:macro name="left_arm" params="prefix shoulder_pos_x shoulder_pos_y shoulder_pos_z">
    <!-- Shoulder -->
    <xacro:cylinder_link name="${prefix}_shoulder_link" radius="0.03" length="0.1"
                        mass="0.4" xyz="0 0 -0.05" rpy="0 0 0" material="gray"/>
    <xacro:joint name="${prefix}_shoulder_pitch_joint" type="revolute"
                 parent="torso_link" child="${prefix}_shoulder_link"
                 xyz="${shoulder_pos_x} ${shoulder_pos_y} ${shoulder_pos_z}" rpy="0 0 0" axis="1 0 0"
                 lower="-1.57" upper="1.57" effort="50" velocity="2"/>

    <!-- Upper Arm -->
    <xacro:cylinder_link name="${prefix}_upper_arm_link" radius="0.035" length="${ARM_UPPER_LENGTH}"
                        mass="0.8" xyz="0 0 -${ARM_UPPER_LENGTH/2}" rpy="0 0 0" material="gray"/>
    <xacro:joint name="${prefix}_shoulder_roll_joint" type="revolute"
                 parent="${prefix}_shoulder_link" child="${prefix}_upper_arm_link"
                 xyz="0 0 -0.1" rpy="0 0 0" axis="0 1 0"
                 lower="0" upper="2.0" effort="50" velocity="2"/>

    <!-- Elbow -->
    <xacro:cylinder_link name="${prefix}_lower_arm_link" radius="0.03" length="${ARM_LOWER_LENGTH}"
                        mass="0.6" xyz="0 0 -${ARM_LOWER_LENGTH/2}" rpy="0 0 0" material="gray"/>
    <xacro:joint name="${prefix}_elbow_joint" type="revolute"
                 parent="${prefix}_upper_arm_link" child="${prefix}_lower_arm_link"
                 xyz="0 0 -${ARM_UPPER_LENGTH}" rpy="0 0 0" axis="0 1 0"
                 lower="0" upper="2.5" effort="40" velocity="2"/>

    <!-- Wrist -->
    <xacro:box_link name="${prefix}_hand_link" size_x="0.1" size_y="0.06" size_z="0.04"
                   mass="0.2" xyz="0 0 0" rpy="0 0 0" material="gray"/>
    <xacro:joint name="${prefix}_wrist_joint" type="revolute"
                 parent="${prefix}_lower_arm_link" child="${prefix}_hand_link"
                 xyz="0 0 -${ARM_LOWER_LENGTH}" rpy="0 0 0" axis="0 0 1"
                 lower="-1.57" upper="1.57" effort="20" velocity="3"/>
  </xacro:macro>

  <!-- Right Arm Macro -->
  <xacro:macro name="right_arm" params="prefix shoulder_pos_x shoulder_pos_y shoulder_pos_z">
    <!-- Shoulder -->
    <xacro:cylinder_link name="${prefix}_shoulder_link" radius="0.03" length="0.1"
                        mass="0.4" xyz="0 0 -0.05" rpy="0 0 0" material="gray"/>
    <xacro:joint name="${prefix}_shoulder_pitch_joint" type="revolute"
                 parent="torso_link" child="${prefix}_shoulder_link"
                 xyz="${shoulder_pos_x} ${shoulder_pos_y} ${shoulder_pos_z}" rpy="0 0 0" axis="1 0 0"
                 lower="-1.57" upper="1.57" effort="50" velocity="2"/>

    <!-- Upper Arm -->
    <xacro:cylinder_link name="${prefix}_upper_arm_link" radius="0.035" length="${ARM_UPPER_LENGTH}"
                        mass="0.8" xyz="0 0 -${ARM_UPPER_LENGTH/2}" rpy="0 0 0" material="gray"/>
    <xacro:joint name="${prefix}_shoulder_roll_joint" type="revolute"
                 parent="${prefix}_shoulder_link" child="${prefix}_upper_arm_link"
                 xyz="0 0 -0.1" rpy="0 0 0" axis="0 1 0"
                 lower="0" upper="2.0" effort="50" velocity="2"/>

    <!-- Elbow -->
    <xacro:cylinder_link name="${prefix}_lower_arm_link" radius="0.03" length="${ARM_LOWER_LENGTH}"
                        mass="0.6" xyz="0 0 -${ARM_LOWER_LENGTH/2}" rpy="0 0 0" material="gray"/>
    <xacro:joint name="${prefix}_elbow_joint" type="revolute"
                 parent="${prefix}_upper_arm_link" child="${prefix}_lower_arm_link"
                 xyz="0 0 -${ARM_UPPER_LENGTH}" rpy="0 0 0" axis="0 1 0"
                 lower="0" upper="2.5" effort="40" velocity="2"/>

    <!-- Wrist -->
    <xacro:box_link name="${prefix}_hand_link" size_x="0.1" size_y="0.06" size_z="0.04"
                   mass="0.2" xyz="0 0 0" rpy="0 0 0" material="gray"/>
    <xacro:joint name="${prefix}_wrist_joint" type="revolute"
                 parent="${prefix}_lower_arm_link" child="${prefix}_hand_link"
                 xyz="0 0 -${ARM_LOWER_LENGTH}" rpy="0 0 0" axis="0 0 1"
                 lower="-1.57" upper="1.57" effort="20" velocity="3"/>
  </xacro:macro>

  <!-- Instantiate arms -->
  <xacro:left_arm prefix="l_arm" shoulder_pos_x="0.05" shoulder_pos_y="0.1" shoulder_pos_z="0.25"/>
  <xacro:right_arm prefix="r_arm" shoulder_pos_x="0.05" shoulder_pos_y="-0.1" shoulder_pos_z="0.25"/>

  <!-- Left Leg Macro -->
  <xacro:macro name="left_leg" params="prefix hip_pos_x hip_pos_y hip_pos_z">
    <!-- Hip -->
    <xacro:cylinder_link name="${prefix}_hip_link" radius="0.04" length="0.1"
                        mass="0.6" xyz="0 0 -0.05" rpy="0 0 0" material="black"/>
    <xacro:joint name="${prefix}_hip_yaw_joint" type="revolute"
                 parent="torso_link" child="${prefix}_hip_link"
                 xyz="${hip_pos_x} ${hip_pos_y} ${hip_pos_z}" rpy="0 0 0" axis="0 0 1"
                 lower="-0.5" upper="0.5" effort="100" velocity="1"/>

    <xacro:joint name="${prefix}_hip_roll_joint" type="revolute"
                 parent="${prefix}_hip_link" child="${prefix}_thigh_link"
                 xyz="0 0 -0.1" rpy="0 0 0" axis="0 1 0"
                 lower="-0.5" upper="0.5" effort="100" velocity="1"/>

    <!-- Thigh -->
    <xacro:cylinder_link name="${prefix}_thigh_link" radius="0.05" length="${LEG_THIGH_LENGTH}"
                        mass="1.2" xyz="0 0 -${LEG_THIGH_LENGTH/2}" rpy="0 0 0" material="black"/>
    <xacro:joint name="${prefix}_hip_pitch_joint" type="revolute"
                 parent="${prefix}_hip_link" child="${prefix}_thigh_link"
                 xyz="0 0 -0.1" rpy="0 0 0" axis="1 0 0"
                 lower="-1.0" upper="0.5" effort="100" velocity="1"/>

    <!-- Shin -->
    <xacro:cylinder_link name="${prefix}_shin_link" radius="0.045" length="${LEG_SHIN_LENGTH}"
                        mass="1.0" xyz="0 0 -${LEG_SHIN_LENGTH/2}" rpy="0 0 0" material="black"/>
    <xacro:joint name="${prefix}_knee_joint" type="revolute"
                 parent="${prefix}_thigh_link" child="${prefix}_shin_link"
                 xyz="0 0 -${LEG_THIGH_LENGTH}" rpy="0 0 0" axis="1 0 0"
                 lower="0" upper="2.0" effort="100" velocity="1"/>

    <!-- Ankle and Foot -->
    <xacro:box_link name="${prefix}_foot_link" size_x="0.15" size_y="0.08" size_z="0.05"
                   mass="0.3" xyz="0 0 0" rpy="0 0 0" material="black"/>
    <xacro:joint name="${prefix}_ankle_joint" type="revolute"
                 parent="${prefix}_shin_link" child="${prefix}_foot_link"
                 xyz="0 0 -${LEG_SHIN_LENGTH}" rpy="0 0 0" axis="1 0 0"
                 lower="-0.5" upper="0.5" effort="50" velocity="1"/>
  </xacro:macro>

  <!-- Right Leg Macro -->
  <xacro:macro name="right_leg" params="prefix hip_pos_x hip_pos_y hip_pos_z">
    <!-- Hip -->
    <xacro:cylinder_link name="${prefix}_hip_link" radius="0.04" length="0.1"
                        mass="0.6" xyz="0 0 -0.05" rpy="0 0 0" material="black"/>
    <xacro:joint name="${prefix}_hip_yaw_joint" type="revolute"
                 parent="torso_link" child="${prefix}_hip_link"
                 xyz="${hip_pos_x} ${hip_pos_y} ${hip_pos_z}" rpy="0 0 0" axis="0 0 1"
                 lower="-0.5" upper="0.5" effort="100" velocity="1"/>

    <xacro:joint name="${prefix}_hip_roll_joint" type="revolute"
                 parent="${prefix}_hip_link" child="${prefix}_thigh_link"
                 xyz="0 0 -0.1" rpy="0 0 0" axis="0 1 0"
                 lower="-0.5" upper="0.5" effort="100" velocity="1"/>

    <!-- Thigh -->
    <xacro:cylinder_link name="${prefix}_thigh_link" radius="0.05" length="${LEG_THIGH_LENGTH}"
                        mass="1.2" xyz="0 0 -${LEG_THIGH_LENGTH/2}" rpy="0 0 0" material="black"/>
    <xacro:joint name="${prefix}_hip_pitch_joint" type="revolute"
                 parent="${prefix}_hip_link" child="${prefix}_thigh_link"
                 xyz="0 0 -0.1" rpy="0 0 0" axis="1 0 0"
                 lower="-1.0" upper="0.5" effort="100" velocity="1"/>

    <!-- Shin -->
    <xacro:cylinder_link name="${prefix}_shin_link" radius="0.045" length="${LEG_SHIN_LENGTH}"
                        mass="1.0" xyz="0 0 -${LEG_SHIN_LENGTH/2}" rpy="0 0 0" material="black"/>
    <xacro:joint name="${prefix}_knee_joint" type="revolute"
                 parent="${prefix}_thigh_link" child="${prefix}_shin_link"
                 xyz="0 0 -${LEG_THIGH_LENGTH}" rpy="0 0 0" axis="1 0 0"
                 lower="0" upper="2.0" effort="100" velocity="1"/>

    <!-- Ankle and Foot -->
    <xacro:box_link name="${prefix}_foot_link" size_x="0.15" size_y="0.08" size_z="0.05"
                   mass="0.3" xyz="0 0 0" rpy="0 0 0" material="black"/>
    <xacro:joint name="${prefix}_ankle_joint" type="revolute"
                 parent="${prefix}_shin_link" child="${prefix}_foot_link"
                 xyz="0 0 -${LEG_SHIN_LENGTH}" rpy="0 0 0" axis="1 0 0"
                 lower="-0.5" upper="0.5" effort="50" velocity="1"/>
  </xacro:macro>

  <!-- Instantiate legs -->
  <xacro:left_leg prefix="l_leg" hip_pos_x="0" hip_pos_y="0.05" hip_pos_z="0"/>
  <xacro:right_leg prefix="r_leg" hip_pos_x="0" hip_pos_y="-0.05" hip_pos_z="0"/>

</xacro:robot>
```

## URDF with ros2_control Integration

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_with_control">

  <!-- ros2_control plugin -->
  <ros2_control name="GazeboSystem" type="system">
    <hardware>
      <plugin>gazebo_ros2_control/GazeboSystem</plugin>
    </hardware>

    <joint name="l_shoulder_pitch_joint">
      <command_interface name="position">
        <param name="min">-1.57</param>
        <param name="max">1.57</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>

    <joint name="l_shoulder_roll_joint">
      <command_interface name="position">
        <param name="min">0</param>
        <param name="max">2.0</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>

    <joint name="l_elbow_joint">
      <command_interface name="position">
        <param name="min">0</param>
        <param name="max">2.5</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>

    <!-- More joints would be defined similarly... -->
  </ros2_control>

  <!-- Gazebo plugin for ros2_control -->
  <gazebo>
    <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
      <parameters>$(find my_robot_description)/config/robot_controllers.yaml</parameters>
      <robot_sim_type>gazebo_ros2_control/GazeboSystem</robot_sim_type>
    </plugin>
  </gazebo>

  <!-- Robot links and joints would continue as in previous examples... -->

</robot>
```

## Common URDF Patterns and Snippets

### 1. Standard Link Definition
```xml
<link name="example_link">
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.1 0.1 0.1"/>
    </geometry>
    <material name="red">
      <color rgba="1 0 0 1"/>
    </material>
  </visual>
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.1 0.1 0.1"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="1.0"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
  </inertial>
</link>
```

### 2. Standard Joint Definition
```xml
<joint name="example_joint" type="revolute">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  <dynamics damping="0.5" friction="0.1"/>
</joint>
```

### 3. Material Definition
```xml
<material name="example_material">
  <color rgba="0.5 0.5 0.5 1"/>
</material>
```

### 4. Gazebo Sensor Integration
```xml
<gazebo reference="sensor_link">
  <sensor name="example_sensor" type="camera">
    <always_on>true</always_on>
    <update_rate>30.0</update_rate>
    <camera name="example">
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
    </camera>
  </sensor>
</gazebo>
```

### 5. Xacro Property Definition
```xml
<xacro:property name="link_length" value="0.1" />
<xacro:property name="link_radius" value="0.05" />
<xacro:property name="link_mass" value="0.5" />
```

These examples demonstrate various aspects of URDF creation for humanoid robots, from basic structure to advanced integration with simulation and control systems. Use these as templates for your own robot designs, adapting the dimensions, masses, and joint limits to match your specific robot requirements.