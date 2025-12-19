---
sidebar_position: 9
---

# Exercises for Creating URDF Files

## Overview

This page contains hands-on exercises to help you practice creating URDF files for humanoid robots. These exercises range from basic to advanced, allowing you to build your skills progressively.

## Exercise 1: Simple Link Creation

**Objective**: Create a single link with visual, collision, and inertial properties.

**Instructions**:
1. Create a URDF file with a single link named "simple_link"
2. Give it a box geometry (0.1 x 0.1 x 0.1 meters)
3. Set the mass to 1.0 kg
4. Calculate and specify the appropriate inertia tensor for a box
5. Add a gray material to the visual element

**Solution**:
```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <link name="simple_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.001666" ixy="0" ixz="0"
               iyy="0.001666" iyz="0"
               izz="0.001666"/>
    </inertial>
  </link>
</robot>
```

**Inertia Calculation**: For a box with mass m and dimensions x, y, z:
- ixx = m*(y² + z²)/12
- iyy = m*(x² + z²)/12
- izz = m*(x² + y²)/12

## Exercise 2: Basic Joint Connection

**Objective**: Create two links connected by a joint.

**Instructions**:
1. Create a "base_link" and an "arm_link"
2. Connect them with a revolute joint
3. Set the joint to rotate around the Z-axis
4. Give the joint limits of -90° to 90° (-1.57 to 1.57 radians)
5. Set effort limit to 10 Nm and velocity limit to 1 rad/s

**Solution**:
```xml
<?xml version="1.0"?>
<robot name="simple_arm">
  <link name="base_link">
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
    </collision>
  </link>

  <joint name="arm_joint" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <link name="arm_link">
    <inertial>
      <mass value="0.3"/>
      <origin xyz="0 0 0.15"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.0005"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.15"/>
      <geometry>
        <cylinder radius="0.02" length="0.3"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0.15"/>
      <geometry>
        <cylinder radius="0.02" length="0.3"/>
      </geometry>
    </collision>
  </link>
</robot>
```

## Exercise 3: Robot with Multiple Links

**Objective**: Create a simple humanoid torso with head and arms.

**Instructions**:
1. Create a torso link (box: 0.3 x 0.2 x 0.4 m, mass: 2.0 kg)
2. Add a head link (sphere: radius 0.08 m, mass: 0.5 kg) on top of torso
3. Add left and right upper arms (cylinders: radius 0.03, length 0.25, mass: 0.8 kg each)
4. Connect all parts with appropriate joints
5. Use proper inertial properties for each link

**Solution**:
```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Torso -->
  <link name="torso_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 0.2"/>
      <inertia ixx="0.04" ixy="0" ixz="0" iyy="0.06" iyz="0" izz="0.05"/>
    </inertial>
  </link>

  <!-- Head -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso_link"/>
    <child link="head_link"/>
    <origin xyz="0 0 0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="5" velocity="1"/>
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
      <origin xyz="0 0 0"/>
      <inertia ixx="0.0011" ixy="0" ixz="0" iyy="0.0011" iyz="0" izz="0.0011"/>
    </inertial>
  </link>

  <!-- Left Shoulder -->
  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso_link"/>
    <child link="left_upper_arm_link"/>
    <origin xyz="0.1 0.1 0.2" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="20" velocity="2"/>
  </joint>

  <link name="left_upper_arm_link">
    <visual>
      <origin xyz="0 0 -0.125"/>
      <geometry>
        <cylinder radius="0.03" length="0.25"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.125"/>
      <geometry>
        <cylinder radius="0.03" length="0.25"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <origin xyz="0 0 -0.125"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.0005"/>
    </inertial>
  </link>

  <!-- Right Shoulder -->
  <joint name="right_shoulder_joint" type="revolute">
    <parent link="torso_link"/>
    <child link="right_upper_arm_link"/>
    <origin xyz="0.1 -0.1 0.2" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="20" velocity="2"/>
  </joint>

  <link name="right_upper_arm_link">
    <visual>
      <origin xyz="0 0 -0.125"/>
      <geometry>
        <cylinder radius="0.03" length="0.25"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.125"/>
      <geometry>
        <cylinder radius="0.03" length="0.25"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <origin xyz="0 0 -0.125"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.0005"/>
    </inertial>
  </link>
</robot>
```

## Exercise 4: Using Xacro for Modularity

**Objective**: Create a parameterized arm using Xacro macros.

**Instructions**:
1. Define Xacro properties for arm dimensions and mass
2. Create a macro for an arm segment that takes parameters for name, length, radius, and mass
3. Use the macro to create a 2-segment arm (upper and lower arm)
4. Connect the segments with a revolute joint

**Solution**:
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="xacro_arm">

  <!-- Define arm properties -->
  <xacro:property name="M_PI" value="3.14159265359"/>
  <xacro:property name="arm_upper_length" value="0.3"/>
  <xacro:property name="arm_lower_length" value="0.25"/>
  <xacro:property name="arm_radius" value="0.03"/>
  <xacro:property name="arm_upper_mass" value="0.8"/>
  <xacro:property name="arm_lower_mass" value="0.6"/>

  <!-- Material -->
  <material name="gray">
    <color rgba="0.5 0.5 0.5 1"/>
  </material>

  <!-- Macro for arm segment -->
  <xacro:macro name="arm_segment" params="name length radius mass xyz rpy">
    <link name="${name}">
      <visual>
        <origin xyz="${xyz}" rpy="${rpy}"/>
        <geometry>
          <cylinder radius="${radius}" length="${length}"/>
        </geometry>
        <material name="gray"/>
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

  <!-- Base link -->
  <link name="base_link">
    <inertial>
      <mass value="0.001"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- Joint to attach arm to base -->
  <joint name="arm_base_joint" type="fixed">
    <parent link="base_link"/>
    <child link="upper_arm_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>

  <!-- Create upper arm using macro -->
  <xacro:arm_segment name="upper_arm_link"
                     length="${arm_upper_length}"
                     radius="${arm_radius}"
                     mass="${arm_upper_mass}"
                     xyz="0 0 -${arm_upper_length/2}"
                     rpy="0 0 0"/>

  <!-- Elbow joint -->
  <joint name="elbow_joint" type="revolute">
    <parent link="upper_arm_link"/>
    <child link="lower_arm_link"/>
    <origin xyz="0 0 -${arm_upper_length}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.5" effort="40" velocity="2"/>
  </joint>

  <!-- Create lower arm using macro -->
  <xacro:arm_segment name="lower_arm_link"
                     length="${arm_lower_length}"
                     radius="${arm_radius}"
                     mass="${arm_lower_mass}"
                     xyz="0 0 -${arm_lower_length/2}"
                     rpy="0 0 0"/>

</robot>
```

## Exercise 5: Adding Sensors to URDF

**Objective**: Add a camera and IMU to your robot model.

**Instructions**:
1. Take your robot from Exercise 3 or 4
2. Add a camera to the head with appropriate field of view
3. Add an IMU to the torso
4. Include Gazebo plugins for the sensors

**Solution**:
```xml
<?xml version="1.0"?>
<robot name="robot_with_sensors">

  <!-- Base link -->
  <link name="base_link">
    <inertial>
      <mass value="0.001"/>
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
        <box size="0.3 0.2 0.4"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 0.2"/>
      <inertia ixx="0.04" ixy="0" ixz="0" iyy="0.06" iyz="0" izz="0.05"/>
    </inertial>
  </link>

  <!-- IMU in torso -->
  <joint name="imu_joint" type="fixed">
    <parent link="torso_link"/>
    <child link="imu_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>

  <link name="imu_link"/>

  <!-- Head -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso_link"/>
    <child link="head_link"/>
    <origin xyz="0 0 0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="5" velocity="1"/>
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
      <origin xyz="0 0 0"/>
      <inertia ixx="0.0011" ixy="0" ixz="0" iyy="0.0011" iyz="0" izz="0.0011"/>
    </inertial>
  </link>

  <!-- Camera in head -->
  <joint name="camera_joint" type="fixed">
    <parent link="head_link"/>
    <child link="camera_link"/>
    <origin xyz="0.06 0 0.02" rpy="0 0 0"/>
  </joint>

  <link name="camera_link"/>

  <!-- Gazebo plugins for sensors -->
  <gazebo reference="camera_link">
    <sensor name="camera" type="camera">
      <always_on>true</always_on>
      <update_rate>30.0</update_rate>
      <camera name="head_camera">
        <horizontal_fov>1.3962634</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>100</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <camera_name>robot/camera1</camera_name>
        <image_topic_name>image_raw</image_topic_name>
        <camera_info_topic_name>camera_info</camera_info_topic_name>
        <frame_name>camera_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>

  <gazebo reference="imu_link">
    <sensor name="imu_sensor" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <visualize>false</visualize>
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

</robot>
```

## Exercise 6: Complete Humanoid Robot

**Objective**: Create a complete humanoid robot with all major body parts.

**Instructions**:
1. Create a torso with proper mass distribution
2. Add head, two arms (with upper and lower segments), and two legs (with thigh, shin, and feet)
3. Use appropriate joint types and limits for human-like movement
4. Include proper inertial properties for stable simulation
5. Add at least one sensor

**Challenge**: Make the robot able to stand stably in simulation.

**Partial Solution** (full solution would be very long):
```xml
<?xml version="1.0"?>
<robot name="complete_humanoid">

  <!-- Base link -->
  <link name="base_link">
    <inertial>
      <mass value="0.001"/>
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
      <origin xyz="0 0 0.15"/>
      <geometry>
        <box size="0.25 0.15 0.3"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.15"/>
      <geometry>
        <box size="0.25 0.15 0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 0.15"/>
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
      <origin xyz="0 0 0"/>
      <inertia ixx="0.0011" ixy="0" ixz="0" iyy="0.0011" iyz="0" izz="0.0011"/>
    </inertial>
  </link>

  <!-- Left Arm -->
  <joint name="l_shoulder_pitch_joint" type="revolute">
    <parent link="torso_link"/>
    <child link="l_upper_arm_link"/>
    <origin xyz="0.05 0.1 0.25" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="2"/>
  </joint>

  <link name="l_upper_arm_link">
    <visual>
      <origin xyz="0 0 -0.15"/>
      <geometry>
        <cylinder radius="0.035" length="0.3"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15"/>
      <geometry>
        <cylinder radius="0.035" length="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <origin xyz="0 0 -0.15"/>
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
      <origin xyz="0 0 -0.125"/>
      <geometry>
        <cylinder radius="0.03" length="0.25"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.125"/>
      <geometry>
        <cylinder radius="0.03" length="0.25"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.6"/>
      <origin xyz="0 0 -0.125"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.0002"/>
    </inertial>
  </link>

  <!-- Continue similarly for right arm, legs, etc. -->

</robot>
```

## Exercise 7: URDF Validation and Debugging

**Objective**: Learn to validate and debug URDF files.

**Instructions**:
1. Create a URDF file with intentional errors (incorrect inertia values, wrong joint limits, etc.)
2. Use `check_urdf` command to identify the errors
3. Fix the errors one by one
4. Use `urdf_to_graphiz` to visualize the kinematic tree
5. Load the URDF in RViz to check visual representation

**Common validation commands**:
```bash
# Check URDF syntax
check_urdf your_robot.urdf

# Visualize kinematic tree
urdf_to_graphiz your_robot.urdf
dot -Tpng your_robot.gv -o your_robot.png

# Load in RViz
ros2 run rviz2 rviz2
# Then add RobotModel display and set Robot Description to your URDF
```

## Exercise 8: Advanced Xacro Features

**Objective**: Use advanced Xacro features like conditions and mathematical expressions.

**Instructions**:
1. Create a macro that creates a link with different properties based on parameters
2. Use conditional statements (xacro:if) to include/exclude parts
3. Use mathematical expressions for calculations
4. Create a parameterized robot that can be configured for different sizes

**Example**:
```xml
<xacro:macro name="configurable_arm" params="prefix length:=0.3 radius:=0.03 mass:=0.8 include_wrist:=true">
  <!-- Arm link -->
  <link name="${prefix}_arm_link">
    <visual>
      <geometry>
        <cylinder radius="${radius}" length="${length}"/>
      </geometry>
    </visual>
    <inertial>
      <mass value="${mass}"/>
      <inertia
        ixx="${0.0833333 * mass * (3*radius*radius + length*length)}"
        ixy="0" ixz="0"
        iyy="${0.0833333 * mass * (3*radius*radius + length*length)}"
        iyz="0"
        izz="${0.5 * mass * radius * radius}"/>
    </inertial>
  </link>

  <!-- Conditionally add wrist joint and hand -->
  <xacro:if value="${include_wrist}">
    <joint name="${prefix}_wrist_joint" type="revolute">
      <parent link="${prefix}_arm_link"/>
      <child link="${prefix}_hand_link"/>
      <origin xyz="0 0 -${length}" rpy="0 0 0"/>
      <axis xyz="0 0 1"/>
      <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
    </joint>

    <link name="${prefix}_hand_link">
      <visual>
        <geometry>
          <box size="0.08 0.06 0.04"/>
        </geometry>
      </visual>
      <inertial>
        <mass value="0.1"/>
        <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
      </inertial>
    </link>
  </xacro:if>
</xacro:macro>
```

## Best Practices Checklist

After completing these exercises, verify your URDF files meet these best practices:

- [ ] All links have proper visual, collision, and inertial elements
- [ ] Mass values are realistic and positive
- [ ] Inertia tensors follow the parallel axis theorem
- [ ] Joint limits are appropriate for the intended motion
- [ ] Origins are placed at logical locations (joint centers)
- [ ] The URDF forms a proper tree structure (no loops)
- [ ] Units are consistent (meters, radians, kilograms)
- [ ] Naming is consistent and descriptive
- [ ] The model is validated using `check_urdf`
- [ ] The kinematic tree is verified using `urdf_to_graphiz`

## Troubleshooting Tips

1. **Robot falls apart in simulation**: Check that all joints are properly connected and that the model forms a single connected component.

2. **Joints don't move properly**: Verify joint axes, limits, and origins are correctly specified.

3. **Simulation is unstable**: Check mass and inertia values; ensure they're physically realistic.

4. **Parts collide with each other**: Adjust joint limits or add collision filters if needed.

5. **URDF doesn't load**: Use `check_urdf` to identify syntax errors.

These exercises provide a comprehensive foundation for creating URDF files for humanoid robots. Practice each exercise to build confidence in URDF creation and validation.