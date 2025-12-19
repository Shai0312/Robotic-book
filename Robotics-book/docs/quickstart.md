---
title: Quick Start Guide
sidebar_position: 1
---

# Quick Start Guide

This guide will help you get started with ROS 2 and the concepts covered in this educational module. Follow these steps to begin your journey in robotics and AI integration.

## Prerequisites

Before starting, ensure you have:

- Basic knowledge of Python programming
- Understanding of fundamental programming concepts (variables, functions, classes)
- A computer with Ubuntu 22.04 LTS or Windows with WSL2
- Administrator access to install software

## Installation

### ROS 2 Humble Hawksbill

1. **Set up your sources**:
   ```bash
   # Add the ROS 2 apt repository
   sudo apt update && sudo apt install -y software-properties-common
   sudo add-apt-repository universe
   ```

2. **Add the ROS 2 GPG key**:
   ```bash
   sudo apt update && sudo apt install -y curl gnupg lsb-release
   curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
   ```

3. **Add the repository to your sources list**:
   ```bash
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   ```

4. **Install ROS 2**:
   ```bash
   sudo apt update
   sudo apt install ros-humble-desktop
   ```

5. **Install ROS 2 development tools**:
   ```bash
   sudo apt install python3-colcon-common-extensions python3-rosdep python3-vcstool
   ```

6. **Source the ROS 2 environment**:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

### Python Dependencies

Install the required Python packages:

```bash
pip3 install rclpy
```

## Creating Your First ROS 2 Node

Let's create a simple publisher node that publishes messages to a topic:

### 1. Create a Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

### 2. Create a Package

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_robot_package
```

### 3. Create a Publisher Node

Navigate to the package directory and create a publisher script:

```bash
cd ~/ros2_ws/src/my_robot_package/my_robot_package
```

Create `publisher_member_function.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 4. Update setup.py

In the package root (`~/ros2_ws/src/my_robot_package/`), update `setup.py`:

```python
from setuptools import find_packages, setup

package_name = 'my_robot_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'minimal_publisher = my_robot_package.publisher_member_function:main',
        ],
    },
)
```

### 5. Build and Run

```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_package
source install/setup.bash
ros2 run my_robot_package minimal_publisher
```

## Creating a Simple URDF Robot Model

Now let's create a simple URDF model for a robot:

Create `simple_robot.urdf`:

```xml
<?xml version="1.0"?>
<robot name="simple_robot">

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Arm link -->
  <link name="arm_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.3"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Joint connecting base and arm -->
  <joint name="arm_joint" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

</robot>
```

## Running Your First Simulation

### 1. Install Gazebo

```bash
sudo apt install ros-humble-gazebo-ros-pkgs
```

### 2. Launch a Simple World

```bash
# Terminal 1
gazebo --verbose

# Terminal 2 (after Gazebo loads)
ros2 run gazebo_ros spawn_entity.py -entity simple_robot -file /path/to/simple_robot.urdf -x 0 -y 0 -z 1
```

## Understanding the Agent → Controller → Actuator Loop

One of the key concepts in this module is the agent → controller → actuator loop:

1. **Agent**: Makes high-level decisions based on sensor input and goals
2. **Controller**: Translates high-level commands into specific actuator commands
3. **Actuator**: Physical components that move the robot

Here's a simple example:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class RobotAgent(Node):
    def __init__(self):
        super().__init__('robot_agent')

        # Subscribe to sensor data
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            10)

        # Publisher for velocity commands
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)

        self.obstacle_detected = False

    def laser_callback(self, msg):
        # Simple obstacle detection
        if min(msg.ranges) < 1.0:  # If obstacle within 1 meter
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False

    def control_loop(self):
        cmd_vel = Twist()

        if self.obstacle_detected:
            # Agent decision: obstacle detected, turn
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.5  # Controller output
        else:
            # Agent decision: no obstacle, move forward
            cmd_vel.linear.x = 0.5   # Controller output
            cmd_vel.angular.z = 0.0

        self.publisher.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    agent = RobotAgent()
    rclpy.spin(agent)
    agent.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Key Concepts to Remember

1. **Nodes communicate through topics, services, and actions**
2. **URDF defines the physical structure of your robot**
3. **The agent → controller → actuator loop is fundamental to robot autonomy**
4. **Simulation is crucial for testing before deploying on real hardware**
5. **TF (Transforms) manages coordinate frame relationships**

## Next Steps

After completing this quick start:

1. Explore the detailed chapters on ROS 2 concepts
2. Learn about Python-ROS integration
3. Dive into URDF modeling for humanoid robots
4. Experiment with more complex robot models
5. Try integrating AI agents with your robot control system

## Troubleshooting

### Common Issues

**Q: Command 'ros2' not found**
A: Make sure you've sourced the ROS 2 environment: `source /opt/ros/humble/setup.bash`

**Q: Python modules not found**
A: Ensure you're using the correct Python environment and have installed the required packages

**Q: Gazebo won't start**
A: Check that your graphics drivers are properly installed and that you have sufficient system resources

For additional help, visit the [ROS Answers](https://answers.ros.org/questions/) forum or check the official [ROS 2 documentation](https://docs.ros.org/en/humble/).