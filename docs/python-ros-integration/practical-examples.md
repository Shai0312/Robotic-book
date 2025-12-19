---
sidebar_position: 7
---

# Practical Code Examples for Python-ROS Integration

## Overview

This section provides practical, ready-to-run code examples that demonstrate the concepts covered in Chapter 2. Each example is designed to be educational and directly applicable to humanoid robot development.

## Example 1: Simple Publisher-Subscriber Pair

### Publisher Node

```python
#!/usr/bin/env python3
"""
Simple publisher that sends robot sensor data
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math

class RobotSensorPublisher(Node):
    def __init__(self):
        super().__init__('robot_sensor_publisher')

        # Create publishers
        self.sensor_pub = self.create_publisher(String, 'robot_status', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # Create timer for periodic publishing
        self.timer = self.create_timer(0.1, self.publish_data)

        # Initialize counter and joint names
        self.counter = 0
        self.joint_names = ['joint1', 'joint2', 'joint3']

        self.get_logger().info('Robot Sensor Publisher started')

    def publish_data(self):
        # Publish simple status message
        status_msg = String()
        status_msg.data = f'Robot is operational - cycle {self.counter}'
        self.sensor_pub.publish(status_msg)

        # Publish joint state message
        joint_msg = JointState()
        joint_msg.header = Header()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.header.frame_id = 'base_link'

        joint_msg.name = self.joint_names
        joint_msg.position = [math.sin(self.counter * 0.1 + i) for i in range(len(self.joint_names))]
        joint_msg.velocity = [0.0] * len(self.joint_names)
        joint_msg.effort = [0.0] * len(self.joint_names)

        self.joint_pub.publish(joint_msg)

        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    publisher = RobotSensorPublisher()

    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        publisher.get_logger().info('Shutting down publisher...')
    finally:
        publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Subscriber Node

```python
#!/usr/bin/env python3
"""
Simple subscriber that receives and processes robot data
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState

class RobotDataProcessor(Node):
    def __init__(self):
        super().__init__('robot_data_processor')

        # Create subscribers
        self.status_sub = self.create_subscription(
            String, 'robot_status', self.status_callback, 10
        )
        self.joint_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_callback, 10
        )

        self.get_logger().info('Robot Data Processor started')

    def status_callback(self, msg):
        self.get_logger().info(f'Received status: {msg.data}')

    def joint_callback(self, msg):
        if len(msg.position) > 0:
            avg_position = sum(msg.position) / len(msg.position)
            self.get_logger().info(f'Average joint position: {avg_position:.3f}')

def main(args=None):
    rclpy.init(args=args)
    processor = RobotDataProcessor()

    try:
        rclpy.spin(processor)
    except KeyboardInterrupt:
        processor.get_logger().info('Shutting down processor...')
    finally:
        processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Example 2: Service Client-Server Pair

### Service Server

```python
#!/usr/bin/env python3
"""
Service server that provides robot control commands
"""
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool

class RobotControlService(Node):
    def __init__(self):
        super().__init__('robot_control_service')

        # Create service
        self.srv = self.create_service(
            SetBool,
            'robot_control_cmd',
            self.control_callback
        )

        self.robot_enabled = False
        self.get_logger().info('Robot Control Service started')

    def control_callback(self, request, response):
        if request.data:
            self.get_logger().info('Enabling robot')
            self.robot_enabled = True
            response.success = True
            response.message = 'Robot enabled successfully'
        else:
            self.get_logger().info('Disabling robot')
            self.robot_enabled = False
            response.success = True
            response.message = 'Robot disabled successfully'

        return response

def main(args=None):
    rclpy.init(args=args)
    service = RobotControlService()

    try:
        rclpy.spin(service)
    except KeyboardInterrupt:
        service.get_logger().info('Shutting down service...')
    finally:
        service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Client

```python
#!/usr/bin/env python3
"""
Service client that sends control commands to robot
"""
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool

class RobotControlClient(Node):
    def __init__(self):
        super().__init__('robot_control_client')

        # Create client
        self.cli = self.create_client(SetBool, 'robot_control_cmd')

        # Wait for service
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for robot control service...')

        self.request = SetBool.Request()

    def send_command(self, enable):
        self.request.data = enable
        future = self.cli.call_async(self.request)
        return future

def main(args=None):
    rclpy.init(args=args)
    client = RobotControlClient()

    # Enable robot
    future = client.send_command(True)
    rclpy.spin_until_future_complete(client, future)

    response = future.result()
    client.get_logger().info(f'Enable response: {response.message}')

    # Disable robot after 2 seconds
    from time import sleep
    sleep(2)

    future = client.send_command(False)
    rclpy.spin_until_future_complete(client, future)

    response = future.result()
    client.get_logger().info(f'Disable response: {response.message}')

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Example 3: Humanoid Robot Walk Controller

This example demonstrates a complete humanoid robot walking controller:

```python
#!/usr/bin/env python3
"""
Humanoid robot walking controller implementing agent-controller-actuator loop
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import numpy as np
import math

class HumanoidWalkController(Node):
    def __init__(self):
        super().__init__('humanoid_walk_controller')

        # Publishers and subscribers
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.joint_state_sub = self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10)
        self.joint_cmd_pub = self.create_publisher(JointTrajectory, 'joint_trajectory_commands', 10)

        # Control timer
        self.control_timer = self.create_timer(0.02, self.control_loop)  # 50Hz

        # Robot parameters
        self.joint_names = [
            'l_hip_roll', 'l_hip_pitch', 'l_knee_pitch', 'l_ankle_pitch', 'l_ankle_roll',
            'r_hip_roll', 'r_hip_pitch', 'r_knee_pitch', 'r_ankle_pitch', 'r_ankle_roll'
        ]

        # Walking parameters
        self.step_length = 0.1  # meters
        self.step_height = 0.02  # meters
        self.step_duration = 0.8  # seconds
        self.walk_phase = 0.0

        # Internal state
        self.desired_velocity = Twist()
        self.current_joint_state = None
        self.is_walking = False

        self.get_logger().info('Humanoid Walk Controller initialized')

    def cmd_vel_callback(self, msg):
        """Receive velocity commands"""
        self.desired_velocity = msg
        self.is_walking = abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01

    def joint_state_callback(self, msg):
        """Update current joint state"""
        self.current_joint_state = msg

    def control_loop(self):
        """Main walking control loop"""
        if self.current_joint_state is None:
            return

        if self.is_walking:
            # Generate walking trajectory
            trajectory = self.generate_walking_trajectory()
            self.joint_cmd_pub.publish(trajectory)
        else:
            # Stop walking - return to neutral position
            neutral_trajectory = self.generate_neutral_trajectory()
            self.joint_cmd_pub.publish(neutral_trajectory)

    def generate_walking_trajectory(self):
        """Generate walking trajectory based on desired velocity"""
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names

        # Calculate step parameters based on velocity
        walk_speed = self.desired_velocity.linear.x
        turn_rate = self.desired_velocity.angular.z

        # Generate trajectory points
        num_points = 20
        for i in range(num_points):
            point = JointTrajectoryPoint()

            # Time from start
            time_from_start = Duration()
            time_from_start.sec = 0
            time_from_start.nanosec = int((i / num_points) * self.step_duration * 1e9)
            point.time_from_start = time_from_start

            # Calculate joint positions for walking gait
            positions = []
            phase = (self.walk_phase + i * 0.1) % (2 * math.pi)

            for j, joint_name in enumerate(self.joint_names):
                if 'hip' in joint_name:
                    # Hip joints move with walking gait
                    base_pos = 0.0 if 'l_' in joint_name else 0.0
                    gait_pos = walk_speed * 0.1 * math.sin(phase + j * 0.5)
                    turn_pos = turn_rate * 0.05 * (1 if 'l_' in joint_name else -1)
                    pos = base_pos + gait_pos + turn_pos
                elif 'knee' in joint_name:
                    # Knee joints bend during walking
                    base_pos = -0.3
                    gait_pos = walk_speed * 0.05 * math.sin(phase * 2 + j * 0.5)
                    pos = base_pos + gait_pos
                elif 'ankle' in joint_name:
                    # Ankle joints adjust for balance
                    base_pos = 0.1 if 'l_' in joint_name else -0.1
                    gait_pos = walk_speed * 0.02 * math.cos(phase + j * 0.5)
                    pos = base_pos + gait_pos
                else:
                    pos = 0.0

                positions.append(pos)

            point.positions = positions
            trajectory.points.append(point)

        # Update walking phase
        self.walk_phase += 0.1 * abs(walk_speed) * 0.5

        return trajectory

    def generate_neutral_trajectory(self):
        """Generate neutral standing position trajectory"""
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.time_from_start = Duration(sec=0, nanosec=50000000)  # 50ms

        # Neutral standing positions
        neutral_positions = [0.0, -0.3, 0.6, -0.3, 0.0,  # Left leg
                           0.0, -0.3, 0.6, -0.3, 0.0]   # Right leg

        point.positions = neutral_positions
        trajectory.points = [point]

        return trajectory

def main(args=None):
    rclpy.init(args=args)
    controller = HumanoidWalkController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down walk controller...')
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Example 4: Robot State Estimator

This example shows how to estimate robot state from sensor data:

```python
#!/usr/bin/env python3
"""
Robot state estimator that fuses sensor data to estimate robot pose
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import PoseStamped, TwistStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import numpy as np

class RobotStateEstimator(Node):
    def __init__(self):
        super().__init__('robot_state_estimator')

        # Subscribers
        self.joint_sub = self.create_subscription(JointState, 'joint_states', self.joint_callback, 10)
        self.imu_sub = self.create_subscription(Imu, 'imu/data', self.imu_callback, 10)

        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, 'robot_pose', 10)
        self.twist_pub = self.create_publisher(TwistStamped, 'robot_twist', 10)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer for state estimation
        self.estimation_timer = self.create_timer(0.05, self.estimate_state)  # 20Hz

        # Internal state
        self.joint_state = None
        self.imu_data = None
        self.estimated_pose = np.array([0.0, 0.0, 0.0])  # x, y, theta
        self.estimated_twist = np.array([0.0, 0.0, 0.0])  # linear_x, linear_y, angular_z

        self.get_logger().info('Robot State Estimator initialized')

    def joint_callback(self, msg):
        """Update joint state"""
        self.joint_state = msg

    def imu_callback(self, msg):
        """Update IMU data"""
        self.imu_data = msg

    def estimate_state(self):
        """Estimate robot state from sensor data"""
        if self.joint_state is None or self.imu_data is None:
            return

        # Estimate pose from forward kinematics (simplified)
        self.estimate_pose_from_joints()

        # Estimate twist from IMU and joint velocities
        self.estimate_twist_from_sensors()

        # Publish estimated state
        self.publish_estimated_state()

    def estimate_pose_from_joints(self):
        """Estimate pose based on joint positions (simplified)"""
        # In a real system, this would use forward kinematics
        # This is a simplified example
        if len(self.joint_state.position) > 0:
            # Simple odometry integration (not accurate in practice)
            dt = 0.05  # 20Hz
            linear_vel = self.estimated_twist[0]  # From previous estimate
            angular_vel = self.estimated_twist[2]

            # Update pose using simple integration
            self.estimated_pose[0] += linear_vel * dt * np.cos(self.estimated_pose[2])
            self.estimated_pose[1] += linear_vel * dt * np.sin(self.estimated_pose[2])
            self.estimated_pose[2] += angular_vel * dt

    def estimate_twist_from_sensors(self):
        """Estimate twist from IMU and joint velocities"""
        if self.imu_data is not None:
            # Use IMU angular velocity
            self.estimated_twist[2] = self.imu_data.angular_velocity.z

        if self.joint_state is not None and len(self.joint_state.velocity) > 0:
            # Estimate linear velocity from joint velocities (simplified)
            # In practice, this would use more sophisticated methods
            avg_joint_vel = np.mean(np.abs(self.joint_state.velocity))
            self.estimated_twist[0] = avg_joint_vel * 0.1  # Scale factor

    def publish_estimated_state(self):
        """Publish estimated state"""
        # Publish pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = float(self.estimated_pose[0])
        pose_msg.pose.position.y = float(self.estimated_pose[1])
        pose_msg.pose.position.z = 0.0  # Assuming 2D movement

        # Convert angle to quaternion
        from math import sin, cos
        theta = self.estimated_pose[2]
        pose_msg.pose.orientation.w = cos(theta / 2)
        pose_msg.pose.orientation.z = sin(theta / 2)

        self.pose_pub.publish(pose_msg)

        # Publish twist
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = 'base_link'
        twist_msg.twist.linear.x = float(self.estimated_twist[0])
        twist_msg.twist.linear.y = float(self.estimated_twist[1])
        twist_msg.twist.angular.z = float(self.estimated_twist[2])

        self.twist_pub.publish(twist_msg)

        # Broadcast transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = float(self.estimated_pose[0])
        t.transform.translation.y = float(self.estimated_pose[1])
        t.transform.translation.z = 0.0

        t.transform.rotation.w = cos(theta / 2)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = sin(theta / 2)

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    estimator = RobotStateEstimator()

    try:
        rclpy.spin(estimator)
    except KeyboardInterrupt:
        estimator.get_logger().info('Shutting down state estimator...')
    finally:
        estimator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Running the Examples

### Setup and Execution

1. **Save the code** to individual Python files in your ROS 2 workspace
2. **Make them executable**: `chmod +x your_script.py`
3. **Source your ROS 2 environment**: `source /opt/ros/humble/setup.bash` (or your ROS version)
4. **Run the nodes**:
   ```bash
   # Terminal 1: Run publisher or service server
   python3 robot_publisher.py

   # Terminal 2: Run subscriber or client
   python3 robot_subscriber.py
   ```

### Testing the Complete System

To test the complete agent-controller-actuator loop:

1. Start the state estimator:
   ```bash
   python3 robot_state_estimator.py
   ```

2. Start the walk controller:
   ```bash
   python3 humanoid_walk_controller.py
   ```

3. Send velocity commands:
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.2}, angular: {z: 0.1}}'
   ```

## Common Issues and Solutions

### 1. Import Errors
**Problem**: `ModuleNotFoundError` for ROS 2 packages
**Solution**: Ensure ROS 2 is properly sourced and packages are installed

### 2. Topic/Service Not Found
**Problem**: Publishers/subscribers don't connect
**Solution**: Verify topic/service names match exactly and nodes are on the same ROS domain

### 3. Timing Issues
**Problem**: Control loops running too fast or too slow
**Solution**: Adjust timer periods and consider using rate limiters

### 4. Memory Issues
**Problem**: High memory usage with large messages
**Solution**: Use appropriate queue sizes and message throttling

## Best Practices Demonstrated

1. **Proper error handling** with try-catch blocks
2. **Resource management** with proper cleanup
3. **Modular design** separating concerns
4. **Logging** for debugging and monitoring
5. **Parameter configuration** for flexibility
6. **Safety checks** before executing commands

These practical examples provide a solid foundation for developing more complex humanoid robot applications using Python and ROS 2. Each example can be extended and modified to suit specific robot platforms and use cases.