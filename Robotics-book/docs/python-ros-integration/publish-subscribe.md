---
sidebar_position: 4
---

# Publishing and Subscribing to Topics

## Overview

In this section, we'll dive deep into implementing publishers and subscribers in Python using rclpy. The publish-subscribe pattern is fundamental to ROS 2 and enables decoupled communication between nodes, which is essential for building robust humanoid robot systems.

## Creating Publishers

### Basic Publisher

Let's start with a basic publisher that sends simple string messages:

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

    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        minimal_publisher.get_logger().info('Interrupted, shutting down...')
    finally:
        minimal_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Publisher with Custom Message Types

Publishers often need to send custom message types. Here's an example using a more complex message:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')

        # Create publisher for joint states
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)

        # Create timer to periodically publish joint states
        self.timer = self.create_timer(0.02, self.publish_joint_states)  # 50Hz

        # Define joint names for a humanoid robot
        self.joint_names = [
            'head_yaw', 'head_pitch',
            'l_shoulder_pitch', 'l_shoulder_roll', 'l_elbow_yaw', 'l_elbow_roll',
            'r_shoulder_pitch', 'r_shoulder_roll', 'r_elbow_yaw', 'r_elbow_roll',
            'l_hip_yaw_pitch', 'l_hip_roll', 'l_hip_pitch', 'l_knee_pitch', 'l_ankle_pitch', 'l_ankle_roll',
            'r_hip_yaw_pitch', 'r_hip_roll', 'r_hip_pitch', 'r_knee_pitch', 'r_ankle_pitch', 'r_ankle_roll'
        ]

        self.get_logger().info('Joint state publisher started')

    def publish_joint_states(self):
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        msg.name = self.joint_names
        msg.position = [0.0] * len(self.joint_names)  # Initialize with zeros
        msg.velocity = [0.0] * len(self.joint_names)
        msg.effort = [0.0] * len(self.joint_names)

        # In a real robot, you would get actual joint positions from encoders
        # For simulation, we'll use placeholder values

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted, shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Creating Subscribers

### Basic Subscriber

Here's a basic subscriber that receives string messages:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()

    try:
        rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt:
        minimal_subscriber.get_logger().info('Interrupted, shutting down...')
    finally:
        minimal_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Subscriber with Message Filters

For more complex scenarios, you might need to process messages differently based on their content:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState

class SmartSubscriber(Node):
    def __init__(self):
        super().__init__('smart_subscriber')

        # Subscribe to different topics
        self.joint_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)

        self.command_subscriber = self.create_subscription(
            String,
            '/robot_commands',
            self.command_callback,
            10)

    def joint_state_callback(self, msg):
        # Process joint state messages
        self.get_logger().debug(f'Received joint state for {len(msg.name)} joints')

        # Example: Check for dangerous joint positions
        for i, (name, pos) in enumerate(zip(msg.name, msg.position)):
            if abs(pos) > 3.0:  # Assuming dangerous position threshold
                self.get_logger().warn(f'Dangerous position detected for joint {name}: {pos}')

    def command_callback(self, msg):
        # Process command messages
        command = msg.data.lower()
        if command == 'stop':
            self.get_logger().info('Stop command received - initiating safe stop')
            # Implement safe stop procedure
        elif command.startswith('move_to'):
            self.get_logger().info(f'Move command received: {command}')
            # Parse and execute move command

def main(args=None):
    rclpy.init(args=args)
    node = SmartSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted, shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Advanced Publisher-Subscriber Patterns

### Quality of Service Settings

For humanoid robots, you often need to configure Quality of Service (QoS) settings:

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import JointState

class QoSPublisher(Node):
    def __init__(self):
        super().__init__('qos_publisher')

        # Define different QoS profiles for different data types
        # For joint states - reliable delivery, keep last 10 messages
        joint_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST
        )

        # For camera images - best effort, with history for late-joining subscribers
        image_qos = QoSProfile(
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        self.joint_publisher = self.create_publisher(JointState, '/joint_states', joint_qos)
        # self.image_publisher = self.create_publisher(Image, '/camera/image_raw', image_qos)

    def publish_joint_states(self):
        # Implementation for publishing joint states with appropriate QoS
        pass

def main(args=None):
    rclpy.init(args=args)
    node = QoSPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted, shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Publisher-Subscriber with Synchronization

Sometimes you need to synchronize messages from multiple topics:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from message_filters import ApproximateTimeSynchronizer, Subscriber

class SynchronizedSubscriber(Node):
    def __init__(self):
        super().__init__('synchronized_subscriber')

        # Create subscribers for both topics
        joint_sub = Subscriber(self, JointState, '/joint_states')
        imu_sub = Subscriber(self, Imu, '/imu/data')

        # Synchronize messages within 0.1 second tolerance
        ats = ApproximateTimeSynchronizer(
            [joint_sub, imu_sub],
            queue_size=10,
            slop=0.1
        )
        ats.registerCallback(self.sync_callback)

    def sync_callback(self, joint_msg, imu_msg):
        # Process synchronized joint state and IMU data
        self.get_logger().info(f'Synchronized data: {len(joint_msg.name)} joints, IMU orientation available')

        # Use synchronized data for state estimation
        self.estimate_robot_state(joint_msg, imu_msg)

    def estimate_robot_state(self, joint_msg, imu_msg):
        # Implementation of state estimation using synchronized data
        pass

def main(args=None):
    rclpy.init(args=args)
    node = SynchronizedSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted, shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Publisher-Subscriber in Humanoid Robotics Context

### Sensor Data Publisher

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu, PointCloud2
import numpy as np

class HumanoidSensorPublisher(Node):
    def __init__(self):
        super().__init__('humanoid_sensor_publisher')

        # Publishers for different sensor types
        self.joint_pub = self.create_publisher(JointState, '/humanoid/joint_states', 10)
        self.imu_pub = self.create_publisher(Imu, '/humanoid/imu', 10)
        self.ft_pub = self.create_publisher(JointState, '/humanoid/ft_sensors', 10)  # Force/torque

        # Timer for sensor data publishing
        self.timer = self.create_timer(0.01, self.publish_sensor_data)  # 100Hz

        # Initialize sensor data
        self.joint_names = [
            'head_yaw', 'head_pitch',
            'l_shoulder_pitch', 'l_shoulder_roll', 'l_elbow_yaw', 'l_elbow_roll',
            'r_shoulder_pitch', 'r_shoulder_roll', 'r_elbow_yaw', 'r_elbow_roll',
            'l_hip_yaw_pitch', 'l_hip_roll', 'l_hip_pitch', 'l_knee_pitch', 'l_ankle_pitch', 'l_ankle_roll',
            'r_hip_yaw_pitch', 'r_hip_roll', 'r_hip_pitch', 'r_knee_pitch', 'r_ankle_pitch', 'r_ankle_roll'
        ]

        self.get_logger().info('Humanoid sensor publisher initialized')

    def publish_sensor_data(self):
        # Simulate or read actual sensor data
        self.publish_joint_states()
        self.publish_imu_data()
        self.publish_force_torque_data()

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.name = self.joint_names

        # In a real robot, read from encoders
        # For simulation, generate realistic positions
        msg.position = [np.random.uniform(-0.1, 0.1) for _ in self.joint_names]
        msg.velocity = [0.0] * len(self.joint_names)
        msg.effort = [0.0] * len(self.joint_names)

        self.joint_pub.publish(msg)

    def publish_imu_data(self):
        from std_msgs.msg import Header
        from geometry_msgs.msg import Vector3, Quaternion

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        # Simulate IMU data
        msg.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        msg.angular_velocity = Vector3(x=0.0, y=0.0, z=0.0)
        msg.linear_acceleration = Vector3(x=0.0, y=0.0, z=9.81)  # Gravity

        self.imu_pub.publish(msg)

    def publish_force_torque_data(self):
        # Similar to joint states but for force/torque sensors
        pass

def main(args=None):
    rclpy.init(args=args)
    node = HumanoidSensorPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted, shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Best Practices

### 1. Proper Publisher/Subscriber Management

```python
class ManagedCommunicationNode(Node):
    def __init__(self):
        super().__init__('managed_communication_node')
        self.publishers = {}
        self.subscribers = {}

        # Create publishers and store references
        self.publishers['commands'] = self.create_publisher(String, '/robot_commands', 10)
        self.publishers['states'] = self.create_publisher(JointState, '/joint_states', 10)

        # Create subscribers and store references
        self.subscribers['sensor_data'] = self.create_subscription(
            JointState, '/sensor_data', self.sensor_callback, 10
        )

    def destroy_node(self):
        # Clean up publishers and subscribers
        for pub in self.publishers.values():
            pub.destroy()
        for sub in self.subscribers.values():
            sub.destroy()
        super().destroy_node()
```

### 2. Error Handling in Callbacks

```python
def sensor_callback(self, msg):
    try:
        # Process the message
        processed_data = self.process_sensor_data(msg)
        self.publish_processed_data(processed_data)
    except ValueError as e:
        self.get_logger().error(f'Invalid sensor data: {e}')
    except Exception as e:
        self.get_logger().error(f'Unexpected error processing sensor data: {e}')
        # Don't let one bad message crash the entire node
```

### 3. Efficient Message Handling

```python
def efficient_callback(self, msg):
    # Early exit for messages we don't need to process
    if not self.should_process_message(msg):
        return

    # Process only the data we need
    relevant_data = self.extract_relevant_data(msg)
    self.update_internal_state(relevant_data)
```

## Common Pitfalls and Solutions

### 1. Publisher/Subscriber Mismatch
**Problem**: Publisher and subscriber use different message types or topic names
**Solution**: Always verify topic names and message types match exactly

### 2. QoS Profile Incompatibility
**Problem**: Publisher and subscriber QoS profiles are incompatible
**Solution**: Ensure compatible QoS profiles (e.g., both RELIABLE or both BEST_EFFORT)

### 3. Memory Issues with Large Messages
**Problem**: Large messages (e.g., images, point clouds) consuming too much memory
**Solution**: Use appropriate queue sizes and consider message throttling

In the next section, we'll explore how to implement services in Python for request-response communication patterns.