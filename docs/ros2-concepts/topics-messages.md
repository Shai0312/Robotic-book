---
sidebar_position: 4
---

# Topics and Messages

## Topics

### What are Topics?

Topics in ROS 2 provide a way for nodes to exchange data through a **publish-subscribe** communication pattern. A topic is a named bus over which nodes exchange messages.

### Publish-Subscribe Pattern

- **Publisher**: A node that sends messages to a topic
- **Subscriber**: A node that receives messages from a topic
- **Topic**: The named channel through which messages flow

### Characteristics of Topics

- **Asynchronous**: Publishers and subscribers don't need to be active simultaneously
- **Unidirectional**: Data flows from publishers to subscribers
- **Anonymous**: Publishers and subscribers are unaware of each other
- **Many-to-many**: Multiple publishers can publish to the same topic, multiple subscribers can subscribe to the same topic

### Example in Python

```python
# Publisher
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

# Subscriber
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
```

### Humanoid Robot Example: Joint States

Here's a more practical example using JointState messages for a humanoid robot:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')

        # Publisher for joint states
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Timer to publish joint states at 50Hz
        self.timer = self.create_timer(0.02, self.publish_joint_states)

        # Simulated joint positions
        self.joint_positions = [0.0] * 28  # 28 joints for a humanoid
        self.joint_names = [
            'left_hip_yaw', 'left_hip_roll', 'left_hip_pitch', 'left_knee',
            'left_ankle_pitch', 'left_ankle_roll', 'right_hip_yaw', 'right_hip_roll',
            'right_hip_pitch', 'right_knee', 'right_ankle_pitch', 'right_ankle_roll',
            'left_shoulder_pitch', 'left_shoulder_roll', 'left_shoulder_yaw', 'left_elbow',
            'right_shoulder_pitch', 'right_shoulder_roll', 'right_shoulder_yaw', 'right_elbow',
            'head_yaw', 'head_pitch'
            # ... add more joint names as needed
        ]

        self.get_logger().info('Joint state publisher started')

    def publish_joint_states(self):
        # Create joint state message
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # Set joint names and positions
        msg.name = self.joint_names
        msg.position = self.joint_positions
        msg.velocity = [0.0] * len(self.joint_positions)  # Zero velocity
        msg.effort = [0.0] * len(self.joint_positions)    # Zero effort

        # Publish the message
        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Messages

### What are Messages?

Messages are the data structures that are passed between nodes through topics. They define the format of the data being exchanged.

### Message Types

Messages are defined using `.msg` files and can contain various data types:
- Basic types: `bool`, `int8`, `uint8`, `float32`, `string`, etc.
- Arrays: `bool[]`, `int32[]`, `string[]`, etc.
- Complex types: Custom message structures

### Common Message Packages

- `std_msgs`: Basic message types
- `geometry_msgs`: Messages for geometry (points, poses, transforms)
- `sensor_msgs`: Messages for sensors (images, laser scans, joint states)
- `nav_msgs`: Messages for navigation

## Topics in Humanoid Robots

In humanoid robot systems, common topics include:

- `/joint_states`: Current positions, velocities, and efforts of robot joints
- `/sensor_data`: Data from various sensors (IMU, cameras, force/torque sensors)
- `/cmd_vel`: Velocity commands for base movement
- `/move_group/goal`: Motion planning goals
- `/humanoid_control/joint_commands`: Commands for individual joint control

## Quality of Service (QoS)

ROS 2 provides QoS settings for topics to configure communication behavior:
- **Reliability**: Best effort or reliable delivery
- **Durability**: Volatile or transient local (for late-joining subscribers)
- **History**: Keep last N messages or keep all messages
- **Deadline**: Maximum time between messages

## Exercise: Implement a Humanoid Sensor Publisher

Create a ROS 2 node that publishes sensor data for a humanoid robot:

1. Create a publisher that publishes IMU data (using sensor_msgs/Imu)
2. Create a publisher that publishes laser scan data (using sensor_msgs/LaserScan)
3. Implement timers to publish each sensor type at appropriate frequencies
4. Include realistic sensor data simulation with some noise
5. Use appropriate QoS settings for sensor data

**Challenge**: Implement a subscriber that subscribes to both sensor topics and fuses the data to estimate the robot's state.