---
sidebar_position: 3
---

# Nodes

## What are Nodes?

In ROS 2, a **node** is a process that performs computation. Nodes are the fundamental building blocks of a ROS 2 program. They are designed to be composable, meaning that multiple nodes can work together to perform complex robotic tasks.

## Node Characteristics

### Process-based Architecture
- Each node runs as a separate process
- Nodes can run on the same machine or distributed across multiple machines
- Communication between nodes is handled by the ROS 2 middleware

### Lifecycle Management
- Nodes can be started, stopped, and restarted independently
- Nodes can have different lifecycle states (unconfigured, inactive, active, finalized)
- Lifecycle management allows for more robust and predictable robot behavior

## Creating Nodes

### In Python
Nodes in Python are created by inheriting from the `Node` class in the `rclpy` library:

```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Hello from minimal_node!')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Humanoid Robot Node Example
Here's a more practical example for a humanoid robot joint controller:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

class JointController(Node):
    def __init__(self):
        super().__init__('joint_controller')

        # Publisher for joint commands
        self.joint_cmd_publisher = self.create_publisher(
            Float64,
            '/left_arm/joint_position',
            10
        )

        # Subscriber for joint states
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Joint controller node started')

    def joint_state_callback(self, msg):
        self.get_logger().info(f'Received joint states: {len(msg.name)} joints')

    def control_loop(self):
        # Simple control logic
        cmd_msg = Float64()
        cmd_msg.data = 0.0  # Default position
        self.joint_cmd_publisher.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Nodes in Humanoid Robots

In humanoid robot systems, nodes typically represent:

- **Sensor nodes**: Process data from cameras, IMUs, joint encoders
- **Controller nodes**: Handle low-level control of joints and actuators
- **Perception nodes**: Process sensor data to understand the environment
- **Planning nodes**: Generate motion plans and trajectories
- **Behavior nodes**: Implement high-level behaviors and decision-making

## Exercise: Creating a Humanoid Robot Node

Create a simple ROS 2 node that simulates a humanoid robot's head controller. The node should:

1. Create a publisher that publishes head orientation data (using geometry_msgs/PointStamped)
2. Create a subscriber that receives target coordinates (using geometry_msgs/Point)
3. Implement a timer that periodically publishes the current head orientation
4. Include proper logging and error handling

**Challenge**: Add a parameter to control the publishing frequency.

## Best Practices

- Keep nodes focused on a single responsibility
- Use appropriate logging for debugging and monitoring
- Handle exceptions gracefully
- Implement proper cleanup in the destructor
- Use parameters for configuration