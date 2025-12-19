---
sidebar_position: 3
---

# Creating ROS 2 Nodes in Python

## Overview

In this section, we'll explore how to create ROS 2 nodes using Python and rclpy. We'll cover the fundamental concepts and build up to more complex examples that demonstrate real-world usage patterns in humanoid robotics.

## Basic Node Structure

### Minimal Node Example

Let's start with the most basic ROS 2 node in Python:

```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Minimal node started')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Key Components Breakdown

1. **Import statements**: Import necessary modules from rclpy
2. **Node class**: Inherit from `rclpy.node.Node`
3. **Constructor**: Call parent constructor with node name
4. **Main function**: Initialize, create node, spin, and cleanup
5. **Execution guard**: `if __name__ == '__main__':` pattern

## Advanced Node Patterns

### Node with Parameters

```python
import rclpy
from rclpy.node import Node

class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with default values
        self.declare_parameter('robot_name', 'humanoid_robot')
        self.declare_parameter('control_rate', 50)  # Hz

        # Access parameter values
        self.robot_name = self.get_parameter('robot_name').value
        self.control_rate = self.get_parameter('control_rate').value

        self.get_logger().info(f'Node initialized for {self.robot_name} at {self.control_rate}Hz')

def main(args=None):
    rclpy.init(args=args)
    node = ParameterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Node with Timer

```python
import rclpy
from rclpy.node import Node

class TimerNode(Node):
    def __init__(self):
        super().__init__('timer_node')

        # Create a timer that calls the callback every 0.1 seconds
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        self.get_logger().info(f'Timer callback executed {self.counter} times')
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = TimerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Practical Example: Humanoid Robot State Publisher

Let's create a more practical example that simulates publishing humanoid robot joint states:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math

class HumanoidStatePublisher(Node):
    def __init__(self):
        super().__init__('humanoid_state_publisher')

        # Declare parameters
        self.declare_parameter('robot_name', 'nao')
        self.declare_parameter('publish_rate', 50)  # Hz

        self.robot_name = self.get_parameter('robot_name').value
        self.publish_rate = self.get_parameter('publish_rate').value

        # Create publisher for joint states
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)

        # Create timer for publishing
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_joint_states)

        # Initialize joint positions (simulated)
        self.joint_names = [
            'head_yaw', 'head_pitch',
            'l_shoulder_pitch', 'l_shoulder_roll', 'l_elbow_yaw', 'l_elbow_roll',
            'r_shoulder_pitch', 'r_shoulder_roll', 'r_elbow_yaw', 'r_elbow_roll',
            'l_hip_yaw_pitch', 'l_hip_roll', 'l_hip_pitch', 'l_knee_pitch', 'l_ankle_pitch', 'l_ankle_roll',
            'r_hip_yaw_pitch', 'r_hip_roll', 'r_hip_pitch', 'r_knee_pitch', 'r_ankle_pitch', 'r_ankle_roll'
        ]

        self.time_step = 0.0
        self.get_logger().info(f'Started {self.robot_name} state publisher')

    def publish_joint_states(self):
        # Create joint state message
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        msg.name = self.joint_names
        msg.position = []
        msg.velocity = []
        msg.effort = []

        # Simulate some periodic joint movements
        for i, joint_name in enumerate(self.joint_names):
            # Create different movement patterns for different joint groups
            if 'head' in joint_name:
                pos = 0.1 * math.sin(self.time_step * 0.5)
            elif 'shoulder' in joint_name or 'elbow' in joint_name:
                pos = 0.2 * math.sin(self.time_step * 0.3 + i)
            elif 'hip' in joint_name or 'knee' in joint_name or 'ankle' in joint_name:
                pos = 0.15 * math.sin(self.time_step * 0.4 + i * 0.5)
            else:
                pos = 0.0

            msg.position.append(pos)
            msg.velocity.append(0.0)  # Not calculated in this example
            msg.effort.append(0.0)    # Not calculated in this example

        # Publish the message
        self.publisher_.publish(msg)
        self.time_step += 0.01

def main(args=None):
    rclpy.init(args=args)
    node = HumanoidStatePublisher()

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

## Node Lifecycle Management

### Lifecycle Node Concepts

ROS 2 supports lifecycle nodes that have well-defined states:
- Unconfigured → Inactive → Active → Finalized

```python
import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from lifecycle_msgs.msg import Transition

class LifecycleExampleNode(LifecycleNode):
    def __init__(self):
        super().__init__('lifecycle_example_node')
        self.get_logger().info('Lifecycle node created, waiting for configuration')

    def on_configure(self, state):
        self.get_logger().info('Configuring node...')
        # Initialize resources here
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.get_logger().info('Activating node...')
        # Activate functionality here
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        self.get_logger().info('Deactivating node...')
        # Deactivate functionality here
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        self.get_logger().info('Cleaning up node...')
        # Clean up resources here
        return TransitionCallbackReturn.SUCCESS
```

## Best Practices for Node Creation

### 1. Proper Resource Management

```python
import rclpy
from rclpy.node import Node

class ResourceManagedNode(Node):
    def __init__(self):
        super().__init__('resource_managed_node')
        self.timers = []
        self.publishers = []
        self.subscribers = []

        # Create and store resources
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.pub = self.create_publisher(String, 'topic', 10)

    def destroy_node(self):
        # Clean up resources before destroying the node
        for timer in self.timers:
            timer.destroy()
        for pub in self.publishers:
            pub.destroy()
        for sub in self.subscribers:
            sub.destroy()
        super().destroy_node()
```

### 2. Error Handling

```python
def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MyNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error during execution: {e}')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
```

### 3. Logging Best Practices

```python
class WellLoggedNode(Node):
    def __init__(self):
        super().__init__('well_logged_node')
        self.get_logger().info('Node initialized successfully')

    def some_method(self):
        try:
            # Do something
            result = self.complex_operation()
            self.get_logger().debug(f'Operation result: {result}')
        except ValueError as e:
            self.get_logger().error(f'Invalid value provided: {e}')
        except Exception as e:
            self.get_logger().error(f'Unexpected error in some_method: {e}')
```

## Running Nodes

### From Command Line
```bash
# Run the node directly
python my_node.py

# Or if it's part of a package
ros2 run my_package my_node
```

### With Parameters
```bash
# Run with parameter overrides
python my_node.py --ros-args -p robot_name:=atlas -p control_rate:=100
```

## Common Pitfalls and Solutions

### 1. Forgetting to Initialize ROS
**Problem**: Forgetting `rclpy.init()`
**Solution**: Always call `rclpy.init()` before creating nodes

### 2. Not Properly Cleaning Up
**Problem**: Not calling `destroy_node()` and `shutdown()`
**Solution**: Use try/finally blocks or context managers

### 3. Incorrect Node Names
**Problem**: Using non-unique or invalid node names
**Solution**: Use descriptive, unique names with proper ROS naming conventions

In the next section, we'll explore how to implement publishers and subscribers in Python to create the publish-subscribe communication pattern essential for humanoid robot systems.