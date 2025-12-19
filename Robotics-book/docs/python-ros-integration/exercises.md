---
sidebar_position: 8
---

# Exercises: Python-ROS Integration

## Overview

This section provides hands-on exercises to reinforce your understanding of Python-ROS integration concepts. Each exercise is designed to build practical skills in implementing the agent → controller → actuator loop for humanoid robots.

## Exercise 1: Basic Publisher-Subscriber Communication

**Difficulty**: Beginner
**Time**: 30-45 minutes
**Objective**: Create a publisher-subscriber pair that simulates robot sensor data

### Instructions

1. Create a publisher node that publishes joint position data for a humanoid robot
2. Create a subscriber node that receives this data and calculates basic statistics
3. Use the `sensor_msgs/JointState` message type
4. Include at least 6 joints representing a simple humanoid model

### Starter Code

```python
# publisher_template.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        # TODO: Create publisher for joint_states topic
        # TODO: Create timer to publish data periodically
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.counter = 0

    def publish_joint_states(self):
        # TODO: Implement this method to publish JointState messages
        pass

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Solution Approach

1. Create a publisher for the `/joint_states` topic
2. Use a timer to publish messages at 50Hz
3. Generate simulated joint positions using sine waves
4. Create subscriber that calculates min, max, and average positions

### Evaluation Criteria

- [ ] Publisher sends valid JointState messages
- [ ] Subscriber receives and processes messages
- [ ] Joint names are correctly specified
- [ ] Messages are published at consistent rate
- [ ] Error handling is implemented

## Exercise 2: Service-Based Robot Control

**Difficulty**: Intermediate
**Time**: 45-60 minutes
**Objective**: Implement a service for controlling robot behavior

### Instructions

1. Create a custom service definition for robot commands
2. Implement a service server that handles different robot states
3. Create a client that sends commands and handles responses
4. Include validation and error handling

### Service Definition Example

Create a file `RobotCommand.srv`:
```
# Request
string command  # 'enable', 'disable', 'reset', 'calibrate'
float64[] parameters  # Additional parameters for the command
---
# Response
bool success
string message
float64[] results  # Results from the command execution
```

### Implementation Tasks

1. Implement the service server with different command handlers
2. Add validation for command parameters
3. Create a client that can send different types of commands
4. Add timeout and retry logic to the client

### Solution Approach

```python
# robot_command_server.py
import rclpy
from rclpy.node import Node
# TODO: Import your custom service type
from example_interfaces.srv import Trigger  # Using example for simplicity

class RobotCommandServer(Node):
    def __init__(self):
        super().__init__('robot_command_server')
        # TODO: Create service server
        self.robot_state = 'disabled'  # disabled, enabled, calibrating

    def handle_command(self, request, response):
        # TODO: Implement command handling logic
        pass

def main(args=None):
    rclpy.init(args=args)
    server = RobotCommandServer()
    rclpy.spin(server)
    server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Evaluation Criteria

- [ ] Service definition is properly created
- [ ] Server handles different commands correctly
- [ ] Client can send commands and receive responses
- [ ] Error handling is implemented
- [ ] Validation prevents invalid commands

## Exercise 3: Implement a Simple Controller

**Difficulty**: Intermediate
**Time**: 60-90 minutes
**Objective**: Create a PID controller for a single joint

### Instructions

1. Create a controller node that subscribes to desired joint positions
2. Subscribe to current joint positions from feedback
3. Implement PID control logic to compute motor commands
4. Publish commands to actuator interface

### System Requirements

- Input: `/joint_position_command` (std_msgs/Float64)
- Feedback: `/joint_position_feedback` (std_msgs/Float64)
- Output: `/joint_motor_command` (std_msgs/Float64)

### Implementation Tasks

1. Implement PID control algorithm with tunable parameters
2. Add safety limits to prevent dangerous commands
3. Include integral windup protection
4. Add derivative kick prevention

### Solution Approach

```python
# joint_controller.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from rcl_interfaces.msg import ParameterDescriptor

class JointController(Node):
    def __init__(self):
        super().__init__('joint_controller')

        # Declare parameters for PID gains
        self.declare_parameter('kp', 10.0, ParameterDescriptor(description='Proportional gain'))
        self.declare_parameter('ki', 0.1, ParameterDescriptor(description='Integral gain'))
        self.declare_parameter('kd', 0.5, ParameterDescriptor(description='Derivative gain'))

        # TODO: Create subscribers and publishers
        # TODO: Initialize PID variables
        self.previous_error = 0.0
        self.integral = 0.0

    def command_callback(self, msg):
        # Store desired position
        pass

    def feedback_callback(self, msg):
        # Calculate control output using PID
        # Publish motor command
        pass

def main(args=None):
    rclpy.init(args=args)
    controller = JointController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Evaluation Criteria

- [ ] PID algorithm is correctly implemented
- [ ] Controller responds to position commands
- [ ] Safety limits prevent dangerous outputs
- [ ] Parameters can be tuned at runtime
- [ ] Integral windup and derivative kick are handled

## Exercise 4: Agent-Controller-Actuator Integration

**Difficulty**: Advanced
**Time**: 90-120 minutes
**Objective**: Implement a complete agent-controller-actuator loop

### Instructions

1. Create an AI agent that makes high-level decisions
2. Implement a controller that translates decisions to commands
3. Create a simulated actuator that executes commands
4. Integrate all components with proper error handling

### System Architecture

```
Environment → Sensors → Agent → Controller → Actuator → Robot → Environment
     ↑                                                       ↓
     ← ← ← ← ← ← ← ← ← ← ← ← ← ← ← ← ← ← ← ← ← ← ← ← ← ← ← ← ← ←
```

### Implementation Tasks

1. Agent: Process sensor data and decide on actions
2. Controller: Convert high-level actions to low-level commands
3. Actuator: Simulate hardware response to commands
4. Integration: Coordinate all components with proper timing

### Solution Approach

```python
# integrated_system.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np

class IntegratedSystem(Node):
    def __init__(self):
        super().__init__('integrated_system')

        # Agent components
        self.sensor_sub = self.create_subscription(
            JointState, 'sensor_data', self.sensor_callback, 10
        )
        self.agent_pub = self.create_publisher(Twist, 'agent_decision', 10)

        # Controller components
        self.decision_sub = self.create_subscription(
            Twist, 'agent_decision', self.decision_callback, 10
        )
        self.command_pub = self.create_publisher(JointState, 'actuator_commands', 10)

        # Actuator feedback
        self.actuator_sub = self.create_subscription(
            JointState, 'actuator_feedback', self.actuator_callback, 10
        )

        # System timer
        self.system_timer = self.create_timer(0.02, self.system_loop)  # 50Hz

        # Internal state
        self.sensor_data = None
        self.actuator_state = None
        self.current_decision = Twist()

    def sensor_callback(self, msg):
        self.sensor_data = msg

    def decision_callback(self, msg):
        self.current_decision = msg

    def actuator_callback(self, msg):
        self.actuator_state = msg

    def system_loop(self):
        # TODO: Implement the complete loop
        # 1. Agent: Process sensor data and make decision
        # 2. Controller: Convert decision to actuator commands
        # 3. Monitor system state
        pass

def main(args=None):
    rclpy.init(args=args)
    system = IntegratedSystem()
    rclpy.spin(system)
    system.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Evaluation Criteria

- [ ] All three components are properly implemented
- [ ] Data flows correctly between components
- [ ] System responds to sensor inputs
- [ ] Error handling is implemented throughout
- [ ] Timing requirements are met

## Exercise 5: Humanoid Robot Walking Pattern Generator

**Difficulty**: Advanced
**Time**: 120-150 minutes
**Objective**: Create a walking pattern generator for humanoid robots

### Instructions

1. Implement a pattern generator that creates walking trajectories
2. Support forward, backward, and turning motions
3. Ensure stable walking patterns with proper foot placement
4. Integrate with joint controllers

### Key Requirements

- Generate trajectories for both legs
- Coordinate arms for balance
- Support variable walking speeds
- Include safety checks for stability

### Implementation Tasks

1. Implement inverse kinematics for leg motion
2. Generate center of mass trajectories
3. Create footstep planning algorithm
4. Add balance feedback control

### Solution Approach

```python
# walking_pattern_generator.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory
import numpy as np
import math

class WalkingPatternGenerator(Node):
    def __init__(self):
        super().__init__('walking_pattern_generator')

        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10
        )
        self.trajectory_pub = self.create_publisher(
            JointTrajectory, 'walking_trajectories', 10
        )

        # Walking parameters
        self.step_length = 0.1  # meters
        self.step_height = 0.02  # meters
        self.step_duration = 0.5  # seconds
        self.zmp_margin = 0.05  # Zero Moment Point safety margin

        self.desired_velocity = Twist()
        self.walk_phase = 0.0

    def cmd_vel_callback(self, msg):
        self.desired_velocity = msg

    def generate_walking_trajectory(self):
        """Generate walking trajectory based on desired velocity"""
        # TODO: Implement walking pattern generation
        # Consider: step timing, foot placement, balance, smooth transitions
        pass

def main(args=None):
    rclpy.init(args=args)
    generator = WalkingPatternGenerator()
    rclpy.spin(generator)
    generator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Evaluation Criteria

- [ ] Walking patterns are dynamically generated
- [ ] Support for different motion types (forward, backward, turning)
- [ ] Stability is maintained during walking
- [ ] Trajectories are smooth and continuous
- [ ] Safety margins are respected

## Exercise 6: Multi-Node Coordination

**Difficulty**: Advanced
**Time**: 90-120 minutes
**Objective**: Coordinate multiple ROS 2 nodes for complex behavior

### Instructions

1. Create multiple specialized nodes for different functions
2. Implement coordination between nodes using topics and services
3. Add state management across the system
4. Include fault tolerance and recovery

### System Design

- Node 1: Perception (processes sensor data)
- Node 2: Planning (generates high-level plans)
- Node 3: Control (executes low-level commands)
- Node 4: Monitoring (tracks system health)

### Implementation Tasks

1. Design message interfaces between nodes
2. Implement state machines for each node
3. Add coordination protocols
4. Include error detection and recovery

### Evaluation Criteria

- [ ] Nodes communicate effectively
- [ ] System state is properly managed
- [ ] Error recovery works correctly
- [ ] Performance requirements are met
- [ ] System is robust to individual node failures

## Exercise 7: Performance Optimization

**Difficulty**: Advanced
**Time**: 60-90 minutes
**Objective**: Optimize a ROS 2 system for performance

### Instructions

1. Profile an existing multi-node system
2. Identify performance bottlenecks
3. Implement optimizations
4. Measure improvement

### Optimization Areas

- Message serialization/deserialization
- Network communication
- Computation efficiency
- Memory management
- Threading and concurrency

### Implementation Tasks

1. Add performance measurement instrumentation
2. Identify and fix timing issues
3. Optimize data structures and algorithms
4. Use appropriate QoS settings

### Evaluation Criteria

- [ ] Performance metrics are measured
- [ ] Bottlenecks are identified
- [ ] Optimizations are implemented
- [ ] Performance improvement is demonstrated
- [ ] System functionality is preserved

## Tips for Success

### General Tips
1. **Start Simple**: Begin with basic functionality before adding complexity
2. **Test Incrementally**: Verify each component works before integrating
3. **Use Logging**: Add logging to understand system behavior
4. **Handle Errors**: Implement proper error handling and recovery
5. **Follow Conventions**: Use ROS 2 naming and coding conventions

### Debugging Tips
1. **Use ROS 2 Tools**: Utilize `ros2 topic echo`, `ros2 node info`, etc.
2. **Check Connections**: Verify publishers and subscribers are connected
3. **Monitor Rates**: Ensure message rates are appropriate
4. **Validate Data**: Check that message contents are valid
5. **Use Simulation**: Test in simulation before hardware

### Best Practices
1. **Resource Management**: Always clean up resources properly
2. **Parameter Configuration**: Use parameters for configurable values
3. **Documentation**: Comment code and document interfaces
4. **Modularity**: Keep components focused and modular
5. **Safety**: Implement safety checks and limits

## Additional Resources

- [ROS 2 Python Developer Guide](https://docs.ros.org/en/humble/How-To-Guides/Using-Python-to-write-a-simple-publisher-and-subscriber.html)
- [rclpy API Documentation](https://docs.ros.org/en/humble/p/rclpy/)
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [Robotics Stack Exchange](https://robotics.stackexchange.com/) for specific questions

Complete these exercises to build a solid foundation in Python-ROS integration for humanoid robot control. Each exercise builds upon the previous ones, so it's recommended to complete them in order.