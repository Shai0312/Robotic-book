---
sidebar_position: 6
---

# Agent → Controller → Actuator Loop Implementation

## Overview

The agent → controller → actuator loop is a fundamental pattern in robotics that connects AI decision-making to physical robot actions. In this section, we'll implement this complete pipeline using ROS 2 and Python, showing how AI agents can control humanoid robots through the ROS 2 middleware.

## Understanding the Loop

### Components of the Loop

The agent → controller → actuator loop consists of three main components:

1. **Agent**: Makes high-level decisions based on goals and sensory input
2. **Controller**: Translates high-level commands into low-level actuator commands
3. **Actuator**: Executes the physical commands on the robot

### Data Flow in the Loop

```
Environment → Sensors → Agent → Controller → Actuators → Environment
     ↑                                            ↓
     ← ← ← ← ← ← ← ← ← ← ← ← ← ← ← ← ← ← ← ← ← ← ← ← ←
```

## Implementing the Agent Component

### Basic AI Agent Node

Let's start with a simple AI agent that makes decisions based on sensor input:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import numpy as np

class SimpleAIAgent(Node):
    def __init__(self):
        super().__init__('simple_ai_agent')

        # Publishers for sending commands to controller
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joint_cmd_publisher = self.create_publisher(JointState, '/joint_commands', 10)

        # Subscribers for sensor data
        self.joint_state_subscriber = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )

        # Timer for decision making loop
        self.timer = self.create_timer(0.1, self.decision_making_loop)  # 10Hz

        # Internal state
        self.current_joint_states = None
        self.current_goal = None
        self.agent_state = 'idle'  # idle, walking, balancing, etc.

        self.get_logger().info('Simple AI Agent initialized')

    def joint_state_callback(self, msg):
        """Update internal joint state from sensor data"""
        self.current_joint_states = msg

    def decision_making_loop(self):
        """Main decision-making loop of the AI agent"""
        if self.current_joint_states is None:
            return

        # Example decision logic
        if self.agent_state == 'idle':
            self.handle_idle_state()
        elif self.agent_state == 'walking':
            self.handle_walking_state()
        elif self.agent_state == 'balancing':
            self.handle_balancing_state()

    def handle_idle_state(self):
        """Handle idle state behavior"""
        # Check if we should transition to walking
        if self.should_start_walking():
            self.agent_state = 'walking'
            self.get_logger().info('Transitioning to walking state')

    def should_start_walking(self):
        """Example condition for starting to walk"""
        # In a real system, this would check for goals, obstacles, etc.
        return self.get_clock().now().nanoseconds % 10000000000 < 1000000000  # Every 10 seconds

    def handle_walking_state(self):
        """Handle walking state - send velocity commands"""
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.2  # Move forward at 0.2 m/s
        cmd_vel.angular.z = 0.0  # No rotation

        self.cmd_vel_publisher.publish(cmd_vel)

        # Check if we should transition back to idle
        if self.should_stop_walking():
            self.agent_state = 'idle'
            self.get_logger().info('Transitioning to idle state')

    def should_stop_walking(self):
        """Example condition for stopping walking"""
        return self.get_clock().now().nanoseconds % 10000000000 > 5000000000  # After 5 seconds

    def handle_balancing_state(self):
        """Handle balancing state - send joint commands"""
        if self.current_joint_states is None:
            return

        # Example: Adjust joint positions to maintain balance
        joint_cmd = JointState()
        joint_cmd.header.stamp = self.get_clock().now().to_msg()
        joint_cmd.name = self.current_joint_states.name.copy()
        joint_cmd.position = []

        for pos in self.current_joint_states.position:
            # Apply small adjustments to maintain balance
            adjusted_pos = pos + np.random.uniform(-0.01, 0.01)  # Small random adjustments
            joint_cmd.position.append(adjusted_pos)

        self.joint_cmd_publisher.publish(joint_cmd)

def main(args=None):
    rclpy.init(args=args)
    agent = SimpleAIAgent()

    try:
        rclpy.spin(agent)
    except KeyboardInterrupt:
        agent.get_logger().info('Interrupted, shutting down...')
    finally:
        agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Advanced AI Agent with Machine Learning

Here's a more sophisticated agent that uses machine learning for decision making:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image, LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float32MultiArray
import numpy as np
import pickle  # For loading pre-trained models (simplified example)

class MLAgent(Node):
    def __init__(self):
        super().__init__('ml_agent')

        # Publishers
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_publisher = self.create_publisher(PoseStamped, '/move_base_simple/goal', 10)

        # Subscribers
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.camera_sub = self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 10)

        # Timer for AI processing
        self.timer = self.create_timer(0.2, self.ai_processing_loop)  # 5Hz

        # ML model (simplified - in practice, you'd load a trained model)
        self.ml_model = self.load_model()

        # Internal state
        self.sensory_data = {
            'joints': None,
            'laser': None,
            'camera': None
        }

        self.get_logger().info('ML Agent initialized')

    def load_model(self):
        """Load pre-trained ML model (simplified example)"""
        # In practice, you'd load a real ML model here
        # For example: TensorFlow model, PyTorch model, etc.
        return {
            'weights': np.random.random((10, 5)),  # Example weights
            'bias': np.random.random(5)  # Example bias
        }

    def joint_callback(self, msg):
        self.sensory_data['joints'] = msg

    def laser_callback(self, msg):
        self.sensory_data['laser'] = msg

    def camera_callback(self, msg):
        self.sensory_data['camera'] = msg

    def ai_processing_loop(self):
        """Main AI processing loop"""
        if not all(self.sensory_data.values()):
            return  # Wait for all sensors to be initialized

        try:
            # Preprocess sensor data
            state_vector = self.preprocess_sensors()

            # Make decision using ML model
            action = self.decide_action(state_vector)

            # Execute action
            self.execute_action(action)

        except Exception as e:
            self.get_logger().error(f'AI processing error: {e}')

    def preprocess_sensors(self):
        """Convert sensor data to state vector for ML model"""
        joint_data = self.sensory_data['joints']
        laser_data = self.sensory_data['laser']

        # Extract relevant features
        joint_positions = np.array(joint_data.position[:10])  # First 10 joints
        laser_ranges = np.array(laser_data.ranges[::10])  # Every 10th laser reading

        # Normalize and combine features
        state = np.concatenate([
            joint_positions / np.pi,  # Normalize joint positions
            laser_ranges / 10.0,      # Normalize laser ranges
        ])

        return state

    def decide_action(self, state_vector):
        """Use ML model to decide on action"""
        # Simplified linear model example
        weights = self.ml_model['weights']
        bias = self.ml_model['bias']

        # Compute action (in practice, this would be more complex)
        action = np.dot(state_vector, weights) + bias
        action = np.tanh(action)  # Apply activation function

        return action

    def execute_action(self, action):
        """Execute the decided action"""
        cmd = Twist()
        cmd.linear.x = float(action[0]) * 0.5  # Scale to reasonable velocity
        cmd.angular.z = float(action[1]) * 0.5  # Scale to reasonable angular velocity

        self.cmd_publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    agent = MLAgent()

    try:
        rclpy.spin(agent)
    except KeyboardInterrupt:
        agent.get_logger().info('Interrupted, shutting down...')
    finally:
        agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Implementing the Controller Component

### Joint Trajectory Controller

The controller receives high-level commands and generates low-level actuator commands:

```python
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
from scipy.interpolate import interp1d

class JointTrajectoryController(Node):
    def __init__(self):
        super().__init__('joint_trajectory_controller')

        # Publishers for sending commands to actuators
        self.joint_cmd_publisher = self.create_publisher(
            JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10
        )

        # Subscribers for desired commands and current state
        self.command_sub = self.create_subscription(
            JointTrajectory, '/joint_commands', self.command_callback, 10
        )
        self.state_sub = self.create_subscription(
            JointState, '/joint_states', self.state_callback, 10
        )

        # Timer for control loop
        self.control_timer = self.create_timer(0.01, self.control_loop)  # 100Hz

        # Internal state
        self.current_trajectory = None
        self.current_joint_state = None
        self.trajectory_index = 0
        self.control_gains = {
            'kp': 100.0,  # Proportional gain
            'ki': 0.1,    # Integral gain
            'kd': 10.0    # Derivative gain
        }

        self.get_logger().info('Joint Trajectory Controller initialized')

    def command_callback(self, msg):
        """Receive trajectory commands from agent"""
        self.current_trajectory = msg
        self.trajectory_index = 0
        self.get_logger().info(f'Received trajectory with {len(msg.points)} points')

    def state_callback(self, msg):
        """Update current joint state"""
        self.current_joint_state = msg

    def control_loop(self):
        """Main control loop"""
        if self.current_trajectory is None or self.current_joint_state is None:
            return

        if self.trajectory_index >= len(self.current_trajectory.points):
            # Trajectory completed
            return

        # Get desired state from trajectory
        desired_point = self.current_trajectory.points[self.trajectory_index]
        desired_positions = desired_point.positions

        # Get current state
        current_positions = self.current_joint_state.position
        current_velocities = self.current_joint_state.velocity

        # Compute control commands using PID
        control_commands = self.compute_pid_control(
            desired_positions, current_positions, current_velocities
        )

        # Publish control commands
        self.publish_control_commands(control_commands)

        # Advance trajectory index
        self.trajectory_index += 1

    def compute_pid_control(self, desired_pos, current_pos, current_vel):
        """Compute PID control commands"""
        if len(desired_pos) != len(current_pos):
            self.get_logger().error('Position vector size mismatch')
            return [0.0] * len(desired_pos)

        commands = []
        for i in range(len(desired_pos)):
            error = desired_pos[i] - current_pos[i]
            error_derivative = 0.0 if i >= len(current_vel) else -current_vel[i]

            command = (
                self.control_gains['kp'] * error +
                self.control_gains['kd'] * error_derivative
            )
            commands.append(command)

        return commands

    def publish_control_commands(self, commands):
        """Publish control commands to actuators"""
        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = self.current_joint_state.name

        point = JointTrajectoryPoint()
        point.positions = commands
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 10000000  # 10ms

        trajectory.points = [point]
        self.joint_cmd_publisher.publish(trajectory)

def main(args=None):
    rclpy.init(args=args)
    controller = JointTrajectoryController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Interrupted, shutting down...')
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Balance Controller for Humanoid Robots

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Vector3, WrenchStamped
import numpy as np

class BalanceController(Node):
    def __init__(self):
        super().__init__('balance_controller')

        # Publishers
        self.joint_cmd_publisher = self.create_publisher(JointState, '/joint_commands', 10)

        # Subscribers
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)

        # Timer for balance control
        self.control_timer = self.create_timer(0.005, self.balance_control_loop)  # 200Hz

        # Balance control parameters
        self.balance_gains = {
            'kp': 50.0,   # Position gain
            'kd': 10.0,   # Damping gain
            'com_kp': 100.0,  # Center of mass gain
        }

        # Internal state
        self.current_imu = None
        self.current_joints = None
        self.desired_com_position = np.array([0.0, 0.0, 0.8])  # Desired CoM height and position

        self.get_logger().info('Balance Controller initialized')

    def imu_callback(self, msg):
        """Update IMU data"""
        self.current_imu = msg

    def joint_callback(self, msg):
        """Update joint state"""
        self.current_joints = msg

    def balance_control_loop(self):
        """Main balance control loop"""
        if self.current_imu is None or self.current_joints is None:
            return

        try:
            # Calculate current center of mass
            current_com = self.calculate_current_com()

            # Compute balance correction
            balance_correction = self.compute_balance_correction(current_com)

            # Apply corrections to joint positions
            corrected_joints = self.apply_balance_correction(balance_correction)

            # Publish corrected joint commands
            self.joint_cmd_publisher.publish(corrected_joints)

        except Exception as e:
            self.get_logger().error(f'Balance control error: {e}')

    def calculate_current_com(self):
        """Calculate current center of mass (simplified)"""
        # In a real system, this would use a full kinematic model
        # This is a simplified example
        if self.current_imu is not None:
            # Use IMU orientation as a proxy for CoM estimation
            orientation = self.current_imu.orientation
            # Simplified calculation - in reality this would use full kinematics
            return np.array([0.0, 0.0, 0.8])
        return self.desired_com_position

    def compute_balance_correction(self, current_com):
        """Compute balance correction based on CoM error"""
        com_error = self.desired_com_position - current_com
        correction = self.balance_gains['com_kp'] * com_error
        return correction

    def apply_balance_correction(self, correction):
        """Apply balance correction to joint positions"""
        if self.current_joints is None:
            return JointState()

        corrected_joints = JointState()
        corrected_joints.header.stamp = self.get_clock().now().to_msg()
        corrected_joints.name = self.current_joints.name.copy()
        corrected_joints.position = []

        for i, (name, pos) in enumerate(zip(self.current_joints.name, self.current_joints.position)):
            # Apply different corrections based on joint type
            if 'ankle' in name or 'hip' in name:
                # Apply balance correction to leg joints
                corrected_pos = pos + correction[1] * 0.01  # Lateral correction
            elif 'torso' in name or 'waist' in name:
                # Apply posture correction
                corrected_pos = pos + correction[2] * 0.005  # Height correction
            else:
                corrected_pos = pos  # No correction for other joints

            corrected_joints.position.append(corrected_pos)

        return corrected_joints

def main(args=None):
    rclpy.init(args=args)
    controller = BalanceController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Interrupted, shutting down...')
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Implementing the Actuator Component

### Actuator Interface Node

The actuator component handles the low-level hardware interface:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState
import numpy as np

class ActuatorInterface(Node):
    def __init__(self):
        super().__init__('actuator_interface')

        # Publishers for hardware interface (simulated)
        self.hardware_cmd_publisher = self.create_publisher(Float64MultiArray, '/hardware_commands', 10)
        self.joint_state_publisher = self.create_publisher(JointState, '/joint_states', 10)

        # Subscribers for control commands
        self.joint_cmd_sub = self.create_subscription(
            JointState, '/joint_commands', self.joint_command_callback, 10
        )

        # Timer for hardware simulation
        self.hardware_timer = self.create_timer(0.001, self.hardware_simulation_loop)  # 1kHz

        # Simulated hardware state
        self.target_positions = {}
        self.current_positions = {}
        self.current_velocities = {}
        self.current_efforts = {}

        # Initialize joint states
        self.joint_names = [
            'head_yaw', 'head_pitch',
            'l_shoulder_pitch', 'l_shoulder_roll', 'l_elbow_yaw', 'l_elbow_roll',
            'r_shoulder_pitch', 'r_shoulder_roll', 'r_elbow_yaw', 'r_elbow_roll',
            'l_hip_yaw_pitch', 'l_hip_roll', 'l_hip_pitch', 'l_knee_pitch', 'l_ankle_pitch', 'l_ankle_roll',
            'r_hip_yaw_pitch', 'r_hip_roll', 'r_hip_pitch', 'r_knee_pitch', 'r_ankle_pitch', 'r_ankle_roll'
        ]

        for name in self.joint_names:
            self.target_positions[name] = 0.0
            self.current_positions[name] = 0.0
            self.current_velocities[name] = 0.0
            self.current_efforts[name] = 0.0

        self.get_logger().info('Actuator Interface initialized')

    def joint_command_callback(self, msg):
        """Receive joint commands from controller"""
        for name, pos in zip(msg.name, msg.position):
            if name in self.target_positions:
                self.target_positions[name] = pos

    def hardware_simulation_loop(self):
        """Simulate hardware behavior and update joint states"""
        for name in self.joint_names:
            # Simple first-order dynamics simulation
            pos_error = self.target_positions[name] - self.current_positions[name]

            # Apply proportional control to simulate actuator response
            velocity_change = pos_error * 10.0  # Gain for response speed
            self.current_velocities[name] = velocity_change

            # Update position based on velocity
            dt = 0.001  # 1kHz timer
            self.current_positions[name] += self.current_velocities[name] * dt

            # Add some damping and simulate effort
            self.current_efforts[name] = pos_error * 50.0 + self.current_velocities[name] * 5.0

        # Publish updated joint states
        self.publish_joint_states()

    def publish_joint_states(self):
        """Publish current joint states"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        msg.name = self.joint_names
        msg.position = [self.current_positions[name] for name in self.joint_names]
        msg.velocity = [self.current_velocities[name] for name in self.joint_names]
        msg.effort = [self.current_efforts[name] for name in self.joint_names]

        self.joint_state_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    actuator = ActuatorInterface()

    try:
        rclpy.spin(actuator)
    except KeyboardInterrupt:
        actuator.get_logger().info('Interrupted, shutting down...')
    finally:
        actuator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Complete Integration Example

### Full Agent-Controller-Actuator System

Here's a complete example that ties all components together:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np

class IntegratedRobotSystem(Node):
    def __init__(self):
        super().__init__('integrated_robot_system')

        # Agent components
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Controller components
        self.joint_cmd_pub = self.create_publisher(JointState, '/joint_commands', 10)
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        # Timer for the complete loop
        self.system_timer = self.create_timer(0.05, self.system_loop)  # 20Hz

        # Internal state
        self.desired_velocity = Twist()
        self.current_joint_state = None
        self.walk_pattern_generator = WalkPatternGenerator()

        self.get_logger().info('Integrated Robot System initialized')

    def cmd_vel_callback(self, msg):
        """Receive velocity commands from agent"""
        self.desired_velocity = msg

    def joint_state_callback(self, msg):
        """Receive current joint states"""
        self.current_joint_state = msg

    def system_loop(self):
        """Complete agent-controller-actuator loop"""
        if self.current_joint_state is None:
            return

        # Agent: Process desired velocity and generate walking pattern
        desired_joint_trajectory = self.walk_pattern_generator.generate_step(
            self.desired_velocity,
            self.current_joint_state
        )

        # Controller: Convert trajectory to joint commands
        joint_commands = self.generate_joint_commands(desired_joint_trajectory)

        # Publish commands to actuators
        self.joint_cmd_pub.publish(joint_commands)

    def generate_joint_commands(self, trajectory):
        """Generate joint commands from trajectory"""
        commands = JointState()
        commands.header.stamp = self.get_clock().now().to_msg()
        commands.name = trajectory.joint_names
        commands.position = [pt.positions[0] for pt in trajectory.points[:len(trajectory.joint_names)]]

        return commands

class WalkPatternGenerator:
    """Generates walking patterns for humanoid robots"""

    def __init__(self):
        self.step_length = 0.1  # 10cm per step
        self.step_height = 0.02  # 2cm step height
        self.step_duration = 0.5  # 0.5 seconds per step
        self.phase = 0.0

    def generate_step(self, cmd_vel, current_state):
        """Generate a walking step based on velocity command"""
        # Calculate step parameters based on desired velocity
        step_length = self.step_length * cmd_vel.linear.x
        step_direction = np.arctan2(cmd_vel.angular.z, cmd_vel.linear.x) if cmd_vel.linear.x != 0 else cmd_vel.angular.z

        # Generate joint trajectory for the step
        trajectory = JointTrajectory()
        trajectory.joint_names = current_state.name

        # Create trajectory points
        for t in np.linspace(0, self.step_duration, 10):  # 10 points per step
            point = JointTrajectoryPoint()

            # Calculate desired positions for this time step
            for i, current_pos in enumerate(current_state.position):
                # Apply walking gait pattern (simplified)
                if 'ankle' in current_state.name[i] or 'hip' in current_state.name[i]:
                    # Add walking motion
                    walking_offset = step_length * np.sin(2 * np.pi * t / self.step_duration)
                    desired_pos = current_pos + walking_offset
                else:
                    desired_pos = current_pos

                point.positions.append(desired_pos)

            point.time_from_start.sec = int(t)
            point.time_from_start.nanosec = int((t - int(t)) * 1e9)
            trajectory.points.append(point)

        return trajectory

def main(args=None):
    rclpy.init(args=args)
    system = IntegratedRobotSystem()

    try:
        rclpy.spin(system)
    except KeyboardInterrupt:
        system.get_logger().info('Interrupted, shutting down...')
    finally:
        system.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Best Practices for Agent-Controller-Actuator Implementation

### 1. Modularity and Separation of Concerns

```python
class ModularSystem(Node):
    def __init__(self):
        super().__init__('modular_system')

        # Separate modules for each component
        self.agent = AIAgent(self)
        self.controller = MotionController(self)
        self.actuator = HardwareInterface(self)

        # Main coordination loop
        self.coordination_timer = self.create_timer(0.02, self.coordination_loop)

    def coordination_loop(self):
        """Coordinate between modules"""
        # Agent makes decisions
        agent_output = self.agent.process()

        # Controller translates to hardware commands
        controller_output = self.controller.process(agent_output)

        # Actuator executes commands
        self.actuator.execute(controller_output)
```

### 2. Safety and Error Handling

```python
def safe_control_loop(self):
    """Safety-aware control loop"""
    try:
        # Check safety conditions
        if self.is_safe_to_operate():
            # Execute normal control
            self.normal_control()
        else:
            # Execute safety behavior
            self.safety_stop()
    except Exception as e:
        self.get_logger().error(f'Control error: {e}')
        self.emergency_stop()

def is_safe_to_operate(self):
    """Check if robot is in safe state"""
    # Check joint limits, temperatures, etc.
    return True
```

### 3. Performance Monitoring

```python
def monitor_performance(self):
    """Monitor control loop performance"""
    current_time = self.get_clock().now()
    loop_time = (current_time - self.last_loop_time).nanoseconds / 1e9

    if loop_time > self.control_period * 1.5:  # 50% tolerance
        self.get_logger().warn(f'Control loop timing violation: {loop_time}s')

    self.last_loop_time = current_time
```

## Common Implementation Patterns

### State Machine Pattern
Use state machines to manage complex behaviors in the agent component.

### Observer Pattern
Use for sensor data distribution to multiple controllers.

### Command Pattern
Use for queuing and executing complex motion sequences.

## Integration Testing

To test the complete loop, create integration tests that:

1. Simulate sensor inputs
2. Verify agent decisions
3. Check controller outputs
4. Validate actuator responses
5. Measure system performance

In the next section, we'll add practical code examples, exercises, and troubleshooting guides to complete Chapter 2 of our educational module.