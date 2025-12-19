---
sidebar_position: 5
---

# Using Services in Python

## Overview

Services provide a request-response communication pattern in ROS 2, which is essential for synchronous operations in humanoid robot systems. In this section, we'll explore how to implement services in Python using rclpy, covering both service servers and clients.

## Creating Service Servers

### Basic Service Server

Let's start with a basic service server that adds two integers:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {request.a} + {request.b} = {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()

    try:
        rclpy.spin(minimal_service)
    except KeyboardInterrupt:
        minimal_service.get_logger().info('Interrupted, shutting down...')
    finally:
        minimal_service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Server with Custom Message Types

For more complex applications, you'll often need custom service definitions. Here's an example for a humanoid robot service:

First, define the service interface in a `.srv` file (e.g., `GetJointPositions.srv`):
```
# Request
string[] joint_names
---
# Response
float64[] positions
bool success
string message
```

Then implement the server:
```python
import rclpy
from rclpy.node import Node
from your_robot_interfaces.srv import GetJointPositions  # Custom service type

class JointPositionService(Node):
    def __init__(self):
        super().__init__('joint_position_service')

        # Create the service
        self.srv = self.create_service(
            GetJointPositions,
            'get_joint_positions',
            self.get_joint_positions_callback
        )

        # Simulated joint positions (in a real robot, these would come from encoders)
        self.simulated_positions = {
            'head_yaw': 0.0, 'head_pitch': 0.0,
            'l_shoulder_pitch': 0.3, 'l_shoulder_roll': 0.2,
            'r_shoulder_pitch': 0.3, 'r_shoulder_roll': -0.2,
            # ... more joints
        }

        self.get_logger().info('Joint position service started')

    def get_joint_positions_callback(self, request, response):
        try:
            response.positions = []
            response.success = True
            response.message = 'Success'

            for joint_name in request.joint_names:
                if joint_name in self.simulated_positions:
                    response.positions.append(self.simulated_positions[joint_name])
                else:
                    self.get_logger().warn(f'Joint {joint_name} not found')
                    response.success = False
                    response.message = f'Joint {joint_name} not found'
                    response.positions.append(0.0)  # Default value

        except Exception as e:
            self.get_logger().error(f'Service error: {e}')
            response.success = False
            response.message = f'Error: {str(e)}'
            response.positions = [0.0] * len(request.joint_names)

        return response

def main(args=None):
    rclpy.init(args=args)
    node = JointPositionService()

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

## Creating Service Clients

### Basic Service Client

Here's how to create a client that calls the basic add service:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClient(Node):
    def __init__(self):
        super().__init__('minimal_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for the service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        future = self.cli.call_async(self.req)
        return future

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClient()

    # Send request
    future = minimal_client.send_request(1, 2)

    try:
        rclpy.spin_until_future_complete(minimal_client, future)
        response = future.result()
        minimal_client.get_logger().info(f'Result: {response.sum}')
    except Exception as e:
        minimal_client.get_logger().error(f'Service call failed: {e}')
    finally:
        minimal_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Advanced Service Client

Here's a more sophisticated client for the joint position service:

```python
import rclpy
from rclpy.node import Node
from your_robot_interfaces.srv import GetJointPositions
import time

class JointPositionClient(Node):
    def __init__(self):
        super().__init__('joint_position_client')
        self.cli = self.create_client(GetJointPositions, 'get_joint_positions')

        # Wait for service with timeout
        if not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Joint position service not available')
            raise RuntimeError('Service not available')

        self.request = GetJointPositions.Request()

    def get_positions(self, joint_names):
        self.request.joint_names = joint_names
        future = self.cli.call_async(self.request)

        # Wait for response with timeout
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if future.done():
            try:
                response = future.result()
                if response.success:
                    self.get_logger().info(f'Got positions: {response.positions}')
                    return response.positions
                else:
                    self.get_logger().error(f'Service call failed: {response.message}')
                    return None
            except Exception as e:
                self.get_logger().error(f'Service call failed: {e}')
                return None
        else:
            self.get_logger().error('Service call timed out')
            return None

def main(args=None):
    rclpy.init(args=args)
    client = JointPositionClient()

    try:
        # Request positions for specific joints
        joint_names = ['head_yaw', 'l_shoulder_pitch', 'r_shoulder_roll']
        positions = client.get_positions(joint_names)

        if positions is not None:
            for name, pos in zip(joint_names, positions):
                print(f'{name}: {pos}')
    finally:
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Service Patterns in Humanoid Robotics

### Motion Planning Service

```python
import rclpy
from rclpy.node import Node
from your_robot_interfaces.srv import PlanMotion  # Custom service
from geometry_msgs.msg import Pose

class MotionPlannerService(Node):
    def __init__(self):
        super().__init__('motion_planner_service')
        self.srv = self.create_service(
            PlanMotion,
            'plan_motion',
            self.plan_motion_callback
        )

        # Initialize motion planning components
        self.kinematics_solver = self.initialize_kinematics()
        self.collision_checker = self.initialize_collision_checker()

    def plan_motion_callback(self, request, response):
        try:
            # Parse request (start pose, goal pose, constraints)
            start_pose = request.start_pose
            goal_pose = request.goal_pose

            # Plan motion using kinematics and collision checking
            trajectory = self.compute_trajectory(start_pose, goal_pose)

            if trajectory is not None:
                response.trajectory = trajectory
                response.success = True
                response.message = 'Motion plan successful'
            else:
                response.success = False
                response.message = 'Could not find valid motion plan'

        except Exception as e:
            self.get_logger().error(f'Motion planning error: {e}')
            response.success = False
            response.message = f'Planning failed: {str(e)}'

        return response

    def compute_trajectory(self, start_pose, goal_pose):
        # Implementation of motion planning algorithm
        # This would use kinematics, collision checking, etc.
        pass

    def initialize_kinematics(self):
        # Initialize kinematics solver
        pass

    def initialize_collision_checker(self):
        # Initialize collision detection
        pass

def main(args=None):
    rclpy.init(args=args)
    node = MotionPlannerService()

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

### Configuration Service

```python
import rclpy
from rclpy.node import Node
from your_robot_interfaces.srv import SetConfiguration
import json

class ConfigurationService(Node):
    def __init__(self):
        super().__init__('configuration_service')
        self.srv = self.create_service(
            SetConfiguration,
            'set_configuration',
            self.set_configuration_callback
        )

        # Load default configuration
        self.config = self.load_default_configuration()

    def set_configuration_callback(self, request, response):
        try:
            # Parse configuration parameters
            new_config = json.loads(request.config_json)

            # Validate configuration
            if self.validate_configuration(new_config):
                # Apply configuration
                self.config.update(new_config)
                self.save_configuration()

                response.success = True
                response.message = 'Configuration updated successfully'
            else:
                response.success = False
                response.message = 'Invalid configuration parameters'

        except json.JSONDecodeError as e:
            response.success = False
            response.message = f'Invalid JSON in configuration: {str(e)}'
        except Exception as e:
            self.get_logger().error(f'Configuration error: {e}')
            response.success = False
            response.message = f'Configuration failed: {str(e)}'

        return response

    def validate_configuration(self, config):
        # Validate configuration parameters
        required_keys = ['control_rate', 'safety_limits', 'joint_ranges']
        return all(key in config for key in required_keys)

    def load_default_configuration(self):
        # Load default configuration from file or parameters
        return {
            'control_rate': 100,
            'safety_limits': {'max_velocity': 1.0, 'max_torque': 100.0},
            'joint_ranges': {}
        }

    def save_configuration(self):
        # Save current configuration to persistent storage
        pass

def main(args=None):
    rclpy.init(args=args)
    node = ConfigurationService()

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

## Advanced Service Features

### Service with Timeout and Retry Logic

```python
import rclpy
from rclpy.node import Node
from your_robot_interfaces.srv import ExecuteAction
from rclpy.qos import QoSProfile
import time

class RobustServiceClient(Node):
    def __init__(self):
        super().__init__('robust_service_client')
        self.cli = self.create_client(ExecuteAction, 'execute_action')

        if not self.cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Service not available after waiting')
            raise RuntimeError('Service not available')

        self.request = ExecuteAction.Request()

    def call_with_retry(self, action, max_retries=3, timeout=5.0):
        for attempt in range(max_retries):
            try:
                self.request.action = action
                future = self.cli.call_async(self.request)

                rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)

                if future.done():
                    response = future.result()
                    if response.success:
                        self.get_logger().info(f'Action completed successfully on attempt {attempt + 1}')
                        return response
                    else:
                        self.get_logger().warn(f'Action failed: {response.message}, attempt {attempt + 1}')
                else:
                    self.get_logger().warn(f'Timeout on attempt {attempt + 1}')

            except Exception as e:
                self.get_logger().error(f'Exception on attempt {attempt + 1}: {e}')

            if attempt < max_retries - 1:
                time.sleep(1.0)  # Wait before retry

        self.get_logger().error(f'Action failed after {max_retries} attempts')
        return None

def main(args=None):
    rclpy.init(args=args)
    client = RobustServiceClient()

    try:
        response = client.call_with_retry('move_to_home_position')
        if response:
            print(f'Success: {response.message}')
        else:
            print('Action failed after all retries')
    finally:
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Best Practices for Services

### 1. Error Handling and Validation

```python
def robust_service_callback(self, request, response):
    try:
        # Validate input parameters
        if not self.validate_request(request):
            response.success = False
            response.message = 'Invalid request parameters'
            return response

        # Perform the service operation
        result = self.perform_operation(request)

        # Set response
        response.result = result
        response.success = True
        response.message = 'Operation completed successfully'

    except ValueError as e:
        # Specific validation errors
        response.success = False
        response.message = f'Validation error: {str(e)}'
    except RuntimeError as e:
        # Runtime errors
        response.success = False
        response.message = f'Runtime error: {str(e)}'
    except Exception as e:
        # Unexpected errors
        self.get_logger().error(f'Unexpected error: {e}')
        response.success = False
        response.message = f'Unexpected error: {str(e)}'

    return response
```

### 2. Service Threading and Concurrency

```python
import threading
from concurrent.futures import ThreadPoolExecutor

class ThreadedService(Node):
    def __init__(self):
        super().__init__('threaded_service')
        self.srv = self.create_service(
            LongRunningTask,
            'long_running_task',
            self.long_running_task_callback
        )
        self.executor = ThreadPoolExecutor(max_workers=4)

    def long_running_task_callback(self, request, response):
        # Submit long-running task to thread pool
        future = self.executor.submit(self.execute_long_task, request)

        # Wait for completion (this blocks the service callback)
        try:
            result = future.result(timeout=30.0)  # 30 second timeout
            response.result = result
            response.success = True
        except TimeoutError:
            response.success = False
            response.message = 'Task timed out'
        except Exception as e:
            response.success = False
            response.message = f'Task failed: {str(e)}'

        return response

    def execute_long_task(self, request):
        # Execute the actual long-running task in a separate thread
        pass
```

### 3. Service Monitoring and Diagnostics

```python
from rcl_interfaces.msg import ParameterDescriptor
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

class MonitoredService(Node):
    def __init__(self):
        super().__init__('monitored_service')

        # Create service
        self.srv = self.create_service(ServiceType, 'service_name', self.service_callback)

        # Create diagnostic publisher
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        # Track service statistics
        self.service_calls = 0
        self.service_errors = 0
        self.start_time = self.get_clock().now()

    def service_callback(self, request, response):
        self.service_calls += 1
        start_time = self.get_clock().now()

        try:
            # Execute service logic
            response = self.execute_service_logic(request)
            execution_time = (self.get_clock().now() - start_time).nanoseconds / 1e9

            # Publish diagnostics periodically
            if self.service_calls % 10 == 0:  # Every 10 calls
                self.publish_diagnostics(execution_time)

        except Exception as e:
            self.service_errors += 1
            self.get_logger().error(f'Service error: {e}')
            raise

        return response

    def publish_diagnostics(self, execution_time):
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()

        status = DiagnosticStatus()
        status.name = f'{self.get_name()}.service_performance'
        status.level = DiagnosticStatus.OK
        status.message = 'Service operating normally'

        status.values.extend([
            {'key': 'service_calls', 'value': str(self.service_calls)},
            {'key': 'service_errors', 'value': str(self.service_errors)},
            {'key': 'avg_execution_time', 'value': f'{execution_time:.3f}s'}
        ])

        diag_array.status.append(status)
        self.diag_pub.publish(diag_array)
```

## Common Pitfalls and Solutions

### 1. Service Not Available
**Problem**: Client cannot find the service
**Solutions**:
- Ensure the service server is running before the client calls it
- Use `wait_for_service()` with appropriate timeout
- Check service names match exactly

### 2. Blocking Service Callbacks
**Problem**: Long-running service callbacks block other requests
**Solutions**:
- Move heavy computation to separate threads
- Use async patterns where appropriate
- Implement timeouts to prevent indefinite blocking

### 3. Exception Handling
**Problem**: Unhandled exceptions crash the service
**Solutions**:
- Wrap service callbacks in try-catch blocks
- Return appropriate error responses
- Log errors for debugging

## When to Use Services vs. Topics

| Use Case | Services | Topics |
|----------|----------|---------|
| Configuration changes | ✓ | |
| Request-response patterns | ✓ | |
| Synchronous operations | ✓ | |
| Continuous data streams | | ✓ |
| Event notifications | | ✓ |
| Real-time control | | ✓ |

In the next section, we'll implement the agent → controller → actuator loop pattern that connects AI decision-making to physical robot actions.