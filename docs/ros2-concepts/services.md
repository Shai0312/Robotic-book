---
sidebar_position: 5
---

# Services

## What are Services?

Services in ROS 2 provide a **request-response** communication pattern between nodes. Unlike topics which use publish-subscribe, services create a direct connection where one node sends a request and waits for a response from another node.

## Service Characteristics

### Synchronous Communication
- The client sends a request and waits for a response
- Communication is blocking until the response is received
- Ensures that the request was processed and a response was received

### Request-Response Pattern
- **Client**: Sends a request and waits for a response
- **Server**: Receives requests, processes them, and sends responses
- **Service**: The named interface that connects clients and servers

### Use Cases
- Actions that require confirmation of completion
- Configuration requests
- Querying the state of a system
- Operations that return a result immediately

## Service Example in Python

### Service Definition (example_srv.srv)
```
# Request
string name
int32 age
---
# Response
bool success
string message
```

### Service Server
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
        self.get_logger().info(f'Incoming request\na: {request.a}, b: {request.b}')
        return response

def main():
    rclpy.init()
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()
```

### Service Client
```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClient(Node):
    def __init__(self):
        super().__init__('minimal_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        future = self.cli.call_async(self.req)
        return future

def main():
    rclpy.init()
    minimal_client = MinimalClient()
    future = minimal_client.send_request(1, 2)

    rclpy.spin_until_future_complete(minimal_client, future)
    response = future.result()
    minimal_client.get_logger().info(f'Result: {response.sum}')

    minimal_client.destroy_node()
    rclpy.shutdown()
```

### Humanoid Robot Service Example

Here's a practical example of a service for controlling a humanoid robot's walking gait:

```python
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from humanoid_msgs.srv import WalkCommand  # Hypothetical message type

class WalkService(Node):
    def __init__(self):
        super().__init__('walk_service')

        # Service for starting/stopping walking
        self.walk_srv = self.create_service(
            Trigger,
            'start_walking',
            self.start_walking_callback
        )

        # Service for sending walk commands
        self.walk_cmd_srv = self.create_service(
            WalkCommand,
            'walk_command',
            self.walk_command_callback
        )

        self.is_walking = False
        self.get_logger().info('Walk service initialized')

    def start_walking_callback(self, request, response):
        if not self.is_walking:
            # Logic to start walking gait
            self.is_walking = True
            response.success = True
            response.message = 'Walking started successfully'
            self.get_logger().info('Started walking gait')
        else:
            response.success = False
            response.message = 'Robot is already walking'

        return response

    def walk_command_callback(self, request, response):
        if self.is_walking:
            # Process walk command (forward, backward, turn, etc.)
            self.get_logger().info(f'Executing walk command: {request.command}')
            response.success = True
            response.message = f'Walk command {request.command} executed'
        else:
            response.success = False
            response.message = 'Cannot execute walk command: robot is not walking'

        return response

def main(args=None):
    rclpy.init(args=args)
    node = WalkService()

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

## Services in Humanoid Robots

Common services in humanoid robot systems include:

- `/humanoid_controller/get_joint_positions`: Get current joint positions
- `/humanoid_controller/set_joint_positions`: Set desired joint positions
- `/robot_state/get_robot_state`: Get overall robot state
- `/motion_planner/plan_path`: Request a motion plan
- `/perception/detect_objects`: Request object detection in sensor data

## Services vs. Topics

| Aspect | Topics | Services |
|--------|--------|----------|
| Communication | Publish-subscribe (async) | Request-response (sync) |
| Data Flow | One-way | Two-way |
| Timing | Continuous | On-demand |
| Guarantees | No delivery guarantee | Request processed, response guaranteed |
| Use Case | Continuous data streams | Action that returns a result |

## Advanced Service Features

### Service Introspection
- Monitor service calls and responses
- Debug service communication
- Measure response times

### Service Namespaces
- Organize services using namespaces
- Prevent naming conflicts
- Enable modular system design

## Exercise: Implement a Humanoid Robot Service

Create a ROS 2 service that allows remote control of a humanoid robot's posture:

1. Define a custom service message for posture control (e.g., `SetPosture.srv` with fields for posture name and duration)
2. Implement a service server that receives posture commands and executes them
3. Implement a service client that sends posture commands
4. Include error handling for invalid postures
5. Add a parameter to control the execution timeout

**Challenge**: Extend the service to support trajectory-based posture transitions with smooth interpolation between poses.