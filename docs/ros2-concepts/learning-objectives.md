---
sidebar_position: 7
---

# Chapter 1: Learning Objectives and Exercises

## Learning Objectives

After completing this chapter on ROS 2 Core Concepts, you should be able to:

### Knowledge Objectives
- Define the fundamental ROS 2 concepts: nodes, topics, services, and messages
- Explain the differences between publish-subscribe and request-response communication patterns
- Describe how data flows between components in a humanoid robot system
- Identify appropriate use cases for topics versus services

### Skills Objectives
- Create a simple ROS 2 node in Python
- Implement a publisher-subscriber pair for basic data exchange
- Design a simple service client-server interaction
- Monitor and debug ROS 2 communication using command-line tools

### Application Objectives
- Apply ROS 2 concepts to a humanoid robot control scenario
- Design appropriate topics and services for a robot system
- Analyze data flow requirements for different robotic tasks
- Evaluate quality of service settings for different types of data

## Prerequisites

Before starting this chapter, you should have:
- Basic Python programming knowledge (variables, functions, classes)
- Understanding of fundamental programming concepts
- Familiarity with command-line tools
- Interest in robotics applications

## Exercises

### Exercise 1: Basic Publisher-Subscriber
**Difficulty**: Beginner
**Time**: 30-45 minutes

Create a publisher node that publishes messages containing your name and a counter, and a subscriber node that receives and displays these messages. This exercise will help you understand the basic publish-subscribe pattern.

**Steps**:
1. Create a publisher node that sends String messages to a topic called "my_topic"
2. Create a subscriber node that listens to "my_topic" and logs received messages
3. Run both nodes and observe the communication
4. Experiment with different message rates

### Exercise 2: Simple Service
**Difficulty**: Intermediate
**Time**: 45-60 minutes

Create a service server that performs a simple calculation (e.g., adding two numbers) and a client that calls this service. This exercise will help you understand the request-response pattern.

**Steps**:
1. Create a custom service definition file
2. Implement a service server that performs the calculation
3. Create a client that calls the service with different parameters
4. Test the service and verify the results

### Exercise 3: Humanoid Robot Data Flow Simulation
**Difficulty**: Advanced
**Time**: 60-90 minutes

Design and implement a simplified version of the humanoid robot data flow described in the chapter. Create nodes that simulate sensor data, state estimation, and control commands.

**Steps**:
1. Create a sensor simulator node that publishes joint position data
2. Create a state estimator node that processes the joint data
3. Create a simple controller node that responds to state changes
4. Connect all nodes using appropriate topics
5. Test the system and observe the data flow

## Practical Examples

### Example 1: Publisher Node
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello ROS 2: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1
```

### Example 2: Subscriber Node
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}"')
```

## Troubleshooting Common Issues

### Topic Communication Issues
- **Nodes not seeing each other**: Check that nodes are on the same ROS domain
- **Message not received**: Verify topic names match exactly
- **High latency**: Check network connectivity and system performance

### Service Communication Issues
- **Service not available**: Ensure service server is running before client calls
- **Timeout errors**: Check service implementation and system performance
- **Connection refused**: Verify service name and node status

## Summary

This chapter introduced you to the fundamental concepts of ROS 2 and how they apply to humanoid robot systems. Understanding these concepts is crucial for the next chapters where you'll learn about connecting AI agents to robot control systems and modeling humanoid robots with URDF.