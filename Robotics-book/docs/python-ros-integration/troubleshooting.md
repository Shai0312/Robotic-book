---
sidebar_position: 9
---

# Troubleshooting: Python-ROS Integration

## Overview

This section addresses common issues encountered when implementing Python-ROS integration for humanoid robots. Each problem includes diagnostic steps, root cause analysis, and solutions.

## Common Python Import Issues

### ModuleNotFoundError: No module named 'rclpy'

**Symptoms**: Python scripts fail with `ModuleNotFoundError` when importing rclpy

**Diagnosis**:
1. Check if ROS 2 is sourced:
   ```bash
   printenv | grep ROS
   ```
2. Verify Python environment:
   ```bash
   python3 -c "import sys; print(sys.path)"
   ```

**Solutions**:
1. Source your ROS 2 installation:
   ```bash
   source /opt/ros/humble/setup.bash  # Replace with your ROS distro
   ```
2. If using a virtual environment, install rclpy:
   ```bash
   pip install rclpy
   ```
3. Check Python version compatibility (ROS 2 requires Python 3.8+)

### Import Issues in Development Environment

**Symptoms**: IDE shows import errors but code runs correctly

**Solutions**:
1. Configure IDE Python interpreter to ROS 2 environment
2. Add ROS 2 site-packages to Python path:
   ```bash
   export PYTHONPATH="${ROS_DISTRO_PATH}/lib/python3.8/site-packages:$PYTHONPATH"
   ```

## Communication Issues

### Nodes Cannot Communicate

**Symptoms**: Publishers and subscribers don't connect; messages aren't received

**Diagnostic Steps**:
1. Check node connections:
   ```bash
   ros2 node list
   ros2 node info <node_name>
   ```
2. Verify topic connections:
   ```bash
   ros2 topic list
   ros2 topic info /topic_name
   ```
3. Check ROS domain ID:
   ```bash
   echo $ROS_DOMAIN_ID
   ```

**Common Causes and Solutions**:
1. **Different ROS domains**: Ensure all nodes use the same domain ID
   ```bash
   export ROS_DOMAIN_ID=0  # Use same value for all nodes
   ```
2. **Network configuration**: For multi-machine setups, set ROS_IP:
   ```bash
   export ROS_IP=192.168.1.100  # Your machine's IP
   export ROS_HOSTNAME=192.168.1.100
   ```
3. **Firewall issues**: Open required ports (11311 and up)

### Quality of Service (QoS) Incompatibility

**Symptoms**: Messages are not received despite nodes being connected

**Diagnosis**:
1. Check QoS profiles:
   ```bash
   ros2 topic info /topic_name -v
   ```

**Solutions**:
1. Ensure publisher and subscriber QoS profiles are compatible:
   ```python
   # Publisher
   qos_profile = rclpy.qos.QoSProfile(depth=10)
   publisher = node.create_publisher(MsgType, 'topic', qos_profile)

   # Subscriber must have compatible profile
   subscriber = node.create_subscription(MsgType, 'topic', callback, qos_profile)
   ```

## Runtime Issues

### Node Fails to Initialize

**Symptoms**: Node creation fails with errors like "Context not initialized"

**Diagnosis**:
1. Check if `rclpy.init()` was called
2. Verify no other initialization issues exist

**Solutions**:
1. Ensure proper initialization sequence:
   ```python
   def main():
       rclpy.init()  # Must be called first
       try:
           node = MyNode()
           try:
               rclpy.spin(node)
           finally:
               node.destroy_node()
       finally:
           rclpy.shutdown()  # Must be called last
   ```

### Timer Issues and Callback Problems

**Symptoms**: Timers don't execute, callbacks don't run, or timing is inconsistent

**Diagnosis**:
1. Check timer creation and callback assignment
2. Verify `rclpy.spin()` is called

**Solutions**:
1. Ensure timer is properly created:
   ```python
   # Correct way
   self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz timer

   def timer_callback(self):
       self.get_logger().info('Timer executed')
   ```
2. Make sure callbacks are properly defined and not blocking

## Memory and Performance Issues

### High Memory Usage

**Symptoms**: Process memory grows over time, system becomes sluggish

**Common Causes**:
1. Message accumulation in queues
2. Circular references preventing garbage collection
3. Large message types being processed frequently

**Solutions**:
1. Limit queue sizes:
   ```python
   publisher = node.create_publisher(MsgType, 'topic', 1)  # Small queue
   ```
2. Use appropriate QoS for large messages:
   ```python
   qos = rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.DurabilityPolicy.VOLATILE)
   ```
3. Process large messages asynchronously to prevent blocking

### Performance Bottlenecks

**Symptoms**: Low message rates, high latency, missed deadlines

**Diagnostic Tools**:
1. Monitor node performance:
   ```bash
   ros2 run top top
   ros2 topic hz /topic_name
   ```
2. Check system resources:
   ```bash
   htop
   iostat -x 1
   ```

**Solutions**:
1. Optimize callback execution time
2. Use threading for heavy computations:
   ```python
   import threading
   from concurrent.futures import ThreadPoolExecutor

   class OptimizedNode(Node):
       def __init__(self):
           super().__init__('optimized_node')
           self.executor = ThreadPoolExecutor(max_workers=2)

       def heavy_computation_callback(self, msg):
           # Submit heavy work to thread pool
           future = self.executor.submit(self.process_data, msg)
   ```

## Service-Specific Issues

### Service Calls Timeout

**Symptoms**: Service clients receive timeout errors

**Diagnostic Steps**:
1. Verify service server is running:
   ```bash
   ros2 service list
   ros2 service info /service_name
   ```
2. Check service server status

**Solutions**:
1. Ensure service server is properly initialized:
   ```python
   # Server side
   self.srv = self.create_service(SrvType, 'service_name', self.service_callback)

   # Client side - use appropriate timeout
   while not self.cli.wait_for_service(timeout_sec=1.0):
       self.get_logger().info('Waiting for service...')
   ```

### Service Callback Blocking

**Symptoms**: Service server becomes unresponsive to new requests

**Solutions**:
1. Avoid long-running operations in service callbacks
2. Use threading for heavy computations:
   ```python
   def service_callback(self, request, response):
       # For long operations, consider using a separate thread or deferring work
       result = self.compute_result(request)  # Keep this fast
       return response
   ```

## Agent-Controller-Actuator Loop Issues

### Timing Problems in Control Loop

**Symptoms**: Robot behavior is unstable, control commands are delayed, or system oscillates

**Diagnostic Steps**:
1. Measure actual loop frequency:
   ```python
   import time
   loop_start = time.time()
   # ... loop body ...
   loop_time = time.time() - loop_start
   self.get_logger().info(f'Loop time: {loop_time}s')
   ```

**Solutions**:
1. Ensure consistent control rate:
   ```python
   # Use appropriate timer frequency
   self.control_timer = self.create_timer(0.01, self.control_loop)  # 100Hz for control
   ```
2. Implement rate limiting if needed:
   ```python
   from rclpy.clock import Clock
   from rclpy.duration import Duration

   class RateLimitedNode(Node):
       def __init__(self):
           super().__init__('rate_limited_node')
           self.rate = 100.0  # Hz
           self.last_time = self.get_clock().now()

       def rate_limit(self):
           current_time = self.get_clock().now()
           expected_duration = Duration(nanoseconds=int(1e9 / self.rate))
           if (current_time - self.last_time) >= expected_duration:
               self.last_time = current_time
               return True
           return False
   ```

### State Synchronization Issues

**Symptoms**: Control decisions are based on outdated sensor data, commands are applied to wrong states

**Solutions**:
1. Implement proper message timestamps:
   ```python
   # In publisher
   msg.header.stamp = self.get_clock().now().to_msg()

   # In subscriber, check message age
   def sensor_callback(self, msg):
       msg_time = Time.from_msg(msg.header.stamp)
       current_time = self.get_clock().now()
       age = (current_time - msg_time).nanoseconds / 1e9
       if age < 0.1:  # Accept messages less than 100ms old
           self.process_sensor_data(msg)
   ```

## Debugging Strategies

### Effective Logging

**Best Practices**:
1. Use appropriate log levels:
   ```python
   self.get_logger().debug('Detailed debugging info')
   self.get_logger().info('Normal operation info')
   self.get_logger().warn('Potential issues')
   self.get_logger().error('Errors that allow continued operation')
   self.get_logger().fatal('Critical errors requiring shutdown')
   ```

2. Include context in log messages:
   ```python
   self.get_logger().info(f'Joint {joint_name} position: {position:.3f}')
   ```

### Using ROS 2 Tools for Debugging

**Essential Commands**:
```bash
# Monitor topics
ros2 topic echo /topic_name
ros2 topic hz /topic_name
ros2 topic bw /topic_name

# Monitor nodes
ros2 node info /node_name
ros2 run demo_nodes_py listener  # Example node

# Monitor services
ros2 service list
ros2 service call /service_name srv_type "{request: values}"

# Overall system view
ros2 doctor  # Check ROS 2 system health
```

### Adding Debug Topics

```python
class DebugNode(Node):
    def __init__(self):
        super().__init__('debug_node')

        # Create debug publishers
        self.debug_pub = self.create_publisher(Float64MultiArray, 'debug_values', 10)
        self.state_pub = self.create_publisher(String, 'debug_state', 10)

    def publish_debug_info(self):
        # Publish internal state for debugging
        debug_msg = Float64MultiArray()
        debug_msg.data = [self.internal_value1, self.internal_value2]
        self.debug_pub.publish(debug_msg)
```

## Hardware Integration Issues

### Joint State Publication Problems

**Symptoms**: Joint states are not published correctly, robot model in RViz doesn't update

**Solutions**:
1. Verify joint names match URDF:
   ```python
   # Joint names must exactly match URDF
   joint_state_msg.name = ['joint1', 'joint2', 'joint3']  # Must match URDF
   joint_state_msg.position = [pos1, pos2, pos3]
   ```

2. Ensure proper timestamps:
   ```python
   joint_state_msg.header.stamp = self.get_clock().now().to_msg()
   ```

### Control Command Issues

**Symptoms**: Commands sent to hardware are ignored or cause unexpected behavior

**Diagnostic Steps**:
1. Verify command message format matches hardware interface
2. Check control loop timing
3. Monitor hardware interface status

**Solutions**:
1. Validate command ranges:
   ```python
   def validate_and_send_command(self, command):
       # Check limits before sending
       if self.is_command_valid(command):
           self.command_publisher.publish(command)
       else:
           self.get_logger().error(f'Invalid command: {command}')
   ```

## Common Humanoid Robot-Specific Issues

### Balance Control Instability

**Symptoms**: Robot falls over, oscillating behavior, control commands diverge

**Solutions**:
1. Tune control gains appropriately
2. Ensure sensor fusion is working correctly
3. Implement safety limits:
   ```python
   def apply_safety_limits(self, command):
       # Limit joint velocities and accelerations
       max_vel = 2.0  # rad/s
       limited_command = np.clip(command, -max_vel, max_vel)
       return limited_command
   ```

### Walking Pattern Generation Problems

**Symptoms**: Robot cannot walk, falls during walking, unnatural gait

**Solutions**:
1. Start with simple, stable walking patterns
2. Verify zero moment point (ZMP) calculation
3. Implement proper footstep planning
4. Add balance feedback control

## Error Recovery and Fault Tolerance

### Graceful Degradation

```python
class FaultTolerantNode(Node):
    def __init__(self):
        super().__init__('fault_tolerant_node')
        self.fallback_mode = False

    def sensor_callback(self, msg):
        if self.is_sensor_data_valid(msg):
            self.normal_operation(msg)
        else:
            self.enable_fallback_mode()

    def enable_fallback_mode(self):
        if not self.fallback_mode:
            self.get_logger().warn('Entering fallback mode')
            self.fallback_mode = True
            # Implement safe behavior
```

### Emergency Stop Implementation

```python
class SafeRobotNode(Node):
    def __init__(self):
        super().__init__('safe_robot_node')

        # Create emergency stop service
        self.emergency_stop_srv = self.create_service(
            Trigger, 'emergency_stop', self.emergency_stop_callback
        )

    def emergency_stop_callback(self, request, response):
        self.get_logger().fatal('EMERGENCY STOP ACTIVATED')
        self.stop_all_motors()
        response.success = True
        response.message = 'Emergency stop executed'
        return response

    def stop_all_motors(self):
        # Send zero commands to all actuators
        pass
```

## Performance Monitoring

### System Health Monitoring

```python
class HealthMonitorNode(Node):
    def __init__(self):
        super().__init__('health_monitor')
        self.heartbeat_timer = self.create_timer(1.0, self.check_system_health)
        self.message_counters = {}

    def check_system_health(self):
        # Monitor message rates, CPU usage, memory, etc.
        if self.is_system_healthy():
            self.get_logger().debug('System is healthy')
        else:
            self.get_logger().error('System health issue detected')
```

## Quick Reference: Common Fixes

| Issue | Command/Solution |
|-------|------------------|
| Nodes not connecting | `export ROS_DOMAIN_ID=0` (same on all machines) |
| Import errors | `source /opt/ros/<distro>/setup.bash` |
| Service timeout | Verify server is running: `ros2 service list` |
| High memory | Reduce queue sizes, check for circular references |
| Timing issues | Use appropriate timer frequencies, check system load |
| Permission errors | Check user permissions for ROS 2 installation |

## Getting Help

When seeking help for issues:

1. **Provide system information**:
   ```bash
   echo "ROS Distro: $(echo $ROS_DISTRO)"
   python3 --version
   lsb_release -a
   ```

2. **Include error messages** exactly as they appear

3. **Provide minimal reproducible example**

4. **Check the logs**:
   ```bash
   ros2 run rclpy_examples listener  # Example that should work
   ```

This troubleshooting guide covers the most common issues encountered in Python-ROS integration for humanoid robots. For additional help, consult the ROS 2 documentation, community forums, and the specific documentation for your robot platform.