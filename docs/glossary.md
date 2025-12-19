---
title: "Glossary of ROS 2 and Robotics Terms"
sidebar_position: 100
---

# Glossary of ROS 2 and Robotics Terms

This glossary provides definitions for key terms used throughout the ROS 2 Robot Control Module documentation.

## A

**Action** - A communication pattern in ROS 2 for long-running tasks with feedback and status updates, built on top of services.

**Agent** - An AI system that perceives its environment and takes actions to achieve goals, often used in robotics for decision-making.

## B

**Bag file** - A file format used by ROS to store recorded data from topics, services, and parameters for later analysis.

**Bridge** - Software that connects ROS 2 with other middleware systems or external applications.

## C

**Callback** - A function that is executed when a specific event occurs, such as receiving a message or service request.

**Client** - A node that sends requests to services and receives responses.

**Collision detection** - The computational process of determining when two or more objects in a simulation intersect.

## D

**D-H parameters** - Denavit-Hartenberg parameters used to define the kinematic structure of robotic manipulators.

**Debug node** - A ROS 2 node used for debugging purposes, often for monitoring or modifying system behavior.

## E

**Executor** - In ROS 2, a class that controls how callbacks are invoked for a set of entities (nodes, timers, subscriptions, etc.).

## F

**Frame** - A coordinate system in robotics, used to define positions and orientations relative to a reference point.

## G

**Gazebo** - A 3D simulation environment for robotics that provides high-fidelity physics simulation and rendering.

**Gimbal lock** - A loss of one degree of freedom in a three-dimensional, three-gimbal mechanism that occurs when the axes of two of the three gimbals are driven into a parallel configuration.

## H

**Hardware abstraction layer (HAL)** - Software layer that allows higher-level software to interact with hardware without needing to know hardware-specific details.

**Homing** - The process of moving a robot or mechanism to a known reference position.

## I

**Inertial measurement unit (IMU)** - A sensor that measures specific force, angular rate, and sometimes magnetic fields to sense a body's specific force, angular rate, and sometimes the magnetic field surrounding the body.

**Inverse kinematics** - The mathematical process of calculating the variable joint parameters needed to place the end of a kinematic chain, such as a robot manipulator or animation character, in a given position and orientation.

## J

**Joint** - A connection between two links that allows relative motion between them in a URDF model.

## K

**Kinematics** - The study of motion without considering the forces that cause the motion.

## L

**Launch file** - An XML or Python file that defines how to start multiple nodes and configure parameters in ROS 2.

**Link** - A rigid body in a URDF model that represents a physical component of a robot.

## M

**Middleware** - Software that provides common services and capabilities to applications beyond what's offered by the operating system.

**Message** - Data structures used for communication between nodes in ROS 2 topics.

## N

**Node** - A process that performs computation in ROS. Nodes are the fundamental building blocks of a ROS system.

## O

**Odometry** - The use of data from motion sensors to estimate change in position over time.

## P

**Parameter** - Configuration values in ROS 2 that can be set at runtime and accessed by nodes.

**Point Cloud** - A set of data points in space, typically representing the external surface of an object, used in 3D sensing.

**Publisher** - A node that sends messages to topics in ROS 2.

## Q

**Quaternion** - A mathematical notation for representing orientations and rotations in 3D space, avoiding gimbal lock issues.

## R

**Robot Operating System (ROS)** - A flexible framework for writing robot software that provides services designed for a heterogeneous computer cluster.

**Robot State Publisher** - A ROS package that uses joint position data to publish coordinate transforms for a robot model.

**Rviz** - The 3D visualization tool for ROS that allows users to visualize robot models, sensor data, and other information.

## S

**Service** - A communication pattern in ROS 2 for request-response interactions between nodes.

**Subscriber** - A node that receives messages from topics in ROS 2.

**Simulation** - The process of creating a virtual model of a system to study its behavior under various conditions.

## T

**TF (Transforms)** - The system in ROS for tracking coordinate frame relationships over time.

**Topic** - A named bus over which nodes exchange messages in ROS 2.

## U

**URDF (Unified Robot Description Format)** - An XML format for representing robot models in ROS, including physical and kinematic properties.

## V

**Visualization** - The process of creating graphical representations of robot data, models, and behaviors.

## W

**World file** - A Gazebo file that defines the environment, including models, lighting, and physics properties for simulation.

## X

**Xacro** - An XML macro language that extends URDF with features like variables, math expressions, and macro definitions.

## Y & Z

**Yaw** - The rotation of a robot around its vertical (Z) axis.

**Zero Torque Mode** - A robot control mode where the actuators provide minimal resistance, allowing free movement.