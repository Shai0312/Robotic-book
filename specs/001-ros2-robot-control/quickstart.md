# Quickstart: ROS 2 Robot Control Module

## Prerequisites

Before you begin working with this educational module, ensure you have:

- Basic Python knowledge (variables, functions, classes)
- A modern web browser (Chrome, Firefox, Safari, or Edge)
- Git installed (for cloning/forking the repository)
- Node.js LTS version installed (for local development)

## Getting Started

### Option 1: Browse Online
The easiest way to access the content is through the published documentation:
1. Visit the GitHub Pages site (URL will be available after deployment)
2. Navigate through the three main chapters:
   - Chapter 1: ROS 2 Core Concepts
   - Chapter 2: Python-ROS Integration
   - Chapter 3: URDF Humanoid Modeling

### Option 2: Local Development
To run the documentation site locally:

1. **Clone the repository**
   ```bash
   git clone <repository-url>
   cd <repository-name>
   ```

2. **Install dependencies**
   ```bash
   npm install
   # or
   yarn install
   ```

3. **Start the development server**
   ```bash
   npm start
   # or
   yarn start
   ```
   This will start the site at `http://localhost:3000`

4. **Build for production** (if needed)
   ```bash
   npm run build
   ```

## Chapter Overview

### Chapter 1: ROS 2 Core Concepts
Start here if you're new to ROS 2. This chapter covers:
- What ROS 2 is and why it's used in robotics
- Nodes: the basic computational elements
- Topics and messages: for data streaming
- Services: for request-response communication
- How these concepts apply to humanoid robots

### Chapter 2: Python-ROS Integration
Once you understand the basics, learn how to:
- Use rclpy, the Python client library for ROS 2
- Create Python nodes that interact with ROS 2
- Implement the agent → controller → actuator loop
- Connect AI agents to robot control systems

### Chapter 3: URDF Humanoid Modeling
The final chapter covers robot modeling:
- Understanding URDF (Unified Robot Description Format)
- Creating links and joints for robot structure
- Working with frames and transformations
- How URDF serves as the robot's physical schema

## Learning Tips

- Start with Chapter 1 if you're new to ROS 2
- Each chapter builds on the previous one
- Try the code examples as you go
- Use the navigation sidebar to jump between sections
- Check the learning objectives at the start of each section

## Getting Help

- Use the search function to find specific topics
- Refer to the official ROS 2 documentation linked throughout
- Join the community discussions (if available)
- Report issues with the documentation via GitHub Issues