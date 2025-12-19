# ROS 2 Robot Control Module

Welcome to the ROS 2 Robot Control Module educational content! This comprehensive guide is designed for AI engineers and robotics students with basic Python knowledge who want to understand how to connect AI agents to humanoid robot control systems.

## About This Module

This educational module covers three main areas:

1. **ROS 2 Core Concepts**: Understanding nodes, topics, services, and messages
2. **Python-ROS Integration**: Connecting Python-based AI agents to robot control systems
3. **URDF Humanoid Modeling**: Modeling humanoid robots using Unified Robot Description Format

## Learning Objectives

By completing this module, you will be able to:

- Understand fundamental ROS 2 concepts including nodes, topics, services, and messages
- Connect Python-based AI agents to robot control systems using rclpy
- Implement the agent → controller → actuator loop pattern
- Model humanoid robots using URDF (Unified Robot Description Format)
- Create complete robot descriptions with proper links, joints, and frames
- Validate and test URDF models for humanoid robots

## Getting Started

### Prerequisites

- Basic knowledge of Python programming
- Understanding of fundamental programming concepts (variables, functions, classes)
- Basic understanding of 3D coordinate systems and transformations
- Familiarity with XML syntax (for URDF modeling)

### Local Development

1. **Clone the repository**:
   ```bash
   git clone https://github.com/your-username/your-repo.git
   cd your-repo
   ```

2. **Install dependencies**:
   ```bash
   npm install
   ```

3. **Start the development server**:
   ```bash
   npm start
   ```

This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

### Build for Production

To build the documentation site for production:

```bash
npm run build
```

This command generates static content into the `build` directory and can be served using any static contents hosting service.

### Deployment

The site is automatically deployed to GitHub Pages when changes are pushed to the `master` branch. See the `.github/workflows/deploy.yml` file for deployment configuration.

## Project Structure

```
robotics-book/
├── docs/                    # Documentation content
│   ├── intro.md            # Introduction page
│   ├── quickstart.md       # Quick start guide
│   ├── ros2-concepts/      # Chapter 1: ROS 2 Core Concepts
│   ├── python-ros-integration/ # Chapter 2: Python-ROS Integration
│   └── urdf-humanoid-modeling/ # Chapter 3: URDF Humanoid Modeling
├── src/                    # Custom React components
├── static/                 # Static files
├── docusaurus.config.js    # Site configuration
├── sidebars.js             # Sidebar configuration
└── package.json            # Dependencies and scripts
```

## Documentation Navigation

The educational content is organized as follows:

- **Quick Start Guide**: Get up and running quickly with ROS 2
- **Chapter 1: ROS 2 Core Concepts**: Fundamental concepts of ROS 2
- **Chapter 2: Python-ROS Integration**: Connecting AI agents to robot control
- **Chapter 3: URDF Humanoid Modeling**: Creating humanoid robot models
- **Reference**: Glossary of ROS 2 and robotics terms

## Contributing

We welcome contributions to improve this educational module! To contribute:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Make your changes
4. Commit your changes (`git commit -m 'Add amazing feature'`)
5. Push to the branch (`git push origin feature/amazing-feature`)
6. Open a Pull Request

## Support

If you encounter any issues or have questions about the content:

- Check the [Glossary](./docs/glossary.md) for definitions of key terms
- Review the [Troubleshooting](./docs/python-ros-integration/troubleshooting.md) section in Chapter 2
- Open an issue in this repository for technical problems
- Visit the [ROS Answers](https://answers.ros.org/questions/) forum for ROS-specific questions

## License

This educational module is open source and available under the [MIT License](./LICENSE).

## Acknowledgments

- Built with [Docusaurus](https://docusaurus.io/), a modern static website generator
- Powered by [ROS 2](https://docs.ros.org/en/humble/), the Robot Operating System
- Inspired by the robotics and AI community