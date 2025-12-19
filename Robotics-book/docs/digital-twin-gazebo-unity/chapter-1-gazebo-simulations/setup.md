---
title: Gazebo Setup Guide
sidebar_label: Gazebo Setup
sidebar_position: 6
description: Complete setup guide for Gazebo simulation environment for digital twin applications
---

# Gazebo Setup Guide

This guide provides comprehensive instructions for setting up Gazebo simulation environment for digital twin applications with humanoid robots. Proper setup is crucial for creating realistic and effective digital twins.

## System Requirements

### Hardware Requirements
- **Processor**: Multi-core processor (Intel i5 or equivalent recommended)
- **Memory**: 8GB RAM minimum, 16GB+ recommended
- **Graphics**: Dedicated GPU with OpenGL 2.1+ support
- **Storage**: 5GB+ free space for Gazebo and models
- **Operating System**: Ubuntu 20.04/22.04, Windows 10/11, or macOS 10.15+

### Software Dependencies
- **CMake**: 3.10 or higher
- **OpenGL**: Version 2.1 or higher
- **Qt**: Version 5.9 or higher (for GUI)
- **Python**: 3.6 or higher (for scripting)
- **Git**: For version control and model repositories

## Installation Methods

### Ubuntu Installation

For Ubuntu systems, install Gazebo using apt:

```bash
# Add OSRF repository
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

# Update and install
sudo apt update
sudo apt install gazebo libgazebo-dev
```

### Windows Installation

1. Download the installer from the [official Gazebo website](http://gazebosim.org/)
2. Run the installer with administrator privileges
3. Follow the installation wizard
4. Add Gazebo to your system PATH

### macOS Installation

Using Homebrew:
```bash
brew install gazebo
```

## ROS Integration Setup

### Installing ROS with Gazebo

For robotics applications, ROS integration is essential:

```bash
# For ROS Noetic (Ubuntu 20.04)
sudo apt install ros-noetic-desktop-full
sudo apt install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control

# For ROS 2 Humble Hawksbill (Ubuntu 22.04)
sudo apt install ros-humble-desktop
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros-control
```

### Environment Setup

Add these lines to your `~/.bashrc` (Linux/macOS) or set as system environment variables (Windows):

```bash
# Gazebo setup
source /usr/share/gazebo/setup.sh
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/.gazebo/models
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/.gazebo/worlds

# For ROS integration
source /opt/ros/noetic/setup.bash  # For ROS Noetic
# OR
source /opt/ros/humble/setup.bash  # For ROS 2 Humble
```

## Initial Configuration

### Gazebo Configuration Directory

Gazebo creates a configuration directory at `~/.gazebo` (Linux/macOS) or `%USERPROFILE%\.gazebo` (Windows). This directory contains:

- `models/` - Downloaded and custom models
- `worlds/` - Custom world files
- `plugins/` - Custom plugins
- `gui.ini` - GUI configuration
- `server.ini` - Server configuration

### Basic Launch Test

Test your installation by launching Gazebo:

```bash
gazebo
```

If successful, you should see the Gazebo interface with a default empty world.

## Digital Twin Specific Configuration

### Physics Engine Configuration

For digital twin applications, optimal physics settings are important:

```bash
# Create a custom physics configuration
mkdir -p ~/.gazebo/config.d
```

Create `~/.gazebo/config.d/digital_twin.ini`:

```ini
[physics]
engine = ode
max_step_size = 0.001
real_time_factor = 1.0
real_time_update_rate = 1000
max_contacts = 20
gravity_x = 0
gravity_y = 0
gravity_z = -9.8

[ode]
sor_pgs_precon_iters = 2
sor_pgs_iters = 50
sor_pgs_w = 1.3
cfm = 0.0
erp = 0.2
contact_surface_layer = 0.001
contact_max_correcting_vel = 100.0
max_contacts = 20
```

### Graphics Configuration

For better visualization of digital twins:

```ini
[video]
fullscreen = false
width = 1280
height = 1024
hz = 60
vsync = true
aa_samples = 4
```

## Model Database Setup

### Default Model Database

Gazebo comes with a default model database. To enhance it for humanoid robotics:

```bash
# Clone the model database
git clone https://github.com/osrf/gazebo_models.git ~/.gazebo/models
```

### Custom Model Directory

Create a directory for your custom digital twin models:

```bash
mkdir -p ~/gazebo_models/humanoid
mkdir -p ~/gazebo_models/digital_twins
```

Add this to your environment:

```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/gazebo_models
```

## Plugin Configuration

### Essential Plugins for Digital Twins

For digital twin applications, ensure these plugins are available:

1. **ROS Bridge Plugins**: For ROS/ROS2 integration
2. **Sensor Plugins**: For various sensor types
3. **Controller Plugins**: For joint control
4. **GUI Plugins**: For enhanced visualization

### Plugin Path Configuration

Add plugin paths to your environment:

```bash
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins
```

## IDE and Development Environment

### Recommended IDE Setup

For developing digital twin applications:

1. **VS Code** with ROS extensions
2. **PyCharm** for Python development
3. **Qt Creator** for C++ development

### ROS Development Setup

Install ROS development tools:

```bash
# For ROS Noetic
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# Initialize rosdep
sudo rosdep init
rosdep update
```

## Network Configuration

### Multi-Machine Setup

For distributed digital twin applications:

```bash
# Set ROS master URI
export ROS_MASTER_URI=http://localhost:11311

# Set ROS IP (important for multi-machine setups)
export ROS_IP=127.0.0.1
```

## Performance Optimization

### Graphics Settings

For optimal performance with humanoid models:

1. **Reduce anti-aliasing** if experiencing performance issues
2. **Adjust shadow quality** based on hardware capabilities
3. **Limit visual effects** in simulation settings

### Physics Optimization

For better simulation performance:

1. **Adjust time step**: Balance accuracy vs. performance
2. **Limit max contacts**: Reduce to minimum required
3. **Optimize collision meshes**: Use simpler shapes when possible

## Troubleshooting Common Issues

### Installation Issues

**Problem**: Gazebo fails to start with OpenGL errors
**Solution**:
```bash
# Check OpenGL support
glxinfo | grep "OpenGL version"
# Install graphics drivers if needed
sudo apt install mesa-utils
```

**Problem**: Model database not loading
**Solution**:
```bash
# Clear and re-download models
rm -rf ~/.gazebo/models
git clone https://github.com/osrf/gazebo_models.git ~/.gazebo/models
```

### Performance Issues

**Problem**: Slow simulation
**Solutions**:
- Reduce `real_time_update_rate` in physics config
- Close other applications to free up resources
- Check for overheating and cooling issues

**Problem**: Robot models falling through surfaces
**Solutions**:
- Check mass and inertia values in URDF
- Verify collision geometries are properly defined
- Adjust physics parameters (time step, ERP, CFM)

### ROS Integration Issues

**Problem**: Cannot find Gazebo plugins
**Solution**:
```bash
# Check plugin paths
echo $GAZEBO_PLUGIN_PATH
# Source ROS setup again
source /opt/ros/noetic/setup.bash  # or appropriate ROS version
```

## Verification Steps

### Basic Verification

1. Launch Gazebo: `gazebo`
2. Verify GUI loads without errors
3. Test basic controls (pan, zoom, rotate camera)
4. Try inserting a simple model (e.g., a box)

### Advanced Verification

1. Test with a simple robot model
2. Verify sensor plugins work
3. Test joint controllers if using ROS
4. Validate physics behavior

### Digital Twin Specific Tests

1. Load a humanoid model
2. Verify all joints respond to commands
3. Test sensor data publication
4. Validate physics realism

## Recommended Workflow

### Development Cycle

1. **Model Creation**: Design robot in CAD software
2. **URDF Generation**: Convert to Gazebo-compatible URDF
3. **Simulation Testing**: Test in Gazebo environment
4. **Validation**: Compare with real robot behavior
5. **Iteration**: Refine based on test results

### Best Practices

- **Version Control**: Use Git for all model and world files
- **Documentation**: Maintain clear documentation for all configurations
- **Testing**: Regularly validate against real-world data
- **Performance**: Monitor simulation performance metrics

## Next Steps

After successful setup:

1. **Learn URDF**: Understand robot description format
2. **Explore Examples**: Work through Gazebo tutorials
3. **Create Models**: Start building your digital twin models
4. **Integrate Sensors**: Add perception capabilities to your twins

## Summary

Proper Gazebo setup is fundamental to creating effective digital twins for humanoid robotics. This guide covered installation, configuration, and optimization for digital twin applications. With a properly configured environment, you can proceed to create realistic simulations that accurately represent real-world robots.

## Quick Start Commands

Here are the essential commands to get started quickly:

```bash
# Launch Gazebo with empty world
gazebo

# Launch with a specific world file
gazebo my_world.world

# Launch with ROS integration
roslaunch my_robot_gazebo my_robot_world.launch

# Check Gazebo version
gazebo --version
```

## Exercises

1. Install Gazebo on your system following this guide
2. Launch Gazebo and verify it works correctly
3. Download and test a sample humanoid model
4. Create a simple custom world file