// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'ROS 2 Core Concepts',
      items: [
        'ros2-concepts/index',
        'ros2-concepts/introduction',
        'ros2-concepts/nodes',
        'ros2-concepts/topics-messages',
        'ros2-concepts/services',
        'ros2-concepts/data-flow-humanoid',
        'ros2-concepts/learning-objectives'
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin (Gazebo and Unity)',
      items: [
        'Module-2-The-Digital-Twin',
        'Chapter-1-Physical-simulation-with-gazebo',
        'Chapter-2-Digital-twins-and-HRI-in-unity',
        'Chapter-3-Sensor-simulation-and-validation'
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        'Module-3-The-AI-Robot-Brain-NVIDIA-Isaac',
        'Chapter-1-Perception-and-Training-with-NVIDIA-Isaac-Sim',
        'Chapter-2-Hardware-Accelerated-Perception-with-Isaac-ROS',
        'Chapter-3-Navigation-and-Planning-for-Humanoid-Robots-Nav2'
      ],
    },
    {
      type: 'category',
      label: 'Python-ROS Integration',
      items: [
        'python-ros-integration/index',
        'python-ros-integration/intro-rclpy',
        'python-ros-integration/creating-nodes',
        'python-ros-integration/publish-subscribe',
        'python-ros-integration/using-services',
        'python-ros-integration/agent-controller-actuator'
      ],
    },
    {
      type: 'category',
      label: 'URDF Humanoid Modeling',
      items: [
        'urdf-humanoid-modeling/index',
        'urdf-humanoid-modeling/intro-urdf',
        'urdf-humanoid-modeling/links-joints',
        'urdf-humanoid-modeling/frames-transformations',
        'urdf-humanoid-modeling/urdf-physical-schema',
        'urdf-humanoid-modeling/practical-examples',
        'urdf-humanoid-modeling/xml-examples',
        'urdf-humanoid-modeling/visual-aids',
        'urdf-humanoid-modeling/exercises',
        'urdf-humanoid-modeling/validation-best-practices'
      ],
    },
  ],
};

module.exports = sidebars;