/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'doc',
      id: 'intro',
      label: 'Introduction',
    },
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module1/overview',
        'module1/ros2-fundamentals',
        'module1/nodes-topics-services',
        'module1/python-rclpy',
        'module1/urdf-humanoids',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'module2/overview',
        'module2/gazebo-simulation',
        'module2/unity-rendering',
        'module2/sensor-simulation',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        'module3/overview',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'module4/overview',
      ],
    },
    {
      type: 'doc',
      id: 'assessments',
      label: 'Assessments',
    },
  ],
};

module.exports = sidebars;
