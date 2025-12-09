import type { SidebarsConfig } from "@docusaurus/plugin-content-docs";

/**
 * Explicit sidebar configuration with proper chapter ordering
 * This prevents Docusaurus from auto-generating chaotic sidebar structure
 */
const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    {
      type: "doc",
      id: "introduction",
      label: "Introduction",
    },
    {
      type: "category",
      label: "ROS 2 Fundamentals",
      collapsed: false,
      link: {
        type: "doc",
        id: "ros2-fundamentals/ros2-fundamentals",
      },
      items: [
        "ros2-fundamentals/01-overview",
        "ros2-fundamentals/02-installation",
        "ros2-fundamentals/03-first-node",
        "ros2-fundamentals/04-topics-messages",
        "ros2-fundamentals/05-services",
        "ros2-fundamentals/06-actions",
        "ros2-fundamentals/07-parameters-launch",
        "ros2-fundamentals/08-best-practices",
      ],
    },
    {
      type: "category",
      label: "The Digital Twin",
      collapsed: false,
      link: {
        type: "doc",
        id: "module-2-digital-twin/module-2-digital-twin",
      },
      items: [
        "module-2-digital-twin/digital-twin-intro",
        "module-2-digital-twin/gazebo-physics/gazebo-physics",
        "module-2-digital-twin/unity-rendering/unity-rendering",
        "module-2-digital-twin/integration",
      ],
    },
    {
      type: "category",
      label: "The AI-Robot Brain (NVIDIA Isaac™)",
      collapsed: false,
      link: {
        type: "doc",
        id: "chapter-3-isaac-ai-brain/index",
      },
      items: [
        // Quick Start
        "chapter-3-isaac-ai-brain/quick-start",

        // Section 1: Introduction & Overview
        {
          type: "category",
          label: "1. Platform Overview",
          collapsed: true,
          items: [
            "chapter-3-isaac-ai-brain/introduction",
            "chapter-3-isaac-ai-brain/isaac-gr00t-architecture",
            "chapter-3-isaac-ai-brain/simulator-comparison",
            "chapter-3-isaac-ai-brain/platform-selection",
          ],
        },

        // Section 2: Isaac Sim & Synthetic Data
        {
          type: "category",
          label: "2. Isaac Sim & Synthetic Data",
          collapsed: true,
          items: [
            "chapter-3-isaac-ai-brain/isaac-sim-synthetic-data",
            "chapter-3-isaac-ai-brain/synthetic-data-overview",
            "chapter-3-isaac-ai-brain/scene-creation",
            "chapter-3-isaac-ai-brain/prerequisites",
            "chapter-3-isaac-ai-brain/installation",
          ],
        },

        // Section 3: VSLAM Implementation
        {
          type: "category",
          label: "3. Visual SLAM",
          collapsed: true,
          items: [
            "chapter-3-isaac-ai-brain/vslam-fundamentals",
            "chapter-3-isaac-ai-brain/isaac-ros-vslam-overview",
            "chapter-3-isaac-ai-brain/isaac-ros-vslam-implementation",
            "chapter-3-isaac-ai-brain/stereo-setup",
            "chapter-3-isaac-ai-brain/vslam-integration",
            "chapter-3-isaac-ai-brain/vslam-launch-snippets",
            "chapter-3-isaac-ai-brain/vslam-accuracy-measurement",
          ],
        },

        // Section 4: Humanoid Navigation
        {
          type: "category",
          label: "4. Humanoid Navigation",
          collapsed: true,
          items: [
            "chapter-3-isaac-ai-brain/nav2-humanoid-integration",
            "chapter-3-isaac-ai-brain/nav2-humanoid-configuration",
            "chapter-3-isaac-ai-brain/bipedal-path-planning",
            "chapter-3-isaac-ai-brain/footstep-planning-parameters",
            "chapter-3-isaac-ai-brain/dynamic-stability-integration",
            "chapter-3-isaac-ai-brain/obstacle-avoidance-humanoid",
            "chapter-3-isaac-ai-brain/roll-pitch-compensation",
            "chapter-3-isaac-ai-brain/walking-gait-integration",
            "chapter-3-isaac-ai-brain/dynamic-gait-transitions",
          ],
        },

        // Section 5: Integration & Deployment
        {
          type: "category",
          label: "5. System Integration",
          collapsed: true,
          items: [
            "chapter-3-isaac-ai-brain/ros2-integration",
            "chapter-3-isaac-ai-brain/runtime-configuration",
            "chapter-3-isaac-ai-brain/complete-integration-guide",
            "chapter-3-isaac-ai-brain/production-deployment-validation",
            "chapter-3-isaac-ai-brain/final-implementation-validation",
          ],
        },

        // Section 6: Performance & Optimization
        {
          type: "category",
          label: "6. Performance & Optimization",
          collapsed: true,
          items: [
            "chapter-3-isaac-ai-brain/hardware-acceleration",
            "chapter-3-isaac-ai-brain/gpu-optimization",
            "chapter-3-isaac-ai-brain/performance-tuning",
          ],
        },

        // Section 7: Reference & Resources
        {
          type: "category",
          label: "7. Reference & Resources",
          collapsed: true,
          items: [
            "chapter-3-isaac-ai-brain/learning-objectives",
            "chapter-3-isaac-ai-brain/common-errors",
            "chapter-3-isaac-ai-brain/humanoid-applications",
            "chapter-3-isaac-ai-brain/writing-guide",
            "chapter-3-isaac-ai-brain/us1-summary",
            "chapter-3-isaac-ai-brain/project-completion-summary",
            "chapter-3-isaac-ai-brain/final-documentation-package",
          ],
        },
      ],
    },
    {
      type: "category",
      label: "Chapter 4: Vision-Language-Action Models",
      collapsed: false,
      link: {
        type: "doc",
        id: "chapter-4-vla/chapter-4-vla",
      },
      items: [
        "chapter-4-vla/vla-introduction",
        "chapter-4-vla/voice-to-action",
        "chapter-4-vla/cognitive-planning",
        "chapter-4-vla/vision-integration",
        "chapter-4-vla/capstone-project",
      ],
    },
  ],
};

export default sidebars;
