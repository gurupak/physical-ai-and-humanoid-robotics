import React from "react";
import { FaRobot, FaCube, FaEye, FaBrain, FaGraduationCap, FaCalendarAlt, FaLightbulb } from "react-icons/fa";
import styles from "./styles.module.css";

interface CardProps {
  icon: React.ReactNode;
  iconFallback: string;
  title: string;
  description?: string;
  items?: string[];
  className?: string;
}

function Card({ icon, iconFallback, title, description, items, className }: CardProps) {
  const [iconError, setIconError] = React.useState(false);

  return (
    <div className={`${styles.card} ${className || ''}`}>
      <div className={styles.cardIcon} aria-hidden="true">
        {iconError ? iconFallback : icon}
      </div>
      <h3 className={styles.cardTitle}>{title}</h3>
      {description && <p className={styles.cardDescription}>{description}</p>}
      {items && items.length > 0 && (
        <ul className={styles.cardList}>
          {items.map((item, index) => (
            <li key={index}>{item}</li>
          ))}
        </ul>
      )}
    </div>
  );
}

export default function HomepageCards(): JSX.Element {
  return (
    <section className={styles.cardsSection}>
      <div className={styles.container}>
        {/* Why Physical AI Matters - Highlighted Overview Card */}
        <div className={styles.highlightCard}>
          <div className={styles.highlightIcon} aria-hidden="true">
            <FaLightbulb />
          </div>
          <h2 className={styles.highlightTitle}>Why Physical AI Matters</h2>
          <p className={styles.highlightDescription}>
            Physical AI represents the convergence of artificial intelligence with robotics,
            enabling machines to perceive, reason, and act in the real world. From autonomous
            vehicles to humanoid assistants, this technology is reshaping industries and
            defining the future of human-machine collaboration.
          </p>
        </div>

        {/* Module Cards */}
        <h2 className={styles.sectionHeading}>Core Modules</h2>
        <div className={styles.cardsGrid}>
          <Card
            icon={<FaRobot size={48} />}
            iconFallback="🤖"
            title="ROS 2 Fundamentals"
            description="Master the Robot Operating System 2"
            items={[
              "Node architecture and communication",
              "Topics, services, and actions",
              "Launch files and parameters",
              "Real-time system integration"
            ]}
          />
          <Card
            icon={<FaCube size={48} />}
            iconFallback="🎮"
            title="Gazebo & Unity Simulation"
            description="Build realistic robotic environments"
            items={[
              "Physics-based simulation",
              "Sensor modeling and URDF",
              "Unity-ROS integration",
              "Testing and validation"
            ]}
          />
          <Card
            icon={<FaEye size={48} />}
            iconFallback="👁️"
            title="NVIDIA Isaac Platform"
            description="GPU-accelerated robotics development"
            items={[
              "Isaac Sim for digital twins",
              "Synthetic data generation",
              "Hardware acceleration",
              "Deployment optimization"
            ]}
          />
          <Card
            icon={<FaBrain size={48} />}
            iconFallback="🧠"
            title="Vision-Language-Action (VLA)"
            description="End-to-end embodied AI systems"
            items={[
              "Multimodal perception",
              "Language-grounded control",
              "Foundation models for robotics",
              "Real-world deployment"
            ]}
          />
        </div>

        {/* Learning Outcomes */}
        <h2 className={styles.sectionHeading}>What You'll Learn</h2>
        <div className={styles.outcomeCard}>
          <div className={styles.outcomeIcon} aria-hidden="true">
            <FaGraduationCap size={48} />
          </div>
          <ul className={styles.outcomeList}>
            <li>Design and implement autonomous robotic systems using ROS 2</li>
            <li>Create realistic simulation environments for testing and validation</li>
            <li>Leverage GPU acceleration for advanced robotics applications</li>
            <li>Integrate vision and language models for embodied AI</li>
            <li>Deploy AI-powered robots from simulation to real hardware</li>
            <li>Apply best practices for safe and reliable robotic systems</li>
          </ul>
        </div>

        {/* Weekly Breakdown */}
        <h2 className={styles.sectionHeading}>13-Week Learning Path</h2>
        <div className={styles.timelineCard}>
          <div className={styles.timelineIcon} aria-hidden="true">
            <FaCalendarAlt size={48} />
          </div>
          <div className={styles.timelineContent}>
            <div className={styles.timelineItem}>
              <span className={styles.timelineWeeks}>Weeks 1-2</span>
              <span className={styles.timelineTitle}>Introduction to Physical AI</span>
              <p className={styles.timelineDescription}>
                Foundations, ecosystem overview, and development setup
              </p>
            </div>
            <div className={styles.timelineItem}>
              <span className={styles.timelineWeeks}>Weeks 3-5</span>
              <span className={styles.timelineTitle}>ROS 2 Fundamentals</span>
              <p className={styles.timelineDescription}>
                Core concepts, communication patterns, and practical projects
              </p>
            </div>
            <div className={styles.timelineItem}>
              <span className={styles.timelineWeeks}>Weeks 6-8</span>
              <span className={styles.timelineTitle}>Simulation & Testing</span>
              <p className={styles.timelineDescription}>
                Gazebo/Unity environments, sensor modeling, and validation
              </p>
            </div>
            <div className={styles.timelineItem}>
              <span className={styles.timelineWeeks}>Weeks 9-10</span>
              <span className={styles.timelineTitle}>NVIDIA Isaac Platform</span>
              <p className={styles.timelineDescription}>
                GPU-accelerated workflows and synthetic data generation
              </p>
            </div>
            <div className={styles.timelineItem}>
              <span className={styles.timelineWeeks}>Weeks 11-12</span>
              <span className={styles.timelineTitle}>Vision-Language-Action Models</span>
              <p className={styles.timelineDescription}>
                Multimodal AI, language-grounded control, and foundation models
              </p>
            </div>
            <div className={styles.timelineItem}>
              <span className={styles.timelineWeeks}>Week 13</span>
              <span className={styles.timelineTitle}>Integration & Deployment</span>
              <p className={styles.timelineDescription}>
                Real-world deployment, best practices, and capstone project
              </p>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}
