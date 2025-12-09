import React from "react";
import Link from "@docusaurus/Link";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";
import styles from "./styles.module.css";

export default function HomepageHero(): JSX.Element {
  const { siteConfig } = useDocusaurusContext();

  return (
    <header className={styles.heroBanner} role="banner">
      <div className={styles.heroContainer}>
        <div className={styles.heroContent}>
          <h1 className={styles.heroTitle}>{siteConfig.title}</h1>
          <p className={styles.heroTagline}>{siteConfig.tagline}</p>
          <p className={styles.heroDescription}>
            Master ROS 2, Gazebo, NVIDIA Isaac, and Vision-Language-Action
            systems. From simulation to real hardware deployment.
          </p>
          <nav className={styles.heroButtons} aria-label="Primary navigation">
            <Link
              className="button button--primary button--lg"
              to="/"
              aria-label="Start reading the introduction"
            >
              Start Reading
            </Link>
            <Link
              className="button button--secondary button--lg"
              to="/docs/introduction"
              aria-label="Explore the book table of contents"
            >
              Explore the Book
            </Link>
          </nav>
        </div>
      </div>
    </header>
  );
}
