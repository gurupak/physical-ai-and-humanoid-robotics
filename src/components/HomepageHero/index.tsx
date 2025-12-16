import React, { useState } from "react";
import Link from "@docusaurus/Link";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";
import BrowserOnly from "@docusaurus/BrowserOnly";
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
          <BrowserOnly>
            {() => {
              const { AuthModal } = require("../AuthModal");
              const { useAuth } = require("../../contexts/AuthContext");

              const HeroButtons = () => {
                const [showAuthModal, setShowAuthModal] = useState(false);
                const { isAuthenticated } = useAuth();

                const handleBookAccess = (e: React.MouseEvent) => {
                  if (!isAuthenticated) {
                    e.preventDefault();
                    setShowAuthModal(true);
                  }
                  // If authenticated, let the link work normally
                };

                return (
                  <>
                    <nav
                      className={styles.heroButtons}
                      aria-label="Primary navigation"
                    >
                      <Link
                        className="button button--primary button--lg"
                        to="/docs/introduction"
                        aria-label="Start reading the introduction"
                        onClick={handleBookAccess}
                      >
                        Start Reading
                      </Link>
                      <Link
                        className="button button--secondary button--lg"
                        to="/docs/introduction"
                        aria-label="Explore the book table of contents"
                        onClick={handleBookAccess}
                      >
                        Explore the Book
                      </Link>
                    </nav>
                    <AuthModal
                      isOpen={showAuthModal}
                      onClose={() => setShowAuthModal(false)}
                    />
                  </>
                );
              };

              return <HeroButtons />;
            }}
          </BrowserOnly>
        </div>
      </div>
    </header>
  );
}
