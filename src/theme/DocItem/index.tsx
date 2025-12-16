import React, { useEffect } from "react";
import DocItem from "@theme-original/DocItem";
import BrowserOnly from "@docusaurus/BrowserOnly";
import { useHistory } from "@docusaurus/router";

export default function DocItemWrapper(props) {
  return (
    <BrowserOnly fallback={<div>Loading...</div>}>
      {() => {
        const { useAuth } = require("../../contexts/AuthContext");
        const { AuthModal } = require("../../components/AuthModal");

        const ProtectedDocItem = () => {
          const { isAuthenticated, loading, checkSession } = useAuth();
          const [showAuthModal, setShowAuthModal] = React.useState(false);
          const history = useHistory();

          useEffect(() => {
            if (!loading && !isAuthenticated) {
              setShowAuthModal(true);
            }
          }, [loading, isAuthenticated]);

          // Continuously verify authentication every 5 seconds
          useEffect(() => {
            const interval = setInterval(() => {
              checkSession();
            }, 5000);

            return () => clearInterval(interval);
          }, [checkSession]);

          const handleAuthClose = () => {
            // Don't allow closing without authentication - redirect to home
            history.push("/physical-ai-and-humanoid-robotics/");
          };

          if (loading) {
            return (
              <div
                style={{
                  display: "flex",
                  justifyContent: "center",
                  alignItems: "center",
                  minHeight: "400px",
                  fontSize: "16px",
                  color: "var(--ifm-font-color-base)",
                }}
              >
                Checking authentication...
              </div>
            );
          }

          if (!isAuthenticated) {
            return (
              <>
                <div
                  style={{
                    display: "flex",
                    flexDirection: "column",
                    justifyContent: "center",
                    alignItems: "center",
                    minHeight: "400px",
                    padding: "40px",
                    textAlign: "center",
                  }}
                >
                  <h1 style={{ marginBottom: "16px" }}>
                    Authentication Required
                  </h1>
                  <p
                    style={{
                      fontSize: "16px",
                      color: "var(--ifm-color-emphasis-700)",
                      marginBottom: "24px",
                      maxWidth: "500px",
                    }}
                  >
                    Please sign in to access the Physical AI & Humanoid Robotics
                    textbook content.
                  </p>
                  <button
                    onClick={() => setShowAuthModal(true)}
                    style={{
                      padding: "12px 24px",
                      fontSize: "16px",
                      fontWeight: 600,
                      color: "white",
                      background: "var(--ifm-color-primary)",
                      border: "none",
                      borderRadius: "8px",
                      cursor: "pointer",
                      transition: "all 0.2s ease",
                    }}
                    onMouseOver={(e) => {
                      e.currentTarget.style.background =
                        "var(--ifm-color-primary-dark)";
                    }}
                    onMouseOut={(e) => {
                      e.currentTarget.style.background =
                        "var(--ifm-color-primary)";
                    }}
                  >
                    Sign In to Continue
                  </button>
                </div>
                <AuthModal isOpen={showAuthModal} onClose={handleAuthClose} />
              </>
            );
          }

          return <DocItem {...props} />;
        };

        return <ProtectedDocItem />;
      }}
    </BrowserOnly>
  );
}
