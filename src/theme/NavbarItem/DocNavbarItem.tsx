import React, { useState } from "react";
import DocNavbarItem from "@theme-original/NavbarItem/DocNavbarItem";
import BrowserOnly from "@docusaurus/BrowserOnly";

export default function DocNavbarItemWrapper(props) {
  const { customProps, ...restProps } = props;
  const requiresAuth = customProps?.requiresAuth;

  if (!requiresAuth) {
    return <DocNavbarItem {...restProps} />;
  }

  return (
    <BrowserOnly fallback={<span />}>
      {() => {
        const { AuthModal } = require("../../components/AuthModal");
        const { useAuth } = require("../../contexts/AuthContext");

        const AuthProtectedNavItem = () => {
          const [showAuthModal, setShowAuthModal] = useState(false);
          const { isAuthenticated } = useAuth();

          if (isAuthenticated) {
            return <DocNavbarItem {...restProps} />;
          }

          // If not authenticated, render a fake link that shows auth modal
          return (
            <>
              <a
                className="navbar__item navbar__link"
                onClick={(e) => {
                  e.preventDefault();
                  setShowAuthModal(true);
                }}
                style={{ cursor: "pointer" }}
              >
                Book
              </a>
              {showAuthModal && (
                <AuthModal
                  isOpen={showAuthModal}
                  onClose={() => setShowAuthModal(false)}
                />
              )}
            </>
          );
        };

        return <AuthProtectedNavItem />;
      }}
    </BrowserOnly>
  );
}
