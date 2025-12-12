import React, { useState } from "react";
import Content from "@theme-original/Navbar/Content";
import BrowserOnly from "@docusaurus/BrowserOnly";
import styles from "./styles.module.css";

export default function ContentWrapper(props) {
  return (
    <>
      <Content {...props} />
      <BrowserOnly>
        {() => {
          const { AuthModal } = require("../../../components/AuthModal");
          const {
            UserProfileDropdown,
          } = require("../../../components/UserProfileDropdown");
          const {
            ProfileUpdateModal,
          } = require("../../../components/ProfileUpdateModal");
          const {
            Toast,
            ToastContainer,
          } = require("../../../components/Toast");
          const { useAuth } = require("../../../contexts/AuthContext");

          const AuthButton = () => {
            const [showAuthModal, setShowAuthModal] = useState(false);
            const [showProfileModal, setShowProfileModal] = useState(false);
            const [toast, setToast] = useState(null);
            const { isAuthenticated, user, signOut, updateProfile } = useAuth();

            const handleSignOut = async () => {
              try {
                await signOut();
                setToast({
                  message: "Signed out successfully!",
                  type: "success",
                });
              } catch (error) {
                console.error("Sign out error:", error);
                setToast({ message: "Failed to sign out", type: "error" });
              }
            };

            const handleUpdateProfile = async (data) => {
              try {
                await updateProfile(data);
                setToast({
                  message: "Profile updated successfully!",
                  type: "success",
                });
              } catch (error) {
                setToast({
                  message: error.message || "Failed to update profile",
                  type: "error",
                });
                throw error;
              }
            };

            if (isAuthenticated && user) {
              return (
                <>
                  <UserProfileDropdown
                    user={user}
                    onSignOut={handleSignOut}
                    onUpdateProfile={() => setShowProfileModal(true)}
                  />
                  <ProfileUpdateModal
                    isOpen={showProfileModal}
                    onClose={() => setShowProfileModal(false)}
                    user={user}
                    onUpdate={handleUpdateProfile}
                  />
                  {toast && (
                    <ToastContainer>
                      <Toast
                        message={toast.message}
                        type={toast.type}
                        onClose={() => setToast(null)}
                      />
                    </ToastContainer>
                  )}
                </>
              );
            }

            return (
              <>
                <button
                  className={styles.signInButton}
                  onClick={() => setShowAuthModal(true)}
                  aria-label="Sign in to access the textbook"
                >
                  Sign In
                </button>
                <AuthModal
                  isOpen={showAuthModal}
                  onClose={() => setShowAuthModal(false)}
                />
              </>
            );
          };

          return <AuthButton />;
        }}
      </BrowserOnly>
    </>
  );
}
