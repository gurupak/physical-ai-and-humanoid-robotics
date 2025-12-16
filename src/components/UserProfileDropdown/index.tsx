import React, { useState, useRef, useEffect } from "react";
import styles from "./styles.module.css";

interface UserProfileDropdownProps {
  user: any;
  onSignOut: () => void;
  onUpdateProfile: () => void;
}

export const UserProfileDropdown: React.FC<UserProfileDropdownProps> = ({
  user,
  onSignOut,
  onUpdateProfile,
}) => {
  const [isOpen, setIsOpen] = useState(false);
  const dropdownRef = useRef<HTMLDivElement>(null);

  const displayName = user.name || user.email || "User";
  const initials = getInitials(displayName);
  const avatarUrl = user.image || null;

  // Close dropdown when clicking outside
  useEffect(() => {
    const handleClickOutside = (event: MouseEvent) => {
      if (
        dropdownRef.current &&
        !dropdownRef.current.contains(event.target as Node)
      ) {
        setIsOpen(false);
      }
    };

    if (isOpen) {
      document.addEventListener("mousedown", handleClickOutside);
    }

    return () => {
      document.removeEventListener("mousedown", handleClickOutside);
    };
  }, [isOpen]);

  function getInitials(name: string): string {
    if (!name) return "U";
    const parts = name.trim().split(" ");
    if (parts.length === 1) return parts[0].charAt(0).toUpperCase();
    return (
      parts[0].charAt(0) + parts[parts.length - 1].charAt(0)
    ).toUpperCase();
  }

  return (
    <div className={styles.dropdown} ref={dropdownRef}>
      <button
        className={styles.trigger}
        onClick={() => setIsOpen(!isOpen)}
        aria-label="User menu"
        aria-expanded={isOpen}
        title={displayName}
      >
        <div className={styles.avatar}>
          {avatarUrl ? (
            <img
              src={avatarUrl}
              alt={displayName}
              className={styles.avatarImage}
            />
          ) : (
            <span className={styles.avatarInitials}>{initials}</span>
          )}
        </div>
        <span className={styles.userName}>{displayName}</span>
        <svg
          className={`${styles.chevron} ${isOpen ? styles.chevronUp : ""}`}
          width="16"
          height="16"
          viewBox="0 0 16 16"
          fill="none"
        >
          <path
            d="M4 6L8 10L12 6"
            stroke="currentColor"
            strokeWidth="2"
            strokeLinecap="round"
            strokeLinejoin="round"
          />
        </svg>
      </button>

      {isOpen && (
        <div className={styles.menu}>
          <div className={styles.userInfo}>
            <div className={styles.userInfoAvatar}>
              {avatarUrl ? (
                <img
                  src={avatarUrl}
                  alt={displayName}
                  className={styles.avatarImageLarge}
                />
              ) : (
                <span className={styles.avatarInitialsLarge}>{initials}</span>
              )}
            </div>
            <div className={styles.userDetails}>
              <div className={styles.fullName}>{displayName}</div>
              <div className={styles.email}>{user.email}</div>
            </div>
          </div>
          <div className={styles.divider} />
          <button
            className={styles.menuItem}
            onClick={() => {
              setIsOpen(false);
              onUpdateProfile();
            }}
          >
            <svg width="20" height="20" viewBox="0 0 20 20" fill="none">
              <path
                d="M10 10C11.6569 10 13 8.65685 13 7C13 5.34315 11.6569 4 10 4C8.34315 4 7 5.34315 7 7C7 8.65685 8.34315 10 10 10Z"
                stroke="currentColor"
                strokeWidth="1.5"
                strokeLinecap="round"
                strokeLinejoin="round"
              />
              <path
                d="M4 17C4 14.2386 6.68629 12 10 12C13.3137 12 16 14.2386 16 17"
                stroke="currentColor"
                strokeWidth="1.5"
                strokeLinecap="round"
                strokeLinejoin="round"
              />
            </svg>
            <span>Update Profile</span>
          </button>
          <button
            className={styles.menuItem}
            onClick={() => {
              setIsOpen(false);
              onSignOut();
            }}
          >
            <svg width="20" height="20" viewBox="0 0 20 20" fill="none">
              <path
                d="M13 17H7C6.44772 17 6 16.5523 6 16V4C6 3.44772 6.44772 3 7 3H13"
                stroke="currentColor"
                strokeWidth="1.5"
                strokeLinecap="round"
                strokeLinejoin="round"
              />
              <path
                d="M14 13L17 10M17 10L14 7M17 10H9"
                stroke="currentColor"
                strokeWidth="1.5"
                strokeLinecap="round"
                strokeLinejoin="round"
              />
            </svg>
            <span>Sign Out</span>
          </button>
        </div>
      )}
    </div>
  );
};
