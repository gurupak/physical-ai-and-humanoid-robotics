import React, { useState, useEffect } from "react";
import styles from "./styles.module.css";

interface ProfileUpdateModalProps {
  isOpen: boolean;
  onClose: () => void;
  user: any;
  onUpdate: (data: ProfileData) => Promise<void>;
}

export interface ProfileData {
  firstName: string;
  lastName: string;
  email: string;
  phone: string;
  expertiseLevel: string;
}

const EXPERTISE_LEVELS = [
  { value: "beginner", label: "Beginner - New to robotics and AI" },
  {
    value: "intermediate",
    label: "Intermediate - Some experience with ROS/Python",
  },
  { value: "advanced", label: "Advanced - Experienced with robotics systems" },
  { value: "expert", label: "Expert - Professional in the field" },
];

export const ProfileUpdateModal: React.FC<ProfileUpdateModalProps> = ({
  isOpen,
  onClose,
  user,
  onUpdate,
}) => {
  const [formData, setFormData] = useState<ProfileData>({
    firstName: "",
    lastName: "",
    email: "",
    phone: "",
    expertiseLevel: "beginner",
  });
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState("");

  useEffect(() => {
    if (isOpen && user) {
      // Parse name into first and last name
      const nameParts = (user.name || "").split(" ");
      setFormData({
        firstName: nameParts[0] || "",
        lastName: nameParts.slice(1).join(" ") || "",
        email: user.email || "",
        phone: user.phone || "",
        expertiseLevel:
          user.expertiseLevel || user.expertise_level || "beginner",
      });
    }
  }, [isOpen, user]);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError("");
    setLoading(true);

    try {
      await onUpdate(formData);
      onClose();
    } catch (err) {
      setError(err.message || "Failed to update profile");
    } finally {
      setLoading(false);
    }
  };

  const handleChange = (
    e: React.ChangeEvent<HTMLInputElement | HTMLSelectElement>,
  ) => {
    setFormData((prev) => ({
      ...prev,
      [e.target.name]: e.target.value,
    }));
  };

  if (!isOpen) return null;

  return (
    <div className={styles.overlay} onClick={onClose}>
      <div className={styles.modal} onClick={(e) => e.stopPropagation()}>
        <div className={styles.header}>
          <h2 className={styles.title}>Update Profile</h2>
          <button
            className={styles.closeButton}
            onClick={onClose}
            aria-label="Close modal"
          >
            <svg width="20" height="20" viewBox="0 0 20 20" fill="none">
              <path
                d="M15 5L5 15M5 5L15 15"
                stroke="currentColor"
                strokeWidth="2"
                strokeLinecap="round"
              />
            </svg>
          </button>
        </div>

        <form onSubmit={handleSubmit} className={styles.form}>
          <div className={styles.formRow}>
            <div className={styles.formGroup}>
              <label htmlFor="firstName" className={styles.label}>
                First Name *
              </label>
              <input
                type="text"
                id="firstName"
                name="firstName"
                value={formData.firstName}
                onChange={handleChange}
                className={styles.input}
                required
              />
            </div>

            <div className={styles.formGroup}>
              <label htmlFor="lastName" className={styles.label}>
                Last Name
              </label>
              <input
                type="text"
                id="lastName"
                name="lastName"
                value={formData.lastName}
                onChange={handleChange}
                className={styles.input}
              />
            </div>
          </div>

          <div className={styles.formGroup}>
            <label htmlFor="email" className={styles.label}>
              Email *
            </label>
            <input
              type="email"
              id="email"
              name="email"
              value={formData.email}
              onChange={handleChange}
              className={styles.input}
              required
            />
          </div>

          <div className={styles.formGroup}>
            <label htmlFor="phone" className={styles.label}>
              Phone
            </label>
            <input
              type="tel"
              id="phone"
              name="phone"
              value={formData.phone}
              onChange={handleChange}
              className={styles.input}
              placeholder="+1 (555) 123-4567"
            />
          </div>

          <div className={styles.formGroup}>
            <label htmlFor="expertiseLevel" className={styles.label}>
              Expertise Level *
            </label>
            <select
              id="expertiseLevel"
              name="expertiseLevel"
              value={formData.expertiseLevel}
              onChange={handleChange}
              className={styles.select}
              required
            >
              {EXPERTISE_LEVELS.map((level) => (
                <option key={level.value} value={level.value}>
                  {level.label}
                </option>
              ))}
            </select>
          </div>

          {error && <div className={styles.error}>{error}</div>}

          <div className={styles.footer}>
            <button
              type="button"
              onClick={onClose}
              className={styles.cancelButton}
              disabled={loading}
            >
              Cancel
            </button>
            <button
              type="submit"
              className={styles.submitButton}
              disabled={loading}
            >
              {loading ? "Updating..." : "Update Profile"}
            </button>
          </div>
        </form>
      </div>
    </div>
  );
};
