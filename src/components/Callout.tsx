import React from 'react';
import styles from './Callout.module.css';

export interface CalloutProps {
  type?: 'note' | 'tip' | 'warning' | 'danger' | 'info';
  title?: string;
  icon?: string;
  children: React.ReactNode;
}

const defaultIcons = {
  note: '📝',
  tip: '💡',
  warning: '⚠️',
  danger: '🚨',
  info: 'ℹ️',
};

const Callout: React.FC<CalloutProps> = ({
  type = 'note',
  title,
  icon,
  children,
}) => {
  const displayIcon = icon || defaultIcons[type];

  return (
    <div className={`${styles.callout} ${styles[type]}`} role="note" aria-label={`${type} callout`}>
      <div className={styles.calloutHeader}>
        <span className={styles.calloutIcon}>{displayIcon}</span>
        {title && <strong className={styles.calloutTitle}>{title}</strong>}
      </div>
      <div className={styles.calloutContent}>{children}</div>
    </div>
  );
};

export default Callout;
