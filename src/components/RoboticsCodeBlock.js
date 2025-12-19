import React from 'react';
import clsx from 'clsx';
import styles from './RoboticsCodeBlock.module.css';

export default function RoboticsCodeBlock({title, children, language = "bash"}) {
  return (
    <div className={styles.roboticsCodeBlock}>
      <div className={styles.codeHeader}>
        <span className={styles.codeTitle}>{title}</span>
        <span className={styles.codeLanguage}>{language}</span>
      </div>
      <div className={styles.codeContent}>
        {children}
      </div>
    </div>
  );
}