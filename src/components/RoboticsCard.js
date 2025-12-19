import React from 'react';
import clsx from 'clsx';
import styles from './RoboticsCard.module.css';

const FeatureList = [
  {
    title: 'Gazebo Simulation',
    description: (
      <>
        Physical-based simulation with realistic physics engine for humanoid robots.
      </>
    ),
    icon: '🤖',
  },
  {
    title: 'Unity Digital Twins',
    description: (
      <>
        High-fidelity digital twins with advanced Human-Robot Interaction (HRI).
      </>
    ),
    icon: '🎮',
  },
  {
    title: 'Sensor Simulation',
    description: (
      <>
        LIDAR, Depth Camera, and IMU sensor simulation for perception validation.
      </>
    ),
    icon: '📡',
  },
];

function Feature({icon, title, description}) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <div className={styles.featureIcon}>{icon}</div>
      </div>
      <div className="text--center padding-horiz--md">
        <h3>{title}</h3>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function RoboticsCard() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}