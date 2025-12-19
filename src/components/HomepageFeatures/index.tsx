import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  Svg: React.ComponentType<React.ComponentProps<'svg'>>;
  description: ReactNode;
  link: string;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Module 1: The Robotic Nervous System',
    Svg: require('@site/static/img/undraw_docusaurus_mountain.svg').default,
    description: (
      <>
        Learn ROS 2 Core Concepts including nodes, topics, services, and messages for building the foundation of robot control systems.
      </>
    ),
    link: '/docs/intro',
  },
  {
    title: 'Module 2: The Digital Twin',
    Svg: require('@site/static/img/undraw_docusaurus_tree.svg').default,
    description: (
      <>
        Physical-based simulation with Gazebo, high-fidelity digital twins and Human-Robot Interaction using Unity, and sensor simulation (LIDAR, DEPTH CAMERA, IMU).
      </>
    ),
    link: '/docs/intro',
  },
  {
    title: 'Module 3: The AI Robot Brain',
    Svg: require('@site/static/img/undraw_docusaurus_react.svg').default,
    description: (
      <>
        Advanced AI concepts for robot decision-making, planning, and control using modern machine learning approaches.
      </>
    ),
    link: '/docs/intro',
  },
  {
    title: 'Module 4: Vision-Language-Action (VLA)',
    Svg: require('@site/static/img/undraw_docusaurus_react.svg').default,
    description: (
      <>
        Integration of perception, cognition, and action systems for autonomous humanoid robots using state-of-the-art AI techniques.
      </>
    ),
    link: '/docs/intro',
  },
];

function Feature({title, description, link}: FeatureItem) {
  return (
    <div className={clsx('col col--3')}>
      <div className="module-card text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
        <a href={link} className="card-button">Read Documentation</a>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
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
