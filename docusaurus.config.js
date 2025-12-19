// @ts-check
// `@ts-check` enables ts-checking for the config file

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI Humanoid Robotics',
  tagline: 'Educational content for AI engineers and robotics students',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://cz-3.github.io', // Update this to your GitHub Pages URL
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub Pages, this is usually '/<repo-name>/'
  baseUrl: '/Robotic/',
// onBrokenLinks aur onBrokenMarkdownLinks ki lines hata do (ya comment kar do) kyunki ab zarurat nahi

  // GitHub pages deployment config.
  organizationName: 'cz-3', // Usually your GitHub org/user name.
  projectName: 'Robotic', // Usually your repo name.
  deploymentBranch: 'gh-pages', // Branch that GitHub Pages will deploy from.

  // SEO and social sharing metadata
  trailingSlash: false,
  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          path: './docs',
          sidebarPath: require.resolve('./sidebars.js'),
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/cz-3/Robotic/edit/main/',
          showLastUpdateTime: true,
          showLastUpdateAuthor: true,
          breadcrumbs: true, // Add breadcrumbs navigation
        },
        blog: false, // Optional: disable the blog plugin
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
        gtag: {
          trackingID: 'G-XXXXXXXXXX', // Optional: Google Analytics
          anonymizeIP: true,
        },
      }),
    ],
  ],

  stylesheets: [
    {
      href: 'https://fonts.googleapis.com/css2?family=Inter:wght@300;400;500;600;700&display=swap',
      rel: 'stylesheet',
    },
  ],

  themes: [
    // ... Your other themes
    // [
    //   require.resolve("@easyops-cn/docusaurus-search-local"),
    //   /** @type {import("@easyops-cn/docusaurus-search-local").PluginOptions} */
    //   ({
    //     // ... Your options.
    //     // `hashed` is recommended as long-term-cache & improved performance
    //     hashed: true,
    //     // For Docs using Chinese, The `language` is recommended to set to:
    //     // ```
    //     // language: ["en", "zh"],
    //     // ```
    //     language: ["en"],
    //     // Optional: To index docs by default
    //     indexDocs: true,
    //     // Optional: To index blog by default
    //     indexBlog: false,
    //     // Optional: To index pages by default
    //     indexPages: false,
    //   }),
    // ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/docusaurus-social-card.jpg',
      colorMode: {
        defaultMode: 'light',
        disableSwitch: false,
        respectPrefersColorScheme: true,
      },
      navbar: {
        title: 'Physical AI Humanoid Robotics',
        logo: {
          alt: 'Robotics Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            type: 'doc',
            docId: 'intro',
            position: 'left',
            label: 'Documentation',
          },
          {
            href: 'https://github.com/cz-3/Robotic',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Docs',
            items: [
              {
                label: 'Tutorial',
                to: '/docs/ros2-concepts',
              },
              {
                label: 'ROS 2 Concepts',
                to: '/docs/ros2-concepts/introduction',
              },
              {
                label: 'Python-ROS Integration',
                to: '/docs/python-ros-integration/intro-rclpy',
              },
              {
                label: 'URDF Humanoid Modeling',
                to: '/docs/urdf-humanoid-modeling/intro-urdf',
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'Stack Overflow',
                href: 'https://stackoverflow.com/questions/tagged/ros2',
              },
              {
                label: 'ROS Answers',
                href: 'https://answers.ros.org/questions/',
              },
              {
                label: 'ROS Discourse',
                href: 'https://discourse.ros.org/',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/cz-3/Robotic',
              },
              {
                label: 'ROS Official',
                href: 'https://www.ros.org/',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} ROS 2 Robot Control Module. Built with Docusaurus.`,
      },
      prism: {
        theme: require('prism-react-renderer').themes.github,
        darkTheme: require('prism-react-renderer').themes.dracula,
        additionalLanguages: ['python', 'bash', 'yaml'],
      },
      docs: {
        sidebar: {
          hideable: true,
          autoCollapseCategories: true,
        },
      },
      tableOfContents: {
        minHeadingLevel: 2,
        maxHeadingLevel: 4,
      },
    }),
};

module.exports = config;