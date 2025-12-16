import { themes as prismThemes } from "prism-react-renderer";
import type { Config } from "@docusaurus/types";
import type * as Preset from "@docusaurus/preset-classic";
import { config as dotenvConfig } from "dotenv";

// Load environment variables from .env file
dotenvConfig();

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

// Debug: Log environment variables
console.log("VITE_AUTH_API_URL:", process.env.VITE_AUTH_API_URL);
console.log("VITE_RAG_API_URL:", process.env.VITE_RAG_API_URL);

const config: Config = {
  title: "Physical AI & Humanoid Robotics",
  tagline: "A comprehensive guide to building intelligent robots",
  favicon: "img/favicon.ico",

  // Set the production url of your site here
  url: "https://gurupak.github.io",
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: "/physical-ai-and-humanoid-robotics/",

  // Custom fields for environment-specific configuration
  customFields: {
    authApiUrl:
      process.env.VITE_AUTH_API_URL ||
      "https://dynamic-courage-production.up.railway.app",
    ragApiUrl:
      process.env.VITE_RAG_API_URL ||
      "https://gregarious-tenderness-production-79e3.up.railway.app",
  },

  // Add client modules to inject config into window
  clientModules: [require.resolve("./src/clientModules/injectConfig.ts")],

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: "gurupak", // Usually your GitHub org/user name.
  projectName: "physical-ai-and-humanoid-robotics", // Usually your repo name.
  githubHost: "github.com",
  onBrokenLinks: "warn",
  onBrokenMarkdownLinks: "warn",

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: "en",
    locales: ["en"],
  },

  presets: [
    [
      "classic",
      {
        docs: {
          sidebarPath: "./sidebars.ts",
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            "https://github.com/gurupak/physical-ai-and-humanoid-robotics/tree/main/",
        },
        blog: false, // Disable blog plugin for docs-only mode
        theme: {
          customCss: "./src/css/custom.css",
        },
      } satisfies Preset.Options,
    ],
  ],

  themes: ["@docusaurus/theme-mermaid"],

  markdown: {
    mermaid: true,
  },

  themeConfig: {
    // Replace with your project's social card
    image: "img/docusaurus-social-card.jpg",
    navbar: {
      title: "Physical AI & Humanoid Robotics",
      logo: {
        alt: "Physical AI Logo",
        src: "img/logo.png",
      },
      items: [
        {
          type: "docSidebar",
          sidebarId: "tutorialSidebar",
          position: "left",
          label: "Book",
          customProps: {
            requiresAuth: true,
          },
        },
        {
          href: "https://github.com/gurupak/physical-ai-and-humanoid-robotics",
          label: "GitHub",
          position: "right",
        },
        // Sign In button is handled by custom Navbar/Content component
      ],
    },
    footer: {
      style: "dark",
      links: [
        {
          title: "Docs",
          items: [
            {
              label: "Introduction",
              to: "/docs/introduction",
            },
          ],
        },
        {
          title: "Community",
          items: [
            {
              label: "GitHub",
              href: "https://github.com/gurupak/physical-ai-and-humanoid-robotics",
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,

  trailingSlash: false,
};

export default config;
