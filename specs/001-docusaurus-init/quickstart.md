# Quickstart: Docusaurus 3.9.x Site Setup

**Feature**: 001-docusaurus-init
**Date**: 2025-12-03
**Audience**: Developers setting up the project for the first time

## Prerequisites

Before starting, ensure you have:
- **Node.js 20.x** installed ([download](https://nodejs.org/))
- **Git** installed for version control
- **GitHub account** with repository access
- **Text editor** (VS Code, WebStorm, etc.)

Verify Node.js version:
```bash
node --version
# Should output v20.x.x
```

## 5-Minute Setup

### 1. Clone and Install (2 minutes)

```bash
# Clone the repository
git clone https://github.com/<username>/hackathon-book.git
cd hackathon-book

# Install dependencies
npm install
```

Expected output: Dependencies install without errors, `node_modules/` folder created.

### 2. Start Development Server (1 minute)

```bash
npm start
```

Expected output:
```
[INFO] Starting the development server...
[SUCCESS] Docusaurus website is running at: http://localhost:3000/
```

Open browser to http://localhost:3000/hackathon-book/ and verify:
- ✅ Homepage loads with title "Physical AI & Humanoid Robotics"
- ✅ Tagline displays: "A comprehensive guide to building intelligent robots"
- ✅ Sidebar shows "Introduction" document
- ✅ No console errors in browser DevTools

### 3. Test Hot Reload (1 minute)

1. Open `docs/intro.md` in your editor
2. Change the first heading to "# Welcome to Robotics!"
3. Save the file
4. Browser automatically refreshes and shows the change

### 4. Build for Production (1 minute)

Stop the dev server (Ctrl+C), then:

```bash
npm run build
```

Expected output:
```
[SUCCESS] Generated static files in "build".
```

Verify: `build/` directory contains `index.html` and assets.

### 5. Preview Production Build (Optional)

```bash
npm run serve
```

Visit http://localhost:3000/hackathon-book/ to preview the production build.

---

## Project Structure Overview

```
hackathon-book/
├── docs/                    # 📄 Your chapter content goes here
│   └── intro.md            # Placeholder intro document
├── src/
│   ├── components/         # 🧩 Future custom React components
│   └── css/
│       └── custom.css      # 🎨 Custom styles (Tailwind future)
├── static/
│   └── img/                # 🖼️ Images and static assets
├── .github/
│   └── workflows/
│       └── deploy.yml      # 🚀 GitHub Actions deployment
├── docusaurus.config.ts    # ⚙️ Site configuration
├── sidebars.ts             # 📑 Sidebar navigation
├── package.json            # 📦 Dependencies and scripts
└── tsconfig.json           # 🔧 TypeScript configuration
```

---

## Common Commands

| Command | Purpose | When to Use |
|---------|---------|-------------|
| `npm install` | Install dependencies | First time setup, after pulling changes |
| `npm start` | Start dev server | Local development and content writing |
| `npm run build` | Build for production | Before deployment, to test build |
| `npm run serve` | Preview production build | Verify build before pushing |
| `npm run clear` | Clear Docusaurus cache | If you see stale content or errors |

---

## Adding a New Chapter

1. Create a new file in `docs/`, e.g., `docs/ros2-basics.md`
2. Add frontmatter:
```markdown
---
id: ros2-basics
title: "Chapter 1: ROS 2 Basics"
sidebar_label: "ROS 2 Basics"
sidebar_position: 2
---

# ROS 2 Basics

Your content here...
```
3. Save the file
4. Sidebar automatically updates with the new chapter

---

## GitHub Pages Deployment

### Automatic Deployment

Every push to the `main` branch triggers automatic deployment:

1. Push changes: `git push origin main`
2. Visit **Actions** tab in GitHub repository
3. Wait for "Deploy to GitHub Pages" workflow to complete (~2-3 minutes)
4. Visit your site at: `https://<username>.github.io/hackathon-book/`

### First-Time Setup Requirements

Before automatic deployment works, ensure:

1. **GitHub Pages is enabled** in repository settings:
   - Go to Settings → Pages
   - Source: "Deploy from a branch"
   - Branch: `gh-pages` (will be auto-created by workflow)

2. **Workflow permissions** are correct:
   - Go to Settings → Actions → General
   - Workflow permissions: "Read and write permissions"

3. **Repository is public** (or GitHub Pro for private repos)

---

## Troubleshooting

### Issue: "Module not found" errors

**Solution**: Run `npm install` to ensure all dependencies are installed.

### Issue: Dev server won't start

**Solution**:
1. Check if port 3000 is already in use
2. Kill the process using port 3000
3. Try starting again: `npm start`

### Issue: Changes not appearing in browser

**Solution**:
1. Hard refresh: Ctrl+Shift+R (Windows/Linux) or Cmd+Shift+R (Mac)
2. Clear Docusaurus cache: `npm run clear`
3. Restart dev server: `npm start`

### Issue: Build fails with TypeScript errors

**Solution**:
1. Check `docusaurus.config.ts` for syntax errors
2. Ensure all imports are correct
3. Run `npm run build` to see detailed error messages
4. Fix errors and rebuild

### Issue: GitHub Actions deployment fails

**Solution**:
1. Check workflow logs in Actions tab
2. Verify GitHub Pages is enabled in repository settings
3. Ensure workflow permissions include "contents: write"
4. Check if `baseUrl` in `docusaurus.config.ts` matches repository name

### Issue: Site loads but CSS/JS 404 errors

**Solution**: Verify `baseUrl` in `docusaurus.config.ts`:
```typescript
baseUrl: '/hackathon-book/',  // Must match repository name
```

---

## Configuration Cheat Sheet

### docusaurus.config.ts Key Settings

```typescript
export default {
  // Site metadata
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'A comprehensive guide to building intelligent robots',

  // GitHub Pages configuration
  url: 'https://<username>.github.io',
  baseUrl: '/hackathon-book/',
  organizationName: '<username>',
  projectName: 'hackathon-book',

  // Docs-only mode
  presets: [
    [
      'classic',
      {
        docs: {
          routeBasePath: '/',  // Serve docs at root
        },
        blog: false,  // Disable blog
      },
    ],
  ],
};
```

---

## Next Steps

After completing this quickstart:

1. ✅ **Verify deployment**: Check that your site is live on GitHub Pages
2. ✅ **Read the spec**: Review `specs/001-docusaurus-init/spec.md` for full requirements
3. ✅ **Plan next feature**: Consider Tailwind CSS integration or first chapter content
4. ✅ **Explore Docusaurus**: Read [official docs](https://docusaurus.io/) for advanced features

---

## Getting Help

- **Docusaurus Issues**: Check [Docusaurus Troubleshooting](https://docusaurus.io/docs/installation#problems)
- **GitHub Pages**: See [GitHub Pages docs](https://docs.github.com/en/pages)
- **Project Issues**: Open an issue in the repository

---

## Performance Benchmarks

Expected timings (on typical development machine):

| Operation | Expected Time |
|-----------|---------------|
| `npm install` (first time) | 60-90 seconds |
| `npm install` (cached) | 10-20 seconds |
| `npm start` (cold start) | 15-30 seconds |
| `npm start` (warm start) | 5-10 seconds |
| `npm run build` | 30-60 seconds |
| GitHub Actions deployment | 2-3 minutes |
| Page load (GitHub Pages) | 1-3 seconds |

If your timings significantly exceed these, check:
- Internet connection speed (affects npm install)
- System resources (CPU, RAM availability)
- Antivirus interference (may slow file operations)

---

**✅ Quickstart Complete!** You now have a working Docusaurus site with GitHub Pages deployment.
