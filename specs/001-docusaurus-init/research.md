# Research: Docusaurus 3.9.x Site Initialization

**Feature**: 001-docusaurus-init
**Date**: 2025-12-03
**Status**: Complete

## Overview

This document consolidates research findings for initializing a Docusaurus 3.9.x documentation site with TypeScript, docs-only mode, and GitHub Pages deployment.

## Key Research Areas

### 1. TypeScript Configuration for Docusaurus 3.x

**Decision**: Use TypeScript with ESM syntax for `docusaurus.config.ts`

**Rationale**:
- Docusaurus 3.x requires TypeScript 5.0+ and ESM syntax (`export default` instead of `module.exports`)
- TypeScript provides type safety and IDE autocompletion for configuration
- Official Docusaurus types available via `@docusaurus/types` and `@docusaurus/preset-classic`

**Implementation Pattern**:
```typescript
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

const config: Config = {
  title: 'My Site',
  favicon: 'img/favicon.ico',

  presets: [
    [
      'classic',
      {
        /* preset config */
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    /* theme config */
  } satisfies Preset.ThemeConfig,
};

export default config;
```

**Required TypeScript Dependencies**:
```bash
npm install --save-dev typescript @docusaurus/module-type-aliases @docusaurus/tsconfig @docusaurus/types
```

**tsconfig.json Configuration**:
```json
{
  "extends": "@docusaurus/tsconfig",
  "compilerOptions": {
    "baseUrl": "."
  }
}
```

**Alternatives Considered**:
- JavaScript configuration (simpler but loses type safety)
- CommonJS syntax (deprecated in Docusaurus 3.x)

**Why TypeScript ESM Was Chosen**:
- Required by Docusaurus 3.x architecture
- Provides better developer experience with autocomplete
- Catches configuration errors at compile time
- Aligns with constitution requirement for TypeScript strict mode

---

### 2. Docs-Only Mode Configuration

**Decision**: Configure docs-only mode with `routeBasePath: '/'` and `blog: false`

**Rationale**:
- Textbook content is documentation, not blog posts
- Serving docs at root (`/`) provides cleaner URLs (e.g., `/intro` instead of `/docs/intro`)
- Removes unnecessary blog UI and reduces bundle size

**Implementation Pattern**:
```typescript
export default {
  presets: [
    [
      '@docusaurus/preset-classic',
      {
        docs: {
          routeBasePath: '/', // Serve docs at site root
          /* other docs plugin options */
        },
        blog: false, // Disable blog plugin
      },
    ],
  ],
};
```

**Alternatives Considered**:
- Default docs mode with `/docs/` prefix (adds unnecessary path segment)
- Hybrid blog + docs mode (blog not needed for textbook)

**Why Docs-Only Was Chosen**:
- Cleaner URLs for educational content
- Removes blog UI clutter
- Aligns with spec requirement (FR-006: docs-only mode)
- Simplifies navigation structure

---

### 3. GitHub Pages Deployment Configuration

**Decision**: Use `peaceiris/actions-gh-pages@v3` action with npm-based workflow

**Rationale**:
- Community-standard action with 15k+ GitHub stars
- Automatically creates and manages `gh-pages` branch
- Simpler than official `actions/deploy-pages@v4` (no artifact upload/download steps)
- Project uses npm (not Yarn) per repository setup

**Implementation Pattern**:
```yaml
name: Deploy to GitHub Pages

on:
  push:
    branches:
      - main

permissions:
  contents: write

jobs:
  deploy:
    name: Deploy to GitHub Pages
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: actions/setup-node@v4
        with:
          node-version: 20
          cache: npm

      - name: Install dependencies
        run: npm ci
      - name: Build website
        run: npm run build

      - name: Deploy to GitHub Pages
        uses: peaceiris/actions-gh-pages@v3
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          publish_dir: ./build
          user_name: github-actions[bot]
          user_email: 41898282+github-actions[bot]@users.noreply.github.com
```

**Alternatives Considered**:
1. **actions/deploy-pages@v4** (official GitHub Pages action)
   - Pros: Official support, artifact-based deployment
   - Cons: More complex workflow (separate build/deploy jobs), requires artifact upload/download

2. **Manual `docusaurus deploy` command**
   - Pros: Built-in Docusaurus command
   - Cons: Requires manual triggering, needs GIT_USER environment variable, SSH key management

**Why peaceiris/actions-gh-pages@v3 Was Chosen**:
- Aligns with spec clarification decision (Q1: "Use peaceiris/actions-gh-pages@v3")
- Simpler single-job workflow
- Automatic branch management (creates `gh-pages` if missing)
- No SSH key or GIT_USER configuration needed
- Works with GITHUB_TOKEN (no secrets to manage)

**Dependency Caching**:
```yaml
- uses: actions/setup-node@v4
  with:
    node-version: 20
    cache: npm  # Aligns with spec clarification Q4
```

---

### 4. GitHub Pages Project Site Configuration

**Decision**: Configure `baseUrl` to match repository name for GitHub Pages project site

**Rationale**:
- GitHub Pages project sites are served at `https://<username>.github.io/<repo-name>/`
- Docusaurus needs `baseUrl` to match this path for correct asset loading
- Mismatched `baseUrl` causes 404 errors for CSS, JS, and images

**Implementation Pattern**:
```typescript
export default {
  url: 'https://<username>.github.io',
  baseUrl: '/<repo-name>/',
  organizationName: '<username>',
  projectName: '<repo-name>',
  trailingSlash: false,
};
```

**For this repository (hackathon-book)**:
```typescript
export default {
  url: 'https://<username>.github.io',
  baseUrl: '/hackathon-book/',
  organizationName: '<username>',
  projectName: 'hackathon-book',
  trailingSlash: false,
};
```

**Alternatives Considered**:
1. **User/Organization site** (`baseUrl: '/'`)
   - Requires repository named `<username>.github.io`
   - Not applicable (repository is `hackathon-book`)

2. **Custom domain** (`baseUrl: '/'`)
   - Requires DNS configuration
   - Out of scope for initial setup

**Why Project Site Configuration Was Chosen**:
- Repository is `hackathon-book` (not `<username>.github.io`)
- GitHub Pages will serve at `/<repo-name>/` path
- Aligns with spec requirement (FR-013)
- No DNS configuration needed

---

### 5. Node.js Version and Dependency Management

**Decision**: Use Node.js 20.x with npm ci for reproducible builds

**Rationale**:
- Node.js 20.x is LTS (Long-Term Support) through April 2026
- Compatible with Docusaurus 3.9.x (requires Node.js 18.0+)
- `npm ci` provides faster, more reliable CI builds than `npm install`

**GitHub Actions Configuration**:
```yaml
- uses: actions/setup-node@v4
  with:
    node-version: 20
    cache: npm
```

**Local Development**:
```bash
# Prerequisites: Node.js 20.x installed
npm install
npm start
```

**Alternatives Considered**:
- Node.js 18.x (older LTS, but 20.x is more current)
- Node.js 22.x (too new, not LTS yet)
- Yarn package manager (project uses npm)

**Why Node.js 20.x + npm ci Was Chosen**:
- Aligns with constitution (IV. Book Platform: "Node.js 18+")
- LTS status ensures long-term stability
- `npm ci` provides reproducible builds (deletes `node_modules`, installs from lock file)
- Spec requires Node.js 20.x (FR-018, Assumption 3)

---

### 6. Intro Document Structure

**Decision**: Create `docs/intro.md` with basic structure including headings and placeholder text

**Rationale**:
- Docusaurus requires at least one document in `docs/` folder
- Basic structure provides template for future content
- Placeholder sections indicate planned content areas

**Implementation Pattern**:
```markdown
---
id: intro
title: Welcome to Physical AI & Humanoid Robotics
sidebar_label: Introduction
sidebar_position: 1
---

# Welcome

[Introductory paragraph placeholder]

## About

Content coming soon.

## Getting Started

Content coming soon.

## Next Steps

Content coming soon.
```

**Alternatives Considered**:
1. **Minimal single-heading document** (too sparse, doesn't show structure)
2. **Full sample content** (out of scope, content is future work)

**Why Basic Structure Was Chosen**:
- Aligns with spec requirement (FR-007: basic structure with placeholder sections)
- Balances between minimal and comprehensive
- Provides clear template for content authors
- Demonstrates Docusaurus markdown features

---

## Configuration Summary

### Required Files

1. **package.json** - Dependencies and scripts
   - `@docusaurus/core: ^3.9.2`
   - `@docusaurus/preset-classic: ^3.9.2`
   - `react: ^18.0.0`
   - `react-dom: ^18.0.0`
   - Dev dependencies: TypeScript tooling

2. **docusaurus.config.ts** - Site configuration
   - Title: "Physical AI & Humanoid Robotics"
   - Tagline: "A comprehensive guide to building intelligent robots"
   - URL: `https://<username>.github.io`
   - baseUrl: `/hackathon-book/`
   - Docs-only mode enabled

3. **tsconfig.json** - TypeScript configuration
   - Extends `@docusaurus/tsconfig`
   - baseUrl set to current directory

4. **sidebars.ts** - Sidebar configuration
   - Autogenerated from docs folder

5. **.github/workflows/deploy.yml** - GitHub Actions workflow
   - Triggers on push to main
   - Uses Node.js 20.x with npm caching
   - Deploys to gh-pages branch

6. **docs/intro.md** - Placeholder intro document
   - Basic structure with section headings
   - Placeholder text for future content

7. **.gitignore** - Ignore patterns
   - node_modules/
   - .docusaurus/
   - build/
   - .cache-loader/

8. **README.md** - Setup instructions
   - Prerequisites: Node.js 20.x
   - Installation: `npm install`
   - Development: `npm start`
   - Build: `npm run build`

9. **src/css/custom.css** - Custom styles (empty initially)

10. **src/components/** - Custom components directory (empty initially)

11. **static/img/** - Static assets directory (empty initially)

---

## Performance Considerations

### Build Time Optimization
- Use `npm ci` instead of `npm install` in CI (faster, reproducible)
- Enable npm caching in GitHub Actions (saves ~30 seconds per build)
- Docusaurus incremental builds in local development

### Runtime Performance
- Static site generation ensures fast page loads
- Docusaurus default optimizations:
  - Code splitting for each route
  - Lazy loading for images
  - Prefetching for internal links
  - Service worker for offline support (optional)

### Deployment Speed
- GitHub Actions workflow completes in ~2-3 minutes:
  - Checkout: ~5 seconds
  - Setup Node.js + cache restore: ~10 seconds
  - npm ci: ~30-60 seconds (with cache)
  - npm run build: ~30-60 seconds
  - Deploy to gh-pages: ~10-20 seconds

---

## Testing Strategy

### Local Development Testing
1. **Installation test**: `npm install` completes without errors
2. **Dev server test**: `npm start` launches server on localhost:3000
3. **Hot reload test**: Edit intro.md and verify browser auto-refreshes
4. **Build test**: `npm run build` completes without TypeScript errors
5. **Preview test**: `npm run serve` previews production build

### GitHub Actions Testing
1. **Workflow trigger test**: Push to main triggers workflow
2. **Build success test**: Workflow completes with green status
3. **Deployment verification**: Visit GitHub Pages URL and confirm site loads
4. **Asset loading test**: Browser console has no 404 errors

### Acceptance Testing
- All acceptance scenarios from spec.md (User Stories 1-3) must pass
- Manual testing checklist covers installation, development, build, and deployment

---

## Security Considerations

### GitHub Actions Permissions
- Workflow uses `permissions: { contents: write }` for gh-pages branch access
- Uses `GITHUB_TOKEN` (automatically provided, scoped to repository)
- No custom secrets needed for basic deployment

### Dependency Security
- Use `npm audit` to scan for vulnerabilities
- Keep dependencies updated (Docusaurus 3.9.x is current release)
- Lock file (`package-lock.json`) ensures reproducible installs

---

## Future Enhancements (Out of Scope)

These items are explicitly deferred to future features:

1. **Tailwind CSS Integration** - Custom styling system (constitution section VII)
2. **Search Functionality** - Algolia DocSearch or local search plugin
3. **Internationalization** - Urdu translation with RTL support
4. **Dark Mode Customization** - Beyond default Docusaurus theme
5. **Analytics Integration** - Google Analytics or Plausible
6. **ChatKit Integration** - RAG chatbot widget
7. **Authentication** - Better-Auth with OAuth providers
8. **API Backend** - Next.js on Vercel for dynamic features
9. **Comprehensive Testing** - E2E tests with Playwright
10. **Performance Monitoring** - Lighthouse CI integration

---

## Conclusion

All technical unknowns from the planning phase have been resolved:

✅ **TypeScript Configuration** - ESM syntax with Docusaurus 3.x types
✅ **Docs-Only Mode** - `routeBasePath: '/'` and `blog: false`
✅ **GitHub Pages Deployment** - peaceiris/actions-gh-pages@v3 workflow
✅ **Project Site Configuration** - baseUrl matching repository name
✅ **Node.js Version** - 20.x with npm ci for CI builds
✅ **Intro Document Structure** - Basic headings with placeholders

Ready to proceed to **Phase 1: Design & Contracts**.
