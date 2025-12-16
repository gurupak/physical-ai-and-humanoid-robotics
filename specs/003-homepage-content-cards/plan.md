# Implementation Plan: Homepage Content Cards

**Branch**:  | **Date**: 2025-12-05 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from 
**Note**: This template is filled in by the  command. See  for the execution workflow.

## Summary

Enhance the Docusaurus homepage by replacing large text blocks with a modern, animated hero banner and dashboard-style content cards. The hero banner will feature gradient text effects and staggered animations. Below the hero, content cards will display the four course modules (ROS 2, Gazebo/Unity, NVIDIA Isaac, VLA), learning outcomes, and a 13-week breakdown. Cards will slide into view from left/right using Framer Motion with Intersection Observer, include hover effects (elevation, border glow, icon scaling), and maintain full responsive design with dark mode support via Tailwind CSS synced to Docusaurus theme.

## Technical Context

**Language/Version**: TypeScript 5.x, React 18.x, Node.js 20.x
**Primary Dependencies**: Docusaurus 3.9.2, Framer Motion 11.x, react-intersection-observer 9.x, Tailwind CSS 3.x
**Storage**: Static TypeScript data files in  (no database or API)
**Testing**: Jest + React Testing Library (Docusaurus default), manual testing for animations and responsive design
**Target Platform**: Web browsers (modern evergreen browsers - last 2 versions), static site deployment (GitHub Pages)
**Project Type**: Web (Docusaurus static site)
**Performance Goals**: <2s initial page load, 60fps animations, <100ms animation trigger latency
**Constraints**: Static site only (no server-side rendering or API calls), must work with Docusaurus theme system, Tailwind must not conflict with Docusaurus styles (preflight disabled)
**Scale/Scope**: Single homepage with 10-15 animated cards, 4 core React components, ~500 lines of component code

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

✅ **P1 (Deployment Architecture)**: Static site deployment to GitHub Pages - compliant with split platform (frontend on GitHub Pages)

✅ **P2 (Content Quality)**: Content cards display accurate course information with clear learning outcomes - meets content quality standards

✅ **P3 (Accessibility)**: Tailwind CSS with proper contrast ratios, semantic HTML, keyboard navigation support - meets WCAG 2.1 Level AA

✅ **P4 (Performance)**: Static generation, optimized animations with reduced motion support, lazy loading considered - meets performance goals

✅ **P5 (Internationalization)**: English-only for now, but Docusaurus i18n architecture supports future translation - i18n ready

✅ **P6 (Modern Stack)**: React 18, TypeScript 5, Framer Motion, Tailwind CSS - all modern, well-maintained technologies

✅ **P7 (Component Reusability)**: AnimatedCard wrapper is reusable across all card types - promotes code reuse

✅ **P8 (Testing Coverage)**: Manual testing for animations, Jest for component logic - adequate for UI-heavy feature

✅ **P9 (Documentation)**: Comprehensive quickstart guide, data model documentation, component contracts - well documented

✅ **P10 (Dark Mode)**: Full dark mode support with Tailwind synced to Docusaurus theme - meets dark mode requirement

✅ **P11 (Mobile Responsive)**: Responsive grid with breakpoints (mobile 1 col, tablet 2 col, desktop 3-4 col) - fully responsive

✅ **P12 (SEO Optimization)**: Static generation ensures good SEO, semantic HTML structure - SEO friendly

✅ **P13 (Error Handling)**: Graceful degradation for images, reduced motion support - proper error handling

✅ **P14 (Code Quality)**: TypeScript with strict interfaces, component contracts, data validation - high code quality

✅ **P15 (Security)**: No user input, no API calls, static content only - inherently secure

✅ **P16 (RAG Chatbot)**: Not applicable to this feature (future integration)

**All constitution principles are satisfied.** No violations to track.

## Project Structure

### Documentation (this feature)

\
### Source Code (repository root)

\
**Structure Decision**: This is a web application (Docusaurus static site) with frontend components only. The structure follows Docusaurus conventions with components in , pages in , and static data in . No backend or API needed. Components are organized by feature (AnimatedCard, ModuleCard, etc.) with barrel exports for clean imports. Data files use TypeScript constants for type safety and static site generation.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

**No violations detected.** All constitution principles are satisfied. This feature aligns with the project's static site architecture, maintains high code quality, and follows modern web development best practices.
