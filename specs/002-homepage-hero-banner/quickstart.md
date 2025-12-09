# Quickstart: Homepage Hero Banner Testing

**Feature**: 002-homepage-hero-banner  
**Time to Complete**: 10-15 minutes  
**Goal**: Verify hero banner displays correctly on homepage with responsive design and accessibility

---

## Prerequisites

- ✅ Node.js 18+ installed
- ✅ Repository cloned and on `002-homepage-hero-banner` branch
- ✅ Dependencies installed (`npm install` already run from 001-docusaurus-init)
- ✅ Hero banner and logo images available in `static/img/`

---

## Quick Test Steps

### 1. Start Development Server

```bash
cd D:\workspace\nextjs\hackathon-book
npm start
```

**Expected Output**:
```
[INFO] Starting the development server...
[SUCCESS] Docusaurus website is running at http://localhost:3000/physical-ai-and-humanoid-robotics/
```

**Troubleshooting**:
- Port 3000 already in use? Run `npm start -- --port 3001`
- Build errors? Run `npm run clear` then `npm start`

---

### 2. Verify Hero Banner on Homepage

**Open**: http://localhost:3000/physical-ai-and-humanoid-robotics/

**Visual Checklist**:
- [ ] Hero section appears at top of page (before intro content)
- [ ] Book title "Physical AI & Humanoid Robotics" displays in large font
- [ ] Tagline "A comprehensive guide to building intelligent robots" displays below title
- [ ] Background image (humanoid robot) visible and properly positioned
- [ ] Two CTA buttons visible: "Explore the Book" (primary) and "Start Reading" (secondary)

**Screenshot Test**:
Take screenshot and compare against design mockup.

---

### 3. Test CTA Button Navigation

**Test Primary CTA ("Explore the Book")**:
1. Click "Explore the Book" button
2. Verify: Sidebar opens/scrolls to Table of Contents (or stays on same page showing TOC in sidebar)

**Test Secondary CTA ("Start Reading")**:
1. Click "Start Reading" button
2. Verify: Page navigates to `/intro` (Introduction page)
3. Verify: Navigation is instant (SPA routing, no page reload)

**Troubleshooting**:
- 404 error on `/intro`? Check `docs/intro.md` exists
- Button does nothing? Check browser console for React errors

---

### 4. Test Responsive Design

**Desktop (1920x1080)**:
1. Open browser DevTools (F12)
2. Set viewport to 1920x1080px
3. Verify:
   - [ ] Hero section visible without scrolling
   - [ ] Title font size ~3rem (48px)
   - [ ] Buttons side-by-side horizontally
   - [ ] Background image covers full width

**Tablet (768x1024)**:
1. Set DevTools viewport to 768x1024px (iPad)
2. Verify:
   - [ ] Title font size ~2rem (32px)
   - [ ] Padding reduced (less whitespace)
   - [ ] Buttons still side-by-side
   - [ ] No horizontal scrolling

**Mobile (375x667)**:
1. Set DevTools viewport to 375x667px (iPhone SE)
2. Verify:
   - [ ] Title font size ~1.5rem (24px)
   - [ ] Buttons stack vertically
   - [ ] Buttons fill full width
   - [ ] Text remains readable
   - [ ] No horizontal scrolling

**Troubleshooting**:
- Horizontal scroll appears? Check CSS `overflow-x: hidden` on body
- Text too small on mobile? Verify media query breakpoints (768px, 576px)

---

### 5. Test Dark Mode Toggle

**Toggle Dark Mode**:
1. Click moon/sun icon in navbar (Docusaurus dark mode toggle)
2. Verify:
   - [ ] Hero background color changes to dark variant
   - [ ] Text remains readable (sufficient contrast)
   - [ ] Buttons adapt to dark theme
   - [ ] Background image opacity/overlay adjusts if needed

**Contrast Check**:
- Light mode: White text on dark blue should have ≥ 4.5:1 ratio
- Dark mode: Light text on darker background should have ≥ 4.5:1 ratio

**Troubleshooting**:
- Text invisible in dark mode? Check CSS custom properties `[data-theme='dark']`

---

### 6. Test Keyboard Navigation (Accessibility)

**Keyboard-Only Test**:
1. Refresh page and close mouse/trackpad tab
2. Press `Tab` key repeatedly
3. Verify:
   - [ ] Focus indicator visible on first CTA button ("Explore the Book")
   - [ ] Press `Tab` again → focus moves to second CTA ("Start Reading")
   - [ ] Press `Enter` on focused button → navigates to correct page
   - [ ] Focus outline clearly visible (not removed by CSS)

**Screen Reader Test (Optional)**:
- Windows: Enable Narrator (Win + Ctrl + Enter)
- macOS: Enable VoiceOver (Cmd + F5)
- Verify: Hero section announced as "banner" landmark, title read correctly

**Troubleshooting**:
- No focus indicator? Check Docusaurus `.button:focus` styles are not overridden
- Tab order wrong? Verify HTML structure (buttons should be in DOM order)

---

### 7. Test Image Loading and Performance

**Check Image Assets**:
1. Open DevTools → Network tab
2. Refresh page
3. Verify:
   - [ ] `hero-banner.jpg` or `hero-banner.webp` loads successfully (200 status)
   - [ ] Image size < 200KB (check file size in Network tab)
   - [ ] Logo `logo.png` loads if present
   - [ ] No 404 errors for missing images

**Performance Audit**:
1. Open DevTools → Lighthouse tab
2. Select "Performance" and "Accessibility" categories
3. Run audit
4. Verify:
   - [ ] Performance score > 90
   - [ ] Accessibility score > 90
   - [ ] LCP (Largest Contentful Paint) < 2.5s
   - [ ] No console errors or warnings

**Troubleshooting**:
- Slow LCP? Compress hero banner image (use WebP, target <200KB)
- Low accessibility score? Check for missing alt text, low contrast, or broken focus indicators

---

### 8. Test on Different Browsers (Cross-Browser)

**Browsers to Test**:
- [x] Chrome/Edge (Chromium)
- [ ] Firefox
- [ ] Safari (if on macOS)

**Per-Browser Checklist**:
- [ ] Hero displays correctly
- [ ] CTAs clickable and navigate
- [ ] Responsive breakpoints work
- [ ] Dark mode toggles
- [ ] No console errors

**Troubleshooting**:
- Firefox: Check for CSS Grid/Flexbox inconsistencies
- Safari: Check for WebP support (Safari 14+ supports WebP)

---

## Build and Preview (Production Test)

**Build for Production**:
```bash
npm run build
```

**Expected Output**:
```
[SUCCESS] Generated static files in "build".
[INFO] Use `npm run serve` to test the build locally.
```

**Serve Build Locally**:
```bash
npm run serve
```

**Open**: http://localhost:3000/physical-ai-and-humanoid-robotics/

**Production Checklist**:
- [ ] Hero banner displays identically to dev server
- [ ] Images load from `/img/` path (not broken links)
- [ ] CSS styles applied correctly (no missing styles)
- [ ] No console errors
- [ ] Page loads fast (< 2 seconds on localhost)

---

## Common Issues and Fixes

### Issue 1: Hero Section Not Appearing

**Symptoms**: Homepage shows intro content but no hero banner

**Causes**:
- Swizzled component not wrapping correctly
- Homepage detection logic failing

**Fix**:
1. Check `src/theme/DocItem/Layout/index.tsx` exists
2. Verify `isHomepage` logic: `metadata.slug === '/' || metadata.id === 'intro'`
3. Check browser console for React errors

### Issue 2: Background Image Not Loading

**Symptoms**: Hero section shows but background is blank/solid color

**Causes**:
- Image path incorrect
- Image file missing from `static/img/`

**Fix**:
1. Verify file exists: `D:\workspace\nextjs\hackathon-book\static\img\hero-banner.jpg`
2. Check CSS path: `url('/img/hero-banner.jpg')` (leading slash is crucial)
3. Clear browser cache (Ctrl + Shift + R)

### Issue 3: Buttons Stack on Desktop

**Symptoms**: CTA buttons appear vertically on large screens

**Causes**:
- Media query breakpoint incorrect
- Flexbox direction not set

**Fix**:
1. Check `.heroButtons` CSS: `display: flex; flex-direction: row;`
2. Verify mobile media query only applies below 768px
3. Check for conflicting Docusaurus button styles

### Issue 4: Text Unreadable in Dark Mode

**Symptoms**: Hero text disappears or low contrast in dark mode

**Causes**:
- CSS custom properties not defined for dark theme
- Background color too similar to text color

**Fix**:
1. Add `[data-theme='dark']` selector in `custom.css`
2. Ensure contrast ratio ≥ 4.5:1 (use contrast checker tool)
3. Test with dark mode enabled in browser

### Issue 5: Horizontal Scroll on Mobile

**Symptoms**: Page wider than viewport on mobile devices

**Causes**:
- Fixed widths exceeding viewport
- Background image not constrained

**Fix**:
1. Add `overflow-x: hidden` to body in `custom.css`
2. Check hero section has `max-width: 100%`
3. Verify background image uses `background-size: cover` not `contain`

---

## Next Steps After Quickstart

**If All Tests Pass** ✅:
1. Commit changes to `002-homepage-hero-banner` branch
2. Push to GitHub and create Pull Request
3. Verify hero banner on deployed GitHub Pages preview

**If Tests Fail** ❌:
1. Review error messages in browser console
2. Check implementation against `plan.md` design
3. Consult Docusaurus documentation for component swizzling
4. Ask for clarification if specification unclear

---

## Performance Baseline

**Expected Metrics** (Lighthouse on localhost):
- Performance: 95-100
- Accessibility: 95-100
- Best Practices: 95-100
- SEO: 90-100

**Load Time Breakdown**:
- Initial HTML: < 50ms
- Hero image (WebP 200KB): < 500ms on localhost
- CSS Modules: < 50ms
- React hydration: < 200ms
- **Total LCP**: < 1s on localhost, < 2.5s on production

**If Metrics Below Target**:
- Compress hero image further (target 100-150KB)
- Use next-gen formats (WebP, AVIF)
- Enable GitHub Pages CDN caching (automatic)

---

## Success Criteria

**Definition of Done**:
- [x] Hero banner visible on homepage only (not on other doc pages)
- [x] All responsive breakpoints functional (desktop, tablet, mobile)
- [x] Dark mode compatible with sufficient contrast
- [x] Keyboard navigation works (Tab, Enter, Space)
- [x] CTA buttons navigate to correct pages
- [x] Images load correctly and meet performance budget (<200KB)
- [x] Lighthouse scores > 90 (Performance & Accessibility)
- [x] No console errors or warnings

**When Ready**:
Proceed to implementation tasks in `tasks.md` (generated by `/sp.tasks` command).
