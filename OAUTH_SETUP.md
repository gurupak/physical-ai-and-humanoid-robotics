# OAuth Setup Guide

This guide will help you configure Google and GitHub OAuth for the authentication system.

## 🔧 Quick Setup

### 1. Google OAuth Setup

1. **Go to Google Cloud Console**: https://console.cloud.google.com/
2. **Create or Select a Project**
3. **Enable Google+ API**:
   - Navigate to "APIs & Services" → "Library"
   - Search for "Google+ API"
   - Click "Enable"

4. **Create OAuth 2.0 Credentials**:
   - Go to "APIs & Services" → "Credentials"
   - Click "Create Credentials" → "OAuth client ID"
   - Choose "Web application"

5. **Configure OAuth Consent Screen** (if prompted):
   - User Type: External
   - App name: "Physical AI & Humanoid Robotics"
   - User support email: your-email@example.com
   - Developer contact: your-email@example.com

6. **Add Authorized Origins**:
   ```
   http://localhost:3000
   http://localhost:3008
   https://gurupak.github.io
   ```

7. **Add Authorized Redirect URIs**:
   ```
   http://localhost:3008/api/auth/callback/google
   https://your-production-api.vercel.app/api/auth/callback/google
   ```

8. **Copy Credentials**:
   - Client ID: `xxxxx.apps.googleusercontent.com`
   - Client Secret: `GOCSPX-xxxxx`

---

### 2. GitHub OAuth Setup

1. **Go to GitHub Settings**: https://github.com/settings/developers
2. **Click "OAuth Apps"** → **"New OAuth App"**
3. **Fill in Application Details**:
   - **Application name**: `Physical AI Textbook (Dev)`
   - **Homepage URL**: `http://localhost:3000`
   - **Authorization callback URL**: `http://localhost:3008/api/auth/callback/github`
   - **Description**: Development OAuth app

4. **Click "Register application"**

5. **Generate Client Secret**:
   - Click "Generate a new client secret"
   - **Copy immediately** (you won't see it again!)

6. **Copy Credentials**:
   - Client ID: `Ov23xxxxx`
   - Client Secret: `xxxxx`

7. **Create Production OAuth App** (repeat steps 2-6 with):
   - **Application name**: `Physical AI Textbook (Production)`
   - **Homepage URL**: `https://gurupak.github.io/physical-ai-and-humanoid-robotics`
   - **Authorization callback URL**: `https://your-production-api.vercel.app/api/auth/callback/github`

---

### 3. Update Environment Variables

1. **Open** `api-server/.env`

2. **Add your credentials**:
   ```bash
   # Keep existing secret
   BETTER_AUTH_SECRET=x7vRqr9lVr9VUw76LNwfHc7aB1hcEQi8G6zYZuG25/U=

   # Add Google OAuth
   GOOGLE_CLIENT_ID=xxxxx.apps.googleusercontent.com
   GOOGLE_CLIENT_SECRET=GOCSPX-xxxxx

   # Add GitHub OAuth
   GITHUB_CLIENT_ID=Ov23xxxxx
   GITHUB_CLIENT_SECRET=xxxxx
   ```

3. **Save the file**

---

### 4. Restart the Auth Server

```bash
cd api-server
npm run dev
```

You should see:
```
Better Auth server running on http://localhost:3008
CORS enabled for: http://localhost:3000, http://localhost:3002
```

---

### 5. Test Authentication

1. **Refresh your browser**: http://localhost:3000/physical-ai-and-humanoid-robotics/
2. **Click "Sign In"** or **"Explore the Book"**
3. **Try Google or GitHub login**
4. **You should be redirected** to OAuth provider and back

---

## 🔒 Production Setup

### For Vercel Deployment:

1. **Deploy your API server** to Vercel
2. **Get your production API URL** (e.g., `https://your-api.vercel.app`)
3. **Update OAuth redirect URIs** in Google/GitHub to include:
   ```
   https://your-api.vercel.app/api/auth/callback/google
   https://your-api.vercel.app/api/auth/callback/github
   ```
4. **Set environment variables in Vercel**:
   - Go to your Vercel project → Settings → Environment Variables
   - Add all the variables from `.env`

### For GitHub Pages:

1. **Update** `src/lib/auth.ts`:
   ```typescript
   baseURL: process.env.NODE_ENV === 'production'
     ? 'https://your-api.vercel.app'
     : 'http://localhost:3008',
   ```

2. **Rebuild and deploy** your Docusaurus site

---

## ❓ Troubleshooting

### Error: "OAuth not configured"
- Check that all credentials are in `api-server/.env`
- Restart the auth server
- Verify no typos in client IDs/secrets

### Error: "redirect_uri_mismatch"
- Make sure the redirect URI in Google/GitHub matches exactly
- Include the full path: `/api/auth/callback/google` or `/api/auth/callback/github`

### Error: "Invalid OAuth credentials"
- Regenerate client secret
- Make sure you copied the full secret (they're usually very long)

### Still not working?
- Check browser console for errors
- Check auth server logs
- Verify CORS origins in `api-server/auth.js`

---

## 📝 Notes

- **Development**: Use separate OAuth apps for dev and production
- **Security**: Never commit `.env` file to git
- **Testing**: You can test with your personal Google/GitHub account
- **Database**: Currently using SQLite (`auth.db`). Migrate to PostgreSQL for production.

---

## ✅ Checklist

- [ ] Created Google OAuth credentials
- [ ] Created GitHub OAuth credentials  
- [ ] Updated `api-server/.env` with all credentials
- [ ] Restarted auth server
- [ ] Tested Google login
- [ ] Tested GitHub login
- [ ] (Production) Updated redirect URIs
- [ ] (Production) Deployed to Vercel
- [ ] (Production) Set Vercel environment variables
