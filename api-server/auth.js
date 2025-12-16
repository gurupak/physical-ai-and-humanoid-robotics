const { betterAuth } = require("better-auth");
const { Pool } = require("pg");
const path = require("path");
require("dotenv").config({ path: path.join(__dirname, ".env") });

// Create PostgreSQL connection pool
const pool = new Pool({
  connectionString: process.env.DATABASE_URL,
});

// Create a simple auth configuration for production
const auth = betterAuth({
  database: pool,
  secret: process.env.BETTER_AUTH_SECRET,

  // ✅ REQUIRED: canonical backend URL
  baseURL:
    process.env.BETTER_AUTH_BASE_URL ||
    "https://dynamic-courage-production.up.railway.app",

  // ✅ REQUIRED: cross-site OAuth cookies
  // cookies: {
  //   session: {
  //     name: "__Secure-better-auth.session",
  //     options: {
  //       httpOnly: true,
  //       secure: true,
  //       sameSite: "none",
  //       path: "/",
  //     },
  //   },
  //   state: {
  //     name: "__Secure-better-auth.state",
  //     options: {
  //       httpOnly: true,
  //       secure: true,
  //       sameSite: "none",
  //       path: "/",
  //     },
  //   },
  // },

  user: {
    additionalFields: {
      phone: { type: "string", required: false, fieldName: "phone" },
      expertiseLevel: {
        type: "string",
        required: false,
        defaultValue: "beginner",
        fieldName: "expertise_level",
      },
      firstName: { type: "string", required: false, fieldName: "first_name" },
      lastName: { type: "string", required: false, fieldName: "last_name" },
    },
  },

  emailAndPassword: {
    enabled: true,
    requireEmailVerification: false,
  },

  socialProviders: {
    google: {
      clientId: process.env.GOOGLE_CLIENT_ID,
      clientSecret: process.env.GOOGLE_CLIENT_SECRET,
    },
    github: process.env.GITHUB_CLIENT_ID
      ? {
          clientId: process.env.GITHUB_CLIENT_ID,
          clientSecret: process.env.GITHUB_CLIENT_SECRET,
        }
      : undefined,
  },
  advanced: {
    useSecureCookies: true, // Forces Secure flag even if not in production mode explicitly
    defaultCookieAttributes: {
      sameSite: "none", // Required for cross-origin requests
      secure: true,
      httpOnly: true,
    },
  },

  trustedOrigins: process.env.TRUSTED_ORIGINS?.split(",") || [
    "https://gurupak.github.io",
    "https://gurupak.github.io/physical-ai-and-humanoid-robotics",
  ],
});

console.log("Better Auth server setup complete");
module.exports = { auth };
