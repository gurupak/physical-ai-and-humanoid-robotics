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
  // Using default basePath: "/api/auth"
  user: {
    additionalFields: {
      phone: {
        type: "string",
        required: false,
        fieldName: "phone",
      },
      expertiseLevel: {
        type: "string",
        required: false,
        defaultValue: "beginner",
        fieldName: "expertise_level",
      },
      firstName: {
        type: "string",
        required: false,
        fieldName: "first_name",
      },
      lastName: {
        type: "string",
        required: false,
        fieldName: "last_name",
      },
    },
  },
  emailAndPassword: {
    enabled: true,
    requireEmailVerification: false,
  },
  socialProviders: {
    github: process.env.GITHUB_CLIENT_ID
      ? {
          clientId: process.env.GITHUB_CLIENT_ID,
          clientSecret: process.env.GITHUB_CLIENT_SECRET,
        }
      : undefined,
    google: process.env.GOOGLE_CLIENT_ID
      ? {
          clientId: process.env.GOOGLE_CLIENT_ID,
          clientSecret: process.env.GOOGLE_CLIENT_SECRET,
        }
      : undefined,
  },
  trustedOrigins: [
    "http://localhost:3000",
    "http://localhost:3002",
    "https://gurupak.github.io",
    "https://gurupak.github.io/physical-ai-and-humanoid-robotics",
  ],
});

console.log("Better Auth server setup complete");
module.exports = { auth };
