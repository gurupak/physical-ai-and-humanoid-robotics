// Better Auth server configuration for Docusaurus
// This is a placeholder - you'll need to implement your own auth server

export const authConfig = {
  basePath: '/auth',
  database: {
    // Configure your database connection here
    provider: 'sqlite',
    url: 'file:./auth.db',
  },
  socialProviders: {
    google: {
      clientId: process.env.GOOGLE_CLIENT_ID,
      clientSecret: process.env.GOOGLE_CLIENT_SECRET,
    },
    github: {
      clientId: process.env.GITHUB_CLIENT_ID,
      clientSecret: process.env.GITHUB_CLIENT_SECRET,
    },
  },
  emailAndPassword: {
    enabled: true,
    requireEmailVerification: true,
  },
  session: {
    expiresIn: 60 * 60 * 24 * 7, // 7 days
  },
};

export default authConfig;