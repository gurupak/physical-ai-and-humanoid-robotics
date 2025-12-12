import { createAuthClient } from "better-auth/react";

export const authClient = createAuthClient({
  baseURL: "https://dynamic-courage-production.up.railway.app",
});
