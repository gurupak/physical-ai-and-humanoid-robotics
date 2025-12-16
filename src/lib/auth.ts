import { createAuthClient } from "better-auth/react";
import { useMemo } from "react";
import { useAuthApiUrl } from "./config";

// Hook to create auth client with correct baseURL from Docusaurus context
export const useAuthClient = () => {
  const authApiUrl = useAuthApiUrl();

  return useMemo(() => {
    console.log("Creating auth client with baseURL:", authApiUrl);
    return createAuthClient({
      baseURL: authApiUrl,
      fetch: {
        credentials: "include",
      },
    });
  }, [authApiUrl]);
};

// For backward compatibility, export a singleton that will be created on first use
// This won't have access to Docusaurus context, so it will use production URL
let defaultClient: ReturnType<typeof createAuthClient> | null = null;

export const authClient = new Proxy({} as ReturnType<typeof createAuthClient>, {
  get(target, prop) {
    if (!defaultClient) {
      console.warn(
        "Using default auth client without Docusaurus context - this will use production URL",
      );
      defaultClient = createAuthClient({
        baseURL: "https://dynamic-courage-production.up.railway.app",
        fetch: {
          credentials: "include",
        },
      });
    }
    return (defaultClient as any)[prop];
  },
});
