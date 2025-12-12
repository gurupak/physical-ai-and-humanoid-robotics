import React, { createContext, useContext, useEffect, useState } from "react";
import { authClient } from "../lib/auth";

interface ProfileData {
  firstName: string;
  lastName: string;
  email: string;
  phone: string;
  expertiseLevel: string;
}

interface AuthContextType {
  user: any;
  session: any;
  loading: boolean;
  isAuthenticated: boolean;
  signIn: (email: string, password: string) => Promise<void>;
  signInSocial: (provider: "github" | "google") => Promise<void>;
  signUp: (data: {
    email: string;
    password: string;
    name?: string;
  }) => Promise<void>;
  signOut: () => Promise<void>;
  checkSession: () => Promise<void>;
  updateProfile: (data: ProfileData) => Promise<void>;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

export const useAuth = () => {
  const context = useContext(AuthContext);
  if (!context) {
    throw new Error("useAuth must be used within an AuthProvider");
  }
  return context;
};

export const AuthProvider: React.FC<{ children: React.ReactNode }> = ({
  children,
}) => {
  const [user, setUser] = useState<any>(null);
  const [session, setSession] = useState<any>(null);
  const [loading, setLoading] = useState(true);
  const isAuthenticated = !!session && !!user;

  const checkSession = async () => {
    try {
      const { data } = await authClient.getSession();
      console.log("Session check result:", data); // Debug log
      setSession(data?.session);
      setUser(data?.user);
    } catch (error) {
      console.error("Error checking session:", error);
      setUser(null);
      setSession(null);
    } finally {
      setLoading(false);
    }
  };

  const signIn = async (email: string, password: string) => {
    try {
      const result = await authClient.signIn.email({ email, password });
      if (result.data) {
        await checkSession();
      }
    } catch (error) {
      console.error("Error signing in:", error);
      throw error;
    }
  };

  const signInSocial = async (provider: "github" | "google") => {
    try {
      await authClient.signIn.social({
        provider,
        callbackURL: window.location.origin + window.location.pathname,
      });
    } catch (error) {
      console.error("Error signing in with social:", error);
      throw error;
    }
  };

  const signUp = async (data: {
    email: string;
    password: string;
    name?: string;
  }) => {
    try {
      await authClient.signUp.email(data);
      // After signup, you might want to sign them in
      await signIn(data.email, data.password);
    } catch (error) {
      console.error("Error signing up:", error);
      throw error;
    }
  };

  const signOut = async () => {
    try {
      await authClient.signOut();
      setUser(null);
      setSession(null);
    } catch (error) {
      console.error("Error signing out:", error);
      throw error;
    }
  };

  const updateProfile = async (data: ProfileData) => {
    try {
      const response = await fetch("http://localhost:3008/api/user/profile", {
        method: "PUT",
        headers: {
          "Content-Type": "application/json",
        },
        credentials: "include",
        body: JSON.stringify(data),
      });

      if (!response.ok) {
        const error = await response.json();
        throw new Error(error.error || "Failed to update profile");
      }

      const result = await response.json();

      console.log("Profile update response:", result.user);

      // Update local user state with the updated data
      setUser(result.user);

      // Note: We don't refresh session here because the API already returned the latest data
      // and checkSession might overwrite with cached data
    } catch (error) {
      console.error("Error updating profile:", error);
      throw error;
    }
  };

  useEffect(() => {
    checkSession();

    // Check session after OAuth redirect
    const handleOAuthCallback = async () => {
      // If URL has error or session params, check session again
      if (
        window.location.search.includes("error") ||
        window.location.search.includes("session")
      ) {
        await checkSession();
        // Clean up URL
        window.history.replaceState(
          {},
          document.title,
          window.location.pathname,
        );
      }
    };

    handleOAuthCallback();
  }, []);

  const value = {
    user,
    session,
    loading,
    isAuthenticated,
    signIn,
    signInSocial,
    signUp,
    signOut,
    checkSession,
    updateProfile,
  };

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
};
