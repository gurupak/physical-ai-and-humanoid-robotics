import { useState, useEffect } from "react";
import ChatWidget from "./components/ChatWidget";
import "./App.css";

function App() {
  const API_URL =
    import.meta.env.VITE_API_URL ||
    "https://gregarious-tenderness-production-79e3.up.railway.app";
  const [userId, setUserId] = useState<string>("");
  const [sessionId, setSessionId] = useState<string>("");
  const [expertiseLevel, setExpertiseLevel] = useState<
    "beginner" | "intermediate" | "advanced"
  >("beginner");

  useEffect(() => {
    // In a real app, get userId from Better Auth
    // For now, use a mock user ID or get from localStorage
    let mockUserId = localStorage.getItem("standalone_user_id");
    if (!mockUserId) {
      mockUserId = "user-" + Math.random().toString(36).substring(7);
      localStorage.setItem("standalone_user_id", mockUserId);
    }
    setUserId(mockUserId);

    // Load or create session
    const storageKey = `chat_session_${mockUserId}`;
    const storedSessionId = localStorage.getItem(storageKey);

    if (storedSessionId) {
      setSessionId(storedSessionId);
    } else {
      createSession(mockUserId);
    }
  }, []);

  const createSession = async (uid: string) => {
    try {
      const response = await fetch(`${API_URL}/api/v1/sessions`, {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({ user_id: uid }),
      });

      if (response.ok) {
        const data = await response.json();
        const newSessionId = data.session_id;
        setSessionId(newSessionId);

        // Store session ID in localStorage
        const storageKey = `chat_session_${uid}`;
        localStorage.setItem(storageKey, newSessionId);

        console.log("Session created:", newSessionId);
      }
    } catch (error) {
      console.error("Failed to create session:", error);
    }
  };

  const startNewChat = () => {
    if (!userId) return;

    // Clear current session from localStorage
    const storageKey = `chat_session_${userId}`;
    localStorage.removeItem(storageKey);

    // Create new session
    createSession(userId);
  };

  const handleExpertiseLevelChange = async (
    level: "beginner" | "intermediate" | "advanced",
  ) => {
    setExpertiseLevel(level);

    try {
      await fetch(`${API_URL}/api/v1/users/preferences`, {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
          "X-User-ID": userId,
        },
        body: JSON.stringify({ expertise_level: level }),
      });
    } catch (error) {
      console.error("Failed to update expertise level:", error);
    }
  };

  return (
    <div className="app">
      <header className="app-header">
        <h1>Interactive Book Learning Assistant</h1>
        <div className="header-controls">
          <div className="expertise-selector">
            <label htmlFor="expertise">Expertise Level:</label>
            <select
              id="expertise"
              value={expertiseLevel}
              onChange={(e) =>
                handleExpertiseLevelChange(e.target.value as any)
              }
            >
              <option value="beginner">Beginner</option>
              <option value="intermediate">Intermediate</option>
              <option value="advanced">Advanced</option>
            </select>
          </div>
          <button
            className="new-chat-btn"
            onClick={startNewChat}
            disabled={!sessionId}
            title="Start a new conversation"
          >
            ✨ New Chat
          </button>
        </div>
      </header>

      <main className="app-main">
        {sessionId ? (
          <ChatWidget
            sessionId={sessionId}
            userId={userId}
            expertiseLevel={expertiseLevel}
            key={sessionId}
          />
        ) : (
          <div className="loading">Creating session...</div>
        )}
      </main>
    </div>
  );
}

export default App;
