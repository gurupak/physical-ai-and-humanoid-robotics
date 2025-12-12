import React, { useState, useEffect, useRef } from "react";
import { useAuth } from "../../contexts/AuthContext";
import TextSelectionHandler from "./TextSelectionHandler";
import styles from "./styles.module.css";

interface Message {
  role: "user" | "assistant";
  content: string;
  timestamp: string;
}

export default function FloatingChatbot(): JSX.Element | null {
  const { session, user } = useAuth();
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [input, setInput] = useState("");
  const [selectedText, setSelectedText] = useState("");
  const [isLoading, setIsLoading] = useState(false);
  const [sessionId, setSessionId] = useState<string>("");
  const messagesEndRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    if (user?.id && !sessionId) {
      createSession(user.id);
    }
  }, [user?.id]);

  useEffect(() => {
    if (sessionId) {
      loadMessages();
    }
  }, [sessionId]);

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const createSession = async (userId: string) => {
    try {
      const response = await fetch("http://localhost:8000/api/v1/sessions", {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({ user_id: userId }),
      });

      if (response.ok) {
        const data = await response.json();
        setSessionId(data.session_id);
      }
    } catch (error) {
      console.error("Failed to create session:", error);
    }
  };

  const loadMessages = async () => {
    try {
      const response = await fetch(
        `http://localhost:8000/api/v1/sessions/${sessionId}`,
      );
      if (response.ok) {
        const data = await response.json();
        setMessages(data.messages || []);
      }
    } catch (error) {
      console.error("Failed to load messages:", error);
    }
  };

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  const handleTextSelected = (text: string) => {
    setSelectedText(text);
    setIsOpen(true);
  };

  const sendMessage = async () => {
    if (!input.trim() || isLoading || !sessionId) return;

    const userMessage = input.trim();
    setInput("");
    setIsLoading(true);

    const newUserMessage: Message = {
      role: "user",
      content: userMessage,
      timestamp: new Date().toISOString(),
    };
    setMessages((prev) => [...prev, newUserMessage]);

    try {
      const response = await fetch("http://localhost:8000/api/v1/chat", {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
          "X-User-ID": user.id,
        },
        body: JSON.stringify({
          session_id: sessionId,
          message: userMessage,
          selected_text: selectedText || null,
        }),
      });

      if (response.ok) {
        const data = await response.json();
        const assistantMessage: Message = {
          role: "assistant",
          content: data.message,
          timestamp: new Date().toISOString(),
        };
        setMessages((prev) => [...prev, assistantMessage]);
        setSelectedText("");
      } else {
        const error = await response.json();
        const errorMessage: Message = {
          role: "assistant",
          content: `Error: ${error.detail || "Failed to get response"}`,
          timestamp: new Date().toISOString(),
        };
        setMessages((prev) => [...prev, errorMessage]);
      }
    } catch (error) {
      console.error("Failed to send message:", error);
      const errorMessage: Message = {
        role: "assistant",
        content: "Failed to connect to the server. Please try again.",
        timestamp: new Date().toISOString(),
      };
      setMessages((prev) => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === "Enter" && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  // Only show chatbot if user is authenticated
  if (!session || !user) {
    return null;
  }

  return (
    <>
      <TextSelectionHandler onTextSelected={handleTextSelected} />

      {/* Floating button */}
      {!isOpen && (
        <button
          className={styles.floatingButton}
          onClick={() => setIsOpen(true)}
          aria-label="Open AI Assistant"
        >
          <span className={styles.icon}>🤖</span>
          <span className={styles.badge}>AI</span>
        </button>
      )}

      {/* Chat window */}
      {isOpen && (
        <div className={styles.chatWindow}>
          <div className={styles.chatHeader}>
            <div className={styles.headerContent}>
              <span className={styles.headerIcon}>🤖</span>
              <div>
                <h3 className={styles.headerTitle}>AI Learning Assistant</h3>
                <p className={styles.headerSubtitle}>
                  Ask about the book content
                </p>
              </div>
            </div>
            <button
              className={styles.closeButton}
              onClick={() => setIsOpen(false)}
              aria-label="Close"
            >
              ✕
            </button>
          </div>

          {selectedText && (
            <div className={styles.selectedTextBanner}>
              <div className={styles.selectedTextLabel}>Selected Text:</div>
              <div className={styles.selectedTextContent}>{selectedText}</div>
              <button
                className={styles.clearSelection}
                onClick={() => setSelectedText("")}
                aria-label="Clear selection"
              >
                ✕
              </button>
            </div>
          )}

          <div className={styles.chatMessages}>
            {messages.length === 0 ? (
              <div className={styles.welcomeMessage}>
                <h4>👋 Welcome, {user.name || "there"}!</h4>
                <p>I'm your AI learning assistant. I can help you:</p>
                <ul>
                  <li>Answer questions about book concepts</li>
                  <li>Explain complex topics in detail</li>
                  <li>Clarify selected text passages</li>
                </ul>
                <p>Try asking me a question to get started!</p>
              </div>
            ) : (
              messages.map((msg, idx) => (
                <div
                  key={idx}
                  className={`${styles.message} ${styles[`message${msg.role.charAt(0).toUpperCase() + msg.role.slice(1)}`]}`}
                >
                  <div className={styles.messageAvatar}>
                    {msg.role === "user" ? "👤" : "🤖"}
                  </div>
                  <div className={styles.messageContent}>
                    <div className={styles.messageText}>{msg.content}</div>
                    <div className={styles.messageTime}>
                      {new Date(msg.timestamp).toLocaleTimeString()}
                    </div>
                  </div>
                </div>
              ))
            )}
            {isLoading && (
              <div className={`${styles.message} ${styles.messageAssistant}`}>
                <div className={styles.messageAvatar}>🤖</div>
                <div className={styles.messageContent}>
                  <div className={styles.typingIndicator}>
                    <span></span>
                    <span></span>
                    <span></span>
                  </div>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          <div className={styles.chatInputArea}>
            {selectedText === "" && (
              <div className={styles.inputControls}>
                <input
                  type="text"
                  className={styles.selectedTextInput}
                  placeholder="Paste selected text here (optional)"
                  value={selectedText}
                  onChange={(e) => setSelectedText(e.target.value)}
                />
              </div>
            )}
            <div className={styles.inputWrapper}>
              <textarea
                className={styles.chatInput}
                placeholder="Ask a question about the book..."
                value={input}
                onChange={(e) => setInput(e.target.value)}
                onKeyPress={handleKeyPress}
                rows={2}
                disabled={isLoading}
              />
              <button
                className={styles.sendButton}
                onClick={sendMessage}
                disabled={!input.trim() || isLoading}
              >
                {isLoading ? "..." : "➤"}
              </button>
            </div>
          </div>
        </div>
      )}
    </>
  );
}
