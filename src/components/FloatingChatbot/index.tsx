import React, { useState, useEffect, useRef } from "react";
import ReactMarkdown from "react-markdown";
import remarkGfm from "remark-gfm";
import { useAuth } from "../../contexts/AuthContext";
import { useRagApiUrl } from "../../lib/config";
import TextSelectionHandler from "./TextSelectionHandler";
import styles from "./styles.module.css";

interface Message {
  role: "user" | "assistant";
  content: string;
  timestamp: string;
}

export default function FloatingChatbot(): JSX.Element | null {
  const RAG_API_URL = useRagApiUrl();
  const { session, user } = useAuth();
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [input, setInput] = useState("");
  const [selectedText, setSelectedText] = useState("");
  const [isLoading, setIsLoading] = useState(false);
  const [isLoadingHistory, setIsLoadingHistory] = useState(false);
  const [sessionId, setSessionId] = useState<string>("");
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Load or create session on mount
  useEffect(() => {
    if (user?.id) {
      const storageKey = `chat_session_${user.id}`;
      const storedSessionId = localStorage.getItem(storageKey);

      if (storedSessionId) {
        // Use existing session
        setSessionId(storedSessionId);
      } else {
        // Create new session
        createSession(user.id);
      }
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
      const response = await fetch(`${RAG_API_URL}/api/v1/sessions`, {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({ user_id: userId }),
      });

      if (response.ok) {
        const data = await response.json();
        const newSessionId = data.session_id;
        setSessionId(newSessionId);

        // Store session ID in localStorage
        const storageKey = `chat_session_${userId}`;
        localStorage.setItem(storageKey, newSessionId);
      }
    } catch (error) {
      console.error("Failed to create session:", error);
    }
  };

  const loadMessages = async () => {
    setIsLoadingHistory(true);
    try {
      const response = await fetch(
        `${RAG_API_URL}/api/v1/sessions/${sessionId}`,
      );
      if (response.ok) {
        const data = await response.json();
        setMessages(data.messages || []);
      }
    } catch (error) {
      console.error("Failed to load messages:", error);
    } finally {
      setIsLoadingHistory(false);
    }
  };

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  const handleTextSelected = (text: string) => {
    // Limit selected text to 500 characters to prevent copying entire book sections
    const limitedText = text.slice(0, 500);
    setSelectedText(limitedText);
    setIsOpen(true);
  };

  const sendMessage = async () => {
    if (!input.trim() || isLoading || !sessionId) return;

    // Limit input to 1000 characters
    const userMessage = input.trim().slice(0, 1000);
    setInput("");
    setIsLoading(true);

    const newUserMessage: Message = {
      role: "user",
      content: userMessage,
      timestamp: new Date().toISOString(),
    };
    setMessages((prev) => [...prev, newUserMessage]);

    try {
      const response = await fetch(`${RAG_API_URL}/api/v1/chat`, {
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

  const startNewChat = async () => {
    if (!user?.id) return;

    // Clear current session from localStorage
    const storageKey = `chat_session_${user.id}`;
    localStorage.removeItem(storageKey);

    // Clear messages
    setMessages([]);
    setInput("");
    setSelectedText("");

    // Create new session
    await createSession(user.id);
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
            <div className={styles.headerActions}>
              <button
                className={styles.newChatButton}
                onClick={startNewChat}
                aria-label="Start new chat"
                title="Start a new conversation"
              >
                ✨
              </button>
              <button
                className={styles.closeButton}
                onClick={() => setIsOpen(false)}
                aria-label="Close"
              >
                ✕
              </button>
            </div>
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
              <>
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
                {isLoadingHistory && (
                  <div className={styles.loadingHistory}>
                    <div className={styles.loadingSpinner}></div>
                    <p>Loading previous conversation...</p>
                  </div>
                )}
              </>
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
                    <div className={styles.messageText}>
                      {msg.role === "assistant" ? (
                        <ReactMarkdown remarkPlugins={[remarkGfm]}>
                          {msg.content}
                        </ReactMarkdown>
                      ) : (
                        msg.content
                      )}
                    </div>
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
                  onChange={(e) =>
                    setSelectedText(e.target.value.slice(0, 500))
                  }
                  maxLength={500}
                  title="💡 Tip: Select text from the book and right-click to get instant help!"
                />
              </div>
            )}
            <div className={styles.inputWrapper}>
              <textarea
                className={styles.chatInput}
                placeholder="Ask a question about the book..."
                value={input}
                onChange={(e) => setInput(e.target.value.slice(0, 1000))}
                onKeyPress={handleKeyPress}
                rows={2}
                disabled={isLoading}
                maxLength={1000}
                title="Type your question here (max 1000 characters)"
              />
              <button
                className={styles.sendButton}
                onClick={sendMessage}
                disabled={!input.trim() || isLoading}
                title="Send message"
              >
                {isLoading ? "..." : "➤"}
              </button>
            </div>
            <div className={styles.tooltipHint}>
              💡 <strong>Tip:</strong> Select any text from the book and
              right-click to get instant explanations!
            </div>
          </div>
        </div>
      )}
    </>
  );
}
