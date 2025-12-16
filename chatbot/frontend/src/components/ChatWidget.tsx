import { useState, useEffect, useRef } from "react";
import ReactMarkdown from "react-markdown";
import remarkGfm from "remark-gfm";
import "./ChatWidget.css";

interface Message {
  role: "user" | "assistant";
  content: string;
  timestamp: string;
}

interface ChatWidgetProps {
  sessionId: string;
  userId: string;
  expertiseLevel: "beginner" | "intermediate" | "advanced";
}

export default function ChatWidget({ sessionId, userId }: ChatWidgetProps) {
  const API_URL =
    import.meta.env.VITE_API_URL ||
    "https://gregarious-tenderness-production-79e3.up.railway.app";
  const [messages, setMessages] = useState<Message[]>([]);
  const [input, setInput] = useState("");
  const [selectedText, setSelectedText] = useState("");
  const [isLoading, setIsLoading] = useState(false);
  const [isLoadingHistory, setIsLoadingHistory] = useState(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    // Load existing messages for the session
    loadMessages();
  }, [sessionId]);

  useEffect(() => {
    // Scroll to bottom when messages change
    scrollToBottom();
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  const loadMessages = async () => {
    setIsLoadingHistory(true);
    try {
      const response = await fetch(`${API_URL}/api/v1/sessions/${sessionId}`);
      if (response.ok) {
        const data = await response.json();
        setMessages(data.messages);
      }
    } catch (error) {
      console.error("Failed to load messages:", error);
    } finally {
      setIsLoadingHistory(false);
    }
  };

  const sendMessage = async () => {
    if (!input.trim() || isLoading) return;

    // Limit input to 1000 characters
    const userMessage = input.trim().slice(0, 1000);
    setInput("");
    setIsLoading(true);

    // Add user message to UI immediately
    const newUserMessage: Message = {
      role: "user",
      content: userMessage,
      timestamp: new Date().toISOString(),
    };
    setMessages((prev) => [...prev, newUserMessage]);

    try {
      const response = await fetch(`${API_URL}/api/v1/chat`, {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
          "X-User-ID": userId,
        },
        body: JSON.stringify({
          session_id: sessionId,
          message: userMessage,
          selected_text: selectedText || null,
        }),
      });

      if (response.ok) {
        const data = await response.json();

        // Add assistant response
        const assistantMessage: Message = {
          role: "assistant",
          content: data.message,
          timestamp: new Date().toISOString(),
        };
        setMessages((prev) => [...prev, assistantMessage]);

        // Clear selected text after use
        setSelectedText("");
      } else {
        const error = await response.json();
        console.error("Error:", error);
        // Show error message
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

  return (
    <div className="chat-widget">
      <div className="chat-header">
        <h2>AI Learning Assistant</h2>
        <p>Ask me anything about the book content!</p>
      </div>

      {selectedText && (
        <div className="selected-text-banner">
          <div className="selected-text-label">Selected Text:</div>
          <div className="selected-text-content">{selectedText}</div>
          <button
            className="clear-selection"
            onClick={() => setSelectedText("")}
            aria-label="Clear selection"
          >
            ✕
          </button>
        </div>
      )}

      <div className="chat-messages">
        {messages.length === 0 ? (
          <>
            <div className="welcome-message">
              <h3>👋 Welcome!</h3>
              <p>I'm your AI learning assistant. I can help you:</p>
              <ul>
                <li>Answer questions about book concepts</li>
                <li>Explain complex topics in detail</li>
                <li>Clarify selected text passages</li>
              </ul>
              <p>Try asking me a question to get started!</p>
            </div>
            {isLoadingHistory && (
              <div className="loading-history">
                <div className="loading-spinner"></div>
                <p>Loading previous conversation...</p>
              </div>
            )}
          </>
        ) : (
          messages.map((msg, idx) => (
            <div key={idx} className={`message message-${msg.role}`}>
              <div className="message-avatar">
                {msg.role === "user" ? "👤" : "🤖"}
              </div>
              <div className="message-content">
                <div className="message-text">
                  {msg.role === "assistant" ? (
                    <ReactMarkdown remarkPlugins={[remarkGfm]}>
                      {msg.content}
                    </ReactMarkdown>
                  ) : (
                    msg.content
                  )}
                </div>
                <div className="message-time">
                  {new Date(msg.timestamp).toLocaleTimeString()}
                </div>
              </div>
            </div>
          ))
        )}
        {isLoading && (
          <div className="message message-assistant">
            <div className="message-avatar">🤖</div>
            <div className="message-content">
              <div className="typing-indicator">
                <span></span>
                <span></span>
                <span></span>
              </div>
            </div>
          </div>
        )}
        <div ref={messagesEndRef} />
      </div>

      <div className="chat-input-area">
        <div className="input-controls">
          <input
            type="text"
            className="selected-text-input"
            placeholder="Paste selected text here (optional)"
            value={selectedText}
            onChange={(e) => setSelectedText(e.target.value.slice(0, 500))}
            maxLength={500}
            title="💡 Tip: Select text from the book and right-click to get instant help!"
          />
        </div>
        <div className="input-wrapper">
          <textarea
            className="chat-input"
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
            className="send-button"
            onClick={sendMessage}
            disabled={!input.trim() || isLoading}
            title="Send message"
          >
            {isLoading ? "..." : "➤"}
          </button>
        </div>
        <div className="tooltip-hint">
          💡 <strong>Tip:</strong> Select any text from the book and right-click
          to get instant explanations!
        </div>
      </div>
    </div>
  );
}
