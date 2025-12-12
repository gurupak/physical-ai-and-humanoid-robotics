import { useState, useEffect } from 'react'
import ChatWidget from './components/ChatWidget'
import './App.css'

function App() {
  const [userId, setUserId] = useState<string>('')
  const [sessionId, setSessionId] = useState<string>('')
  const [expertiseLevel, setExpertiseLevel] = useState<'beginner' | 'intermediate' | 'advanced'>('beginner')

  useEffect(() => {
    // In a real app, get userId from Better Auth
    // For now, use a mock user ID
    const mockUserId = 'user-' + Math.random().toString(36).substring(7)
    setUserId(mockUserId)

    // Create a session when component mounts
    createSession(mockUserId)
  }, [])

  const createSession = async (uid: string) => {
    try {
      const response = await fetch('http://localhost:8000/api/v1/sessions', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ user_id: uid }),
      })

      if (response.ok) {
        const data = await response.json()
        setSessionId(data.session_id)
        console.log('Session created:', data.session_id)
      }
    } catch (error) {
      console.error('Failed to create session:', error)
    }
  }

  const handleExpertiseLevelChange = async (level: 'beginner' | 'intermediate' | 'advanced') => {
    setExpertiseLevel(level)

    try {
      await fetch('http://localhost:8000/api/v1/users/preferences', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'X-User-ID': userId,
        },
        body: JSON.stringify({ expertise_level: level }),
      })
    } catch (error) {
      console.error('Failed to update expertise level:', error)
    }
  }

  return (
    <div className="app">
      <header className="app-header">
        <h1>Interactive Book Learning Assistant</h1>
        <div className="expertise-selector">
          <label htmlFor="expertise">Expertise Level:</label>
          <select
            id="expertise"
            value={expertiseLevel}
            onChange={(e) => handleExpertiseLevelChange(e.target.value as any)}
          >
            <option value="beginner">Beginner</option>
            <option value="intermediate">Intermediate</option>
            <option value="advanced">Advanced</option>
          </select>
        </div>
      </header>

      <main className="app-main">
        {sessionId ? (
          <ChatWidget
            sessionId={sessionId}
            userId={userId}
            expertiseLevel={expertiseLevel}
          />
        ) : (
          <div className="loading">Creating session...</div>
        )}
      </main>
    </div>
  )
}

export default App
