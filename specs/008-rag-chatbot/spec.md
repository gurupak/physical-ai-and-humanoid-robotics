# Feature Specification: RAG Chatbot for Interactive Book Learning

**Feature Branch**: `008-rag-chatbot`  
**Created**: 2025-12-11  
**Status**: Draft  
**Input**: User description: "Integrated RAG Chatbot Development: Build and embed a Retrieval-Augmented Generation (RAG) chatbot within the published book. This chatbot, utilizing the OpenAI Agents/ChatKit SDKs, FastAPI, Neon Serverless Postgres database, and Qdrant Cloud Free Tier, must be able to answer user questions about the book's content, including answering questions based only on text selected by the user. The node server is running in api-server, the neon api key is added in .env, the qdrant cloud will be added in it, add the KEY for it. I need to tailor the responses based on the user expertise level, so they can understand it, and LLM chatbot make use of it in system prompt."

## Clarifications

### Session 2025-12-11

- Q: How should the system identify and persist reader preferences across browser sessions without requiring login? → A: Better Auth session with Neon Postgres database (existing implementation at api-server/ with expertiseLevel field already configured)
- Q: How should the book content be divided into chunks for indexing and retrieval? → A: Semantic chunking by section/subsection
- Q: What should happen when external AI or search services fail during a query? → A: Display specific error message indicating which service failed, allow user to retry manually
- Q: Where should conversation history be stored during an active session? → A: Server-side in-memory storage tied to session ID
- Q: What information should be logged for each chatbot interaction? → A: No logging at all (privacy-first approach)
- Q: When and how should the book content be indexed and loaded into Qdrant? → A: One-time build script (manual or during deployment) to index content, then server just queries Qdrant. Manual re-indexing on content updates. This approach is most practical for a book that changes infrequently, keeping content synchronized with published version.
- Q: Should authentication be required to use the chatbot, or can unauthenticated users ask questions? → A: User must authenticate (email/password, GitHub, or Google OAuth via existing Better Auth) before using the chatbot
- Q: How many relevant chunks should be retrieved from Qdrant and passed to the LLM for each question? → A: Dynamic based on query complexity (simple questions get fewer chunks, complex get more) and user expertise level (beginners may need more context, advanced users fewer). Both factors determine retrieval count.
- Q: How should the system handle rate limiting from external AI or search services? → A: Display general rate limit message, suggest waiting before retry
- Q: How long should a conversation session remain active in server memory before automatic cleanup? → A: 15 minutes after last activity
- Q: What is the maximum conversation history length (number of message turns) to retain per session? → A: Maximum 10 conversation turns

## User Scenarios & Testing *(mandatory)*

### User Story 1 - General Book Question Answering (Priority: P1)

A reader is studying the book content and has questions about concepts, examples, or explanations. They open the chatbot interface, type their question, and receive an accurate answer grounded in the book's content. The answer is tailored to their expertise level (beginner, intermediate, advanced) so they can understand it effectively.

**Why this priority**: This is the core value proposition - helping readers understand book content through conversational interaction. Without this, the feature provides no value.

**Independent Test**: Can be fully tested by asking the chatbot questions about various book chapters and verifying that responses are accurate, grounded in book content, and appropriate for the selected expertise level.

**Acceptance Scenarios**:

1. **Given** a reader has opened the book and activated the chatbot, **When** they ask "What is a Vision-Language-Action model?", **Then** the chatbot provides an answer based on book content with citations to relevant sections
2. **Given** a reader has set their expertise level to "beginner", **When** they ask a complex question, **Then** the chatbot explains the concept using simpler language and analogies
3. **Given** a reader has set their expertise level to "advanced", **When** they ask a question, **Then** the chatbot provides detailed technical explanations with mathematical notation and advanced terminology
4. **Given** a reader asks a question outside the book's scope, **When** the chatbot processes the question, **Then** it politely indicates the topic is not covered in the book
5. **Given** a reader asks a follow-up question, **When** the chatbot processes it, **Then** it maintains conversation context and references previous exchanges

---

### User Story 2 - Text Selection-Based Questions (Priority: P2)

A reader is reading a specific paragraph or section and wants clarification or deeper explanation about that exact content. They highlight the text, open the chatbot, and ask a question. The chatbot provides an answer specifically grounded in the selected text, using it as the primary context.

**Why this priority**: This enables precise, contextual learning by allowing readers to dig deeper into specific passages they're struggling with. It's the second most valuable feature after general Q&A.

**Independent Test**: Can be tested independently by selecting various text passages, asking questions about them, and verifying that responses reference and explain the selected content specifically.

**Acceptance Scenarios**:

1. **Given** a reader has selected a paragraph of text, **When** they ask "Can you explain this in simpler terms?", **Then** the chatbot provides an explanation specifically based on the selected text
2. **Given** a reader has selected a code snippet, **When** they ask "What does this code do?", **Then** the chatbot explains the selected code with step-by-step breakdown
3. **Given** a reader has selected multiple paragraphs, **When** they ask a question, **Then** the chatbot considers the entire selection as primary context
4. **Given** a reader has selected text and asks an unrelated question, **When** the chatbot processes it, **Then** it prioritizes the selected text context but also searches broader book content if needed

---

### User Story 3 - Expertise Level Customization (Priority: P1)

A reader wants to set their expertise level (beginner, intermediate, advanced) so that all chatbot responses are tailored to their understanding. They access a settings or preference menu, select their level, and all subsequent responses are adapted accordingly.

**Why this priority**: This is critical for making the chatbot useful across different reader backgrounds. Without expertise-level adaptation, responses may be too complex for beginners or too simplistic for advanced users, reducing feature value significantly.

**Independent Test**: Can be tested by setting different expertise levels and asking the same question, then verifying that response complexity, terminology, and depth vary appropriately.

**Acceptance Scenarios**:

1. **Given** a new reader opens the chatbot for the first time, **When** the interface loads, **Then** they are prompted to select their expertise level (beginner, intermediate, advanced)
2. **Given** a reader has selected "beginner" expertise, **When** they ask about machine learning concepts, **Then** responses avoid jargon and use everyday analogies
3. **Given** a reader has selected "intermediate" expertise, **When** they ask technical questions, **Then** responses balance technical accuracy with accessible explanations
4. **Given** a reader has selected "advanced" expertise, **When** they ask questions, **Then** responses include mathematical formulations, research references, and assume prerequisite knowledge
5. **Given** a reader wants to change their expertise level, **When** they access settings, **Then** they can update their level and see it reflected in subsequent responses

---

### User Story 4 - Chatbot UI Integration in Book (Priority: P2)

An authenticated reader is navigating the published Docusaurus book and wants quick access to the chatbot. They see a visible, accessible chatbot icon or button on every book page. When clicked, it opens a chat interface that doesn't disrupt their reading experience. Unauthenticated visitors see the icon but are prompted to log in when they attempt to use it.

**Why this priority**: The chatbot must be easily discoverable and accessible, but the core functionality (Q&A) is more important than the UI polish. This can be implemented with a basic interface initially.

**Independent Test**: Can be tested by navigating different book pages and verifying the chatbot is consistently accessible and functional without requiring backend changes.

**Acceptance Scenarios**:

1. **Given** a reader is viewing any book page, **When** they look for the chatbot, **Then** they see a clearly visible chat icon/button in a consistent location
2. **Given** a reader clicks the chatbot icon, **When** the interface opens, **Then** it appears as an overlay or sidebar without navigating away from the current page
3. **Given** the chatbot interface is open, **When** the reader wants to close it, **Then** they can dismiss it and return to normal reading
4. **Given** a reader is on a mobile device, **When** they access the chatbot, **Then** the interface is responsive and usable on smaller screens
5. **Given** a reader has the chatbot open and scrolls the page, **When** they interact with either, **Then** both the book content and chat remain accessible
6. **Given** an unauthenticated visitor clicks the chatbot icon, **When** the system checks authentication status, **Then** they are prompted to log in via Better Auth (email/password, GitHub, or Google OAuth)

---

### User Story 5 - Persistent Conversation History (Priority: P3)

A reader wants to maintain conversation context across their learning session. They ask multiple related questions over time, and the chatbot remembers previous exchanges to provide more coherent, contextual responses.

**Why this priority**: This enhances the learning experience by enabling multi-turn conversations, but the chatbot can still be valuable without it (each question answered independently).

**Independent Test**: Can be tested by having a multi-turn conversation, verifying that later responses reference earlier questions and answers appropriately.

**Acceptance Scenarios**:

1. **Given** a reader has asked a question about neural networks, **When** they follow up with "Can you give an example?", **Then** the chatbot provides an example related to neural networks from the conversation context
2. **Given** a reader has had a 5-message conversation, **When** they refer to "the previous example", **Then** the chatbot correctly identifies and references it
3. **Given** a reader closes and reopens the chatbot, **When** they view the interface, **Then** their conversation history is preserved (at least for the current session)
4. **Given** a reader wants to start a new topic, **When** they clear the conversation or start fresh, **Then** the chatbot resets context appropriately

---

### Edge Cases

- What happens when a reader asks a question but no relevant content exists in the book?
- How does the system handle extremely long text selections (entire chapters)?
- What happens when the reader's question is ambiguous or poorly formed?
- How does the chatbot handle questions in languages other than English?
- What happens when external services (AI or search) are temporarily unavailable? → Display specific error message indicating which service failed and provide retry option
- How does the system handle concurrent requests from multiple users?
- What happens when a reader asks personal or inappropriate questions?
- How does the chatbot handle questions that require information from multiple disconnected book sections?
- What happens when the selected text is very short (single word or phrase)?
- How does the system handle rate limiting from external services? → Display general rate limit message and suggest waiting before retry

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST allow readers to ask natural language questions about book content through a chat interface
- **FR-002**: System MUST retrieve relevant book content using semantic search to ground chatbot responses, with retrieval count dynamically adjusted based on query complexity and user expertise level
- **FR-003**: System MUST generate responses that are factually accurate based on retrieved book content
- **FR-004**: System MUST support three expertise levels (beginner, intermediate, advanced) that adjust response complexity, terminology, and depth
- **FR-005**: System MUST allow readers to select text passages and ask questions specifically about that selection
- **FR-006**: System MUST prioritize selected text as primary context when answering text-selection-based questions
- **FR-007**: System MUST provide citations or references to specific book sections/subsections when answering questions
- **FR-008**: System MUST handle questions outside the book's scope by politely declining rather than hallucinating answers
- **FR-009**: System MUST maintain conversation context for multi-turn dialogues within a session using server-side in-memory storage tied to the Better Auth session ID, with automatic cleanup after 15 minutes of inactivity and a maximum of 10 conversation turns retained per session
- **FR-010**: System MUST display the chatbot interface as an accessible component on all book pages without disrupting reading experience
- **FR-011**: System MUST index book content using semantic chunking by section/subsection to enable efficient semantic search while preserving content structure and context
- **FR-012**: System MUST allow readers to set and change their expertise level through a settings interface
- **FR-013**: System MUST adapt response generation behavior based on selected expertise level
- **FR-014**: System MUST handle API failures gracefully by displaying specific error messages indicating which service failed (AI service or search service) and providing users with a retry option
- **FR-015**: System MUST support mobile and desktop viewport sizes for the chat interface
- **FR-016**: System MUST prevent malicious queries and protect against security exploits
- **FR-017**: System MUST NOT log chatbot interactions or conversation content to ensure reader privacy (privacy-first approach)
- **FR-018**: System MUST return responses within a reasonable time frame (target: under 5 seconds for typical queries)
- **FR-019**: System MUST persist reader expertise level preference across sessions using the existing Better Auth session stored in Neon Postgres database
- **FR-020**: System MUST indicate when the chatbot is processing a query (loading state)
- **FR-021**: System MUST require reader authentication via Better Auth (email/password, GitHub OAuth, or Google OAuth) before allowing access to the chatbot interface

### Key Entities

- **Reader**: A person using the book who interacts with the chatbot; has an expertise level preference; can select text and ask questions
- **Question**: A natural language query submitted by a reader; may reference selected text; has associated conversation context
- **Book Content**: Text, code, and explanations from the published book; organized into searchable chunks based on section/subsection structure for efficient retrieval while preserving context
- **Conversation**: A sequence of question-answer exchanges between a reader and the chatbot; maintains context; belongs to a session; stored in server-side memory and automatically cleaned up when session expires
- **Expertise Level**: An enumeration (beginner, intermediate, advanced) that determines response style; set by reader; persists across sessions
- **Text Selection**: A highlighted portion of book content; serves as primary context for questions; passed to the chatbot for grounding
- **Response**: An answer generated by the chatbot; grounded in retrieved book content; tailored to expertise level; may include citations
- **Session**: A time-bounded interaction period managed by Better Auth; maintains conversation history; tied to an authenticated reader; stores expertise level preference in Neon Postgres

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Readers can receive accurate answers to at least 90% of in-scope questions about book content
- **SC-002**: Chatbot responses are returned within 5 seconds for 95% of queries under normal load
- **SC-003**: 85% of readers successfully set their expertise level on first use
- **SC-004**: Text selection-based questions receive responses that explicitly reference the selected content in 100% of cases
- **SC-005**: Beginner-level responses contain no unexplained technical jargon in 95% of cases
- **SC-006**: Advanced-level responses include appropriate technical depth and terminology in 90% of cases
- **SC-007**: The chatbot correctly declines to answer out-of-scope questions in at least 80% of such cases
- **SC-008**: The chat interface is accessible and functional on both mobile and desktop devices with no critical usability issues
- **SC-009**: System maintains conversation context for at least 10 consecutive turns without losing coherence
- **SC-010**: Reader satisfaction with chatbot answers is rated 4/5 or higher in 80% of feedback responses
- **SC-011**: The chatbot handles at least 100 concurrent reader sessions without performance degradation
- **SC-012**: Zero security incidents related to malicious queries or chatbot exploits in production

## Assumptions

- The book content is accessible in a structured format that can be parsed and chunked for searching
- The existing backend server infrastructure (api-server/) with Better Auth and Neon Postgres can be extended with new chatbot endpoints
- API credentials for external services can be securely managed using environment configuration (.env file)
- The book content is primarily in English
- Readers access the book through modern web browsers that support interactive features
- External service free tier limits are sufficient for initial deployment and testing
- Third-party service rate limits will not be exceeded under expected usage patterns (or graceful degradation is acceptable)
- Readers authenticate using the existing Better Auth system (email/password, GitHub, or Google OAuth) to access the chatbot
- The Better Auth user table's expertiseLevel field will store and persist reader expertise preferences
- Conversation history is stored in server-side memory tied to session ID and is automatically cleaned up when sessions expire (no long-term persistent storage)
- The book content does not contain sensitive or private information that requires special access controls
- Reader privacy is prioritized - no chatbot interaction logging or conversation content storage beyond the in-memory session data
- Citations to book sections can be provided as chapter/section references without requiring exact page numbers
- The book publishing platform can be extended with custom interactive components for the chat interface

## Dependencies

- Third-party AI service availability and service stability for natural language processing
- Cloud-based semantic search service availability
- Cloud-based data storage service availability
- Existing backend server infrastructure
- Book publishing platform build system and extensibility capabilities
- Browser support for modern web features and real-time communication protocols

## Out of Scope

- Multi-language support for non-English questions and responses
- Voice-based interaction with the chatbot
- Image or diagram analysis from book content
- New user authentication system (existing Better Auth system will be reused)
- Long-term conversation history beyond current session
- Integration with external knowledge sources beyond the book
- Automatic expertise level detection based on reader behavior
- Real-time collaborative learning features (multiple readers in same chat)
- Administrative dashboard for chatbot analytics
- A/B testing different response strategies
- Custom training of AI models on book content
- Offline mode or progressive web app capabilities
- Export of conversation transcripts
- Gamification or learning progress tracking
