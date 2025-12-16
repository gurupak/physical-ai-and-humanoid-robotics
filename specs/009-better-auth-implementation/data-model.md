# Data Model: Better Auth Implementation

## Entities

### User Table

**Purpose**: Core user account information

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `id` | uuid | PRIMARY KEY | Unique user identifier |
| `email` | varchar(255) | UNIQUE NOT NULL | User's email from OAuth provider |
| `name` | varchar(255) | | Display name from OAuth |
| `profile_image` | text | | Avatar URL from OAuth provider |
| `python_level` | enum('beginner','intermediate','advanced') | DEFAULT 'intermediate' | Python programming expertise |
| `ros_experience` | enum('none','ros1','ros2') | DEFAULT 'none' | ROS framework experience |
| `hardware_access` | enum('none','simulation','physical') | DEFAULT 'simulation' | Available hardware environment |
| `learning_goals` | enum('career','student','hobbyist','research') | DEFAULT 'hobbyist' | Primary learning objective |
| `created_at` | timestamptz | DEFAULT NOW() | Account creation timestamp |
| `updated_at` | timestamptz | DEFAULT NOW() | Last modification timestamp |

**Relationships**:
- One-to-many with Session table (user has many sessions)
- One-to-many with Account table (user can have OAuth accounts)
- One-to-many with UserProgress table (track chapter progress)

### Account Table (OAuth Provider Links)

**Purpose**: Store OAuth provider account connections

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `id` | uuid | PRIMARY KEY | Unique account identifier |
| `user_id` | uuid | FOREIGN KEY → User(id) | Link to user |
| `provider` | varchar(50) | NOT NULL | 'google' or 'github' |
| `provider_account_id` | varchar(255) | NOT NULL | ID from OAuth provider |
| `access_token` | text | | OAuth access token (encrypted) |
| `refresh_token` | text | | OAuth refresh token (encrypted) |
| `expires_at` | timestamptz | | Token expiration|
| `created_at` | timestamptz | DEFAULT NOW() | Connection timestamp |

**Constraints**:
- Unique constraint on (provider, provider_account_id)
- User can only have one account per provider (strict linking per clarification)

### Session Table

**Purpose**: Maintain authenticated browser sessions with security tracking

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `id` | uuid | PRIMARY KEY | Unique session identifier |
| `user_id` | uuid | FOREIGN KEY → User(id) | Associated user |
| `token` | varchar(255) | UNIQUE NOT NULL | Secure session token |
| `expires_at` | timestamptz | NOT NULL | Session expiration (30 days) |
| `created_at` | timestamptz | DEFAULT NOW() | Session creation |
| `ip_address` | inet | | Client IP for security |
| `user_agent` | text | | Browser identifier |
| `last_active` | timestamptz | DEFAULT NOW() | Last activity timestamp |

**Constraints**:
- Token must be securely random
- Sessions expire after 30 days of inactivity
- Max 90 days absolute lifetime

### UserProgress Table

**Purpose**: Track user's reading progress through textbook chapters

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `id` | uuid | PRIMARY KEY | Unique progress identifier |
| `user_id` | uuid | FOREIGN KEY → User(id) | User tracking progress |
| `chapter_id` | varchar(100) | NOT NULL | Chapter identifier from Docusaurus |
| `completion_percentage` | integer | CHECK (0-100) | Reading completion (0-100%) |
| `last_read_position` | text | | Last visited section URL |
| `updated_at` | timestamptz | DEFAULT NOW() | Last progress update |

**Constraints**:
- Unique constraint on (user_id, chapter_id)
- Multiple chapters per user allowed

### Validation Rules

**Email Validation**:
- Must match OAuth provider's verified email
- Standard RFC 5322 format validation

**Session Token Security**:
- 32+ character secured random strings
- Rotated on suspicious activity

**Expertise Defaults** (per clarification):
- python_level: 'intermediate' if not specified
- ros_experience: 'none' if not specified
- hardware_access: 'simulation' if not specified
- learning_goals: 'hobbyist' if not specified

### State Transitions

**User Account Lifecycle**:
1. `Created` - Account created via OAuth
2. `Active` - Onboarding completed
3. `Inactive` - No activity for 30+ days
4. `Deleted` - On user request or retention policy

**Session Lifecycle**:
1. `Created` - Successful authentication
2. `Active` - Regular activity within 30 days
3. `Expired` - Inactive > 30 days or max age 90 days
4. `Invalidated` - Logout or security event

### Performance Considerations

- Indexes on: `user_id`, `(provider, provider_account_id)`, `token`, `email`
- UUIDv4 for distributed system compatibility
- Timestamptz for timezone-aware operations
- Connection pooling with prepared statements

### Security Considerations

- OAuth tokens encrypted at rest
- IP addresses logged for security events only
- No passwords stored (OAuth only)
- Session tokens hashed in memory
- Access tokens refreshed automatically (8 hours before expiry)

### Data Retention

- Active sessions: Until expiration
- Security logs: 30 days (basic logging per clarification)
- User data: Retained until user requests deletion or account soft delete
- Progress data: Retained with user account lifecycle

This data model supports the authentication requirements while maintaining security, performance, and extensibility for future features like AI-powered personalization and learning analytics.,