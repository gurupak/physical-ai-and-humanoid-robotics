# PHR (Prompt History Record) - Detailed Process

This document contains detailed instructions for creating Prompt History Records. Referenced by CLAUDE.md but not loaded into context unless needed.

## When to Create PHRs

- Implementation work (code changes, new features)
- Planning/architecture discussions
- Debugging sessions
- Spec/task/plan creation
- Multi-step workflows

## PHR Creation Process

### 1. Detect Stage

Determine one of:
- `constitution` - Constitution changes
- `spec` - Specification work
- `plan` - Planning/architecture
- `tasks` - Task creation
- `red` - Red phase (failing tests)
- `green` - Green phase (passing tests)
- `refactor` - Refactoring
- `explainer` - Explanations
- `misc` - Miscellaneous feature work
- `general` - General tasks

### 2. Generate Title

- 3-7 words
- Create slug for filename (lowercase, hyphens)

### 3. Resolve Route

All PHRs go under `history/prompts/`:
- Constitution → `history/prompts/constitution/`
- Feature stages → `history/prompts/<feature-name>/` (requires feature context)
- General → `history/prompts/general/`

### 4. Agent-Native Flow (Preferred)

1. Read PHR template:
   - `.specify/templates/phr-template.prompt.md`
   - Or `templates/phr-template.prompt.md`

2. Allocate ID (increment; on collision, increment again)

3. Compute output path:
   - Constitution: `history/prompts/constitution/<ID>-<slug>.constitution.prompt.md`
   - Feature: `history/prompts/<feature-name>/<ID>-<slug>.<stage>.prompt.md`
   - General: `history/prompts/general/<ID>-<slug>.general.prompt.md`

4. Fill ALL placeholders:
   - **ID**: Allocated number
   - **TITLE**: Generated title
   - **STAGE**: Detected stage
   - **DATE_ISO**: YYYY-MM-DD format
   - **SURFACE**: "agent"
   - **MODEL**: Best known model name
   - **FEATURE**: Feature name or "none"
   - **BRANCH**: Current git branch
   - **USER**: Current user
   - **COMMAND**: Current slash command
   - **LABELS**: ["topic1", "topic2", ...]
   - **LINKS**:
     - SPEC: URL or "null"
     - TICKET: URL or "null"
     - ADR: URL or "null"
     - PR: URL or "null"
   - **FILES_YAML**: List created/modified files (one per line with " - ")
   - **TESTS_YAML**: List tests run/added (one per line with " - ")
   - **PROMPT_TEXT**: Full user input (verbatim, NOT truncated)
   - **RESPONSE_TEXT**: Key assistant output (concise but representative)
   - **OUTCOME/EVALUATION**: Any fields required by template

5. Write file using agent tools (Write/Edit)

6. Confirm absolute path in output

### 5. Shell Fallback (Only if agent-native unavailable)

```bash
.specify/scripts/bash/create-phr.sh \
  --title "<title>" \
  --stage <stage> \
  [--feature <name>] \
  --json
```

Then open/patch created file to fill all placeholders.

### 6. Post-Creation Validations

Must pass:
- ✅ No unresolved placeholders (e.g., `{{THIS}}`, `[THAT]`)
- ✅ Title, stage, dates match front-matter
- ✅ PROMPT_TEXT is complete (not truncated)
- ✅ File exists at expected path and is readable
- ✅ Path matches routing rules

### 7. Reporting

Print:
- PHR ID
- File path
- Stage
- Title

On failure: Warn but don't block main command.

**Exception**: Skip PHR for `/sp.phr` itself.

## Example PHR Front Matter

```yaml
---
id: "0042"
title: "Implement User Authentication"
stage: "green"
date: "2025-12-06"
surface: "agent"
model: "claude-sonnet-4.5"
feature: "user-auth"
branch: "feature/user-auth"
user: "developer"
command: "/sp.implement"
labels: ["authentication", "security", "backend"]
links:
  spec: "specs/user-auth/spec.md"
  ticket: "null"
  adr: "history/adr/0005-oauth2-choice.md"
  pr: "null"
files:
  - src/auth/oauth.ts
  - src/middleware/auth-check.ts
tests:
  - tests/auth/oauth.test.ts
---
```

## Tips

- Always preserve full user input (no truncation)
- Link related artifacts (specs, ADRs, PRs)
- Use descriptive labels for searchability
- Keep RESPONSE_TEXT concise but meaningful
- Validate before considering PHR complete
