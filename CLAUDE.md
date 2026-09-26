# Agent Documentation

`CLAUDE.md` is intentionally lightweight. More detailed documentation lives in `docs/` so `CLAUDE.md` and any other agent-specific config files can point to the same source of truth:

- `docs/agents.md` — SPEAR programming model (read always).
- `docs/agents.mcp_usage.md` — MCP tool usage, API patterns, and rendering. Read once at the start of a session that uses SPEAR MCP tools.
- `docs/agents.style_guide.md` — code style conventions. Read when writing or reviewing code that will be checked in.

`.claudeignore` is separate from `.gitignore` and should not be synchronized with it.

Knowledge that would be useful to other developers or MCP users should go in the checked-in `docs/agents*.md` files, not in personal agent memories. Memories are appropriate only for per-user preferences (e.g., communication style, role context).
