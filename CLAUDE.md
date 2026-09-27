# Agent Documentation

`CLAUDE.md` is intentionally lightweight. More detailed documentation lives in `docs/` so `CLAUDE.md` and any other agent-specific config files can point to the same source of truth:

- `docs/agents.md` — SPEAR programming model (read always).
- `docs/agents.mcp_usage.md` — MCP tool usage, API patterns, and rendering. Read once at the start of a session that uses SPEAR MCP tools.
- `docs/agents.style_guide.md` — code style conventions. Read when writing or reviewing code that will be checked in.

`.claudeignore` is separate from `.gitignore` and should not be synchronized with it. The intent of `.claudeignore` is to keep agents out of large binary/vendored/generated trees (e.g. `.uasset`/`.umap` files, `cpp/unreal_projects/SpearSim/Content`, `third_party`) that aren't meant for inspection or editing, so don't read, open, or modify files under those paths even if your tooling doesn't enforce `.claudeignore` directly.

Knowledge that would be useful to other developers or MCP users should go in the checked-in `docs/agents*.md` files, not in personal agent memories. Memories are appropriate only for per-user preferences (e.g., communication style, role context).
