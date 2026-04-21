# Controlling SPEAR Applications with Natural Language

We provide a [MCP](https://modelcontextprotocol.io) server that allows [Claude Code](https://claude.com/claude-code) to qury and manipulate any SPEAR application.

Our first step is to launch a SPEAR application for our MCP server to connect to. This can be any Unreal application that has been built with the SPEAR plugins (e.g., the `SpearSim` application that we built in the [Getting Started](getting_started.md) tutorial), or it can be the Unreal Editor that is running a project that includes the SPEAR plugins.

Our next step is to launch Claude Code from the project root directory. At this point, we can control the application using natural language. For example, we can ask Claude Code,

- _"I have a UE instance running, what am I looking at?"_
- _"Spawn a spotlight above the vase I'm looking at."_
- _"Move the two chairs I'm looking at so they're almost but not quite touching."_
- _"Swap the materials of the coffee table and the floor."_

In contrast to all existing MCP servers for the Unreal Engine, the SPEAR MCP server can control standalone applications, and can ground the user's text prompt (e.g., "the two chairs I'm looking at") with specific Unreal objects using an ID rendering approach. Internally, our MCP server acts on the user's behalf by writing snippets of SPEAR code, and therefore it not limited to a fixed vocabulary of actions.
