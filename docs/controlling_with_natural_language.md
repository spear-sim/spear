# Controlling SPEAR Applications with Natural Language

We provide a [MCP](https://modelcontextprotocol.io) server that allows Claude Code (e.g., [Claude Code](https://claude.com/claude-code)) to inspect and manipulate any SPEAR application.

## Assumptions

We will assume that you have completed all the steps in our [Getting Started](getting_started.md) tutorial, and that you have Claude Code installed.

## Launch a SPEAR application

Our first step is to launch a SPEAR application for our MCP server to connect to. This can be any Unreal application that has been built with the SPEAR plugins (e.g., the `SpearSim` application that we built in the [Getting Started](getting_started.md) tutorial), or it can be the Unreal Editor that is running a project that includes the SPEAR plugins.

## Launch and interact with Claude Code

Our next step is to launch Claude Code from the project root directory. At this point, we can control the application using natural language. For example, we can ask Claude Code:

- _"I have a UE instance running, what am I looking at?"_
- _"Spawn a spotlight above the vase I'm looking at."_
- _"Move the two chairs I'm looking at so they're almost but not quite touching."_
- _"Swap the materials of the coffee table and the floor."_
