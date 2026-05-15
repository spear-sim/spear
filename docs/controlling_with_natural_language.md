# Controlling Unreal Applications with Natural Language

We provide an [MCP](https://modelcontextprotocol.io) server that interoperates with [Claude Code](https://claude.com/claude-code) and enables a user to query and manipulate any Unreal application using natural language.

To use our MCP server, the first step is to launch an Unreal application for our server to connect to. This can be any Unreal application that has been built with the SPEAR plugins (e.g., the `SpearSim` application that we built in the [Getting Started](getting_started.md) tutorial), or it can be the Unreal Editor after it has opened any project that includes the SPEAR plugins.

Our next step is to activate our `spear-env` Anaconda environment and launch Claude Code from this repository's root directory. At this point, we can control the UE application using natural language. For example, we can ask Claude Code,

- _"I have a UE instance running, what am I looking at?"_
- _"Spawn a spotlight above the vase I'm looking at."_
- _"Move the two chairs I'm looking at so they're almost but not quite touching."_
- _"Swap the materials of the coffee table and the floor."_

## Comparison to existing MCP servers

In the table below, we compare our MCP server to existing MCP servers for interacting with the Unreal Editor, including the official MCP server that ships with Unreal 5.8. We indicate each server's ability to provide visual feedback to an agent in the form of _RGB images_ and _ID images_ that ground rendered images to specific Unreal objects. We also indicate whether each server can execute arbitrary _Python_ code in the editor Python environment, as well as Unreal _console_ commands, given by the agent. Finally, we indicate whether each server can be used to control _play-in-editor_ sessions and _standalone_ Unreal applications. In contrast to all existing MCP servers, our server can control play-in-editor sessions and standalone applications, and can ground the user's text prompt (e.g., "the two chairs I'm looking at") to specific Unreal objects using ID images.

| MCP Server | RGB images | ID images | Python | Console | Play-in-editor | Standalone |
|--------|:-----------:|:---------:|:------:|:-------:|:---:|:----------:|
| [aadeshrao123/Unreal-MCP](https://github.com/aadeshrao123/Unreal-MCP) | ✓ | | ✓ | ✓ | | |
| [ArtisanGameworks/SpecialAgentPlugin](https://github.com/ArtisanGameworks/SpecialAgentPlugin) | ✓ | | ✓ | ✓ | | |
| [ayeletstudioindia/unreal-analyzer-mcp](https://github.com/ayeletstudioindia/unreal-analyzer-mcp) | | | | | | |
| [ChiR24/Unreal_mcp](https://github.com/ChiR24/Unreal_mcp) | ✓ | | | ✓ | | |
| [chongdashu/unreal-mcp](https://github.com/chongdashu/unreal-mcp) | | | | | | |
| [db-lyon/ue-mcp](https://github.com/db-lyon/ue-mcp) | ✓ | | ✓ | ✓ | | |
| [EpicGames/UnrealEngine](https://github.com/EpicGames/UnrealEngine) (official) | ✓ | | | | | |
| [flopperam/unreal-engine-mcp](https://github.com/flopperam/unreal-engine-mcp) | ✓ | | ✓ | ✓ | | |
| [GenOrca/unreal-mcp](https://github.com/GenOrca/unreal-mcp) | ✓ | | ✓ | ✓ | | |
| [gingerol/vhcilab-unreal-engine-mcp](https://github.com/gingerol/vhcilab-unreal-engine-mcp) | | | | | | |
| [kvick-games/UnrealMCP](https://github.com/kvick-games/UnrealMCP) | ✓ | | ✓ | ✓ | | |
| [Natfii/ue5-mcp-bridge](https://github.com/Natfii/ue5-mcp-bridge) | | | | | | |
| [prajwalshettydev/UnrealGenAISupport](https://github.com/prajwalshettydev/UnrealGenAISupport) | ✓ | | ✓ | ✓ | | |
| [remiphilippe/mcp-unreal](https://github.com/remiphilippe/mcp-unreal) | ✓ | | ✓ | ✓ | | |
| [runreal/unreal-mcp](https://github.com/runreal/unreal-mcp) | ✓ | | ✓ | ✓ | | |
| [StraySpark Unreal MCP Server](https://forums.unrealengine.com/t/strayspark-unreal-mcp-server-200-ai-tools-for-ue5-editor-automation-via-mcp/2707474) | ✓ | | ✓ | ✓ | | |
| [VedantRGosavi/UE5-MCP](https://github.com/VedantRGosavi/UE5-MCP) | | | | | | |
| **spear-mcp (ours)** | **✓** | **✓** | **✓** | **✓** | **✓** | **✓** |
