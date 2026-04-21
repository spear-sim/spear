# Controlling Unreal Applications with Natural Language

We provide a [MCP](https://modelcontextprotocol.io) server that allows [Claude Code](https://claude.com/claude-code) to qury and manipulate any Unreal application.

Our first step is to launch an Unreal application for our MCP server to connect to. This can be any Unreal application that has been built with the SPEAR plugins (e.g., the `SpearSim` application that we built in the [Getting Started](getting_started.md) tutorial), or it can be the Unreal Editor that is running a project that includes the SPEAR plugins.

Our next step is to launch Claude Code from the project root directory. At this point, we can control the application using natural language. For example, we can ask Claude Code,

- _"I have a UE instance running, what am I looking at?"_
- _"Spawn a spotlight above the vase I'm looking at."_
- _"Move the two chairs I'm looking at so they're almost but not quite touching."_
- _"Swap the materials of the coffee table and the floor."_

In contrast to all existing MCP servers for the Unreal Engine (see References below), our MCP server can control standalone applications, and can link the user's text prompt (e.g., "the two chairs I'm looking at") with specific Unreal objects using an ID rendering approach. Internally, our MCP server acts on the user's behalf by writing snippets of SPEAR code, and therefore it not limited to a fixed vocabulary of actions.

## References

- [ayeletstudioindia/unreal-analyzer-mcp](https://github.com/ayeletstudioindia/unreal-analyzer-mcp)
- [ChiR24/Unreal_mcp](https://github.com/ChiR24/Unreal_mcp)
- [chongdashu/unreal-mcp](https://github.com/chongdashu/unreal-mcp)
- [db-lyon/ue-mcp](https://github.com/db-lyon/ue-mcp)
- [flopperam/unreal-engine-mcp](https://github.com/flopperam/unreal-engine-mcp)
- [GenOrca/unreal-mcp](https://github.com/GenOrca/unreal-mcp)
- [gingerol/vhcilab-unreal-engine-mcp](https://github.com/gingerol/vhcilab-unreal-engine-mcp)
- [kvick-games/UnrealMCP](https://github.com/kvick-games/UnrealMCP)
- [Natfii/ue5-mcp-bridge](https://github.com/Natfii/ue5-mcp-bridge)
- [prajwalshettydev/UnrealGenAISupport](https://github.com/prajwalshettydev/UnrealGenAISupport)
- [remiphilippe/mcp-unreal](https://github.com/remiphilippe/mcp-unreal)
- [runreal/unreal-mcp](https://github.com/runreal/unreal-mcp)
- [SpecialAgent Plugin](https://forums.unrealengine.com/t/specialagent-plugin-free-mcp-plugin-for-llm-control-of-ue-editor/2690142)
- [StraySpark Unreal MCP Server](https://forums.unrealengine.com/t/strayspark-unreal-mcp-server-200-ai-tools-for-ue5-editor-automation-via-mcp/2707474)
- [unreal-engine-mcp-server](https://hub.docker.com/r/mcp/unreal-engine-mcp-server)
- [unrealmcp](https://pypi.org/project/unrealmcp)
- [VedantRGosavi/UE5-MCP](https://github.com/VedantRGosavi/UE5-MCP)
