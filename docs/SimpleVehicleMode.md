# SimpleVehicle Mode 
SimpleVehicle mode uses the UE4 Vehicle Module to provide agent with differential driving control and better stability compared to UrdfBot mode.

Example agent used is [OpenBot](https://github.com/isl-org/OpenBot) with four wheels differential driving robot.

### How to load SimpleVehicle
In `RobotSim.Build.cs`, add UE4 dependency `PhysXVehicles` and `NavigationSystem`

In `RobotSim/setting/setting.json`, change `SimMode` to `SimpleVehicle`, and update corresponding settings. 

Example settings for OpenBot can be found in [settings_OpenBot.json](code/unreal_plugins/RobotSim/setting/settings_OpenBot.json)

see [ActorSpawn.md](ActorSpawn.md) for agent spawning location setting. 

### Keyboard control
Use arrow key or `wasd` to drive the agent when `EnableKeyboard` is true.

Disable keyboard control to drive agent using API directly (Keyboard input will override API cmd).

---

#### For Test Only
Press `1` to spawn a new agent if not exists.

Press `2` to destroy current agent.
