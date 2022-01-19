# Actor Spawn 
RobotSim provides several functionalities and flexible setting for choosing appropriate robot spawn location in the Virtual World.

### Configure Spawn Location in Settings.json
In settings.json, there are several setting related to spawn location.
```
    "Vehicles": {
        "<VehicleName>": {
            "EnableRandomSpawn": true,
            "SpawnTransform": {
                "X": 0,
                "Y": 0,
                "Z": 0,
                "Yaw": 0,
                "Pitch": 0,
                "Roll": 0
            },
            "EnableSpawnTracingGround": true,
            "BoundingBox": {
                "X": 0.3,
                "Y": 0.3,
                "Z": 0.205
            }
            ...
        }
```
`EnableRandomSpawn`: bool, default `true`. If true, agent will spawn at a random location based on NavMesh.

`EnableSpawnTracingGround` can be enabled at the same time, while `SpawnTransform` will be neglected. 

`SpawnTransform`: vector, extra offset transformation from `PlayerStart`. Neglected if `EnableRandomSpawn` is enabled.  

`EnableSpawnTracingGround`: bool, default `true`. If true, enable ground tracing to find a location near ground to minimize instability after spawn. 

`BoundingBox`: vector, bounding box for location. Used in `UrdfBotPawn` mode to determine appropriate box size to trace ground. 


### Ground Tracing
When agent spawns away from ground, waiting for agent to be settled down may take many ticks, and it is tricky to determine whether agent is stabilized.
RobotSim uses box tracing to detect location closest to ground for agent spawn. 
Box tracing uses agent collision box to find all collision along z-axis, and returns first hit location as the spawn location.
For more info, see [Unreal Doc](https://docs.unrealengine.com/4.27/en-US/InteractiveExperiences/Tracing/Overview/)

Note that for current version, ground tracing cannot determine the best location if spawn location is not perfectly flat, agent might be tilted in special cases such as carpet edge.  

### NavMesh for Random Spawn 
For Random spawn, RobotSim uses NavMesh ([Unreal Doc](https://docs.unrealengine.com/4.27/en-US/Resources/ContentExamples/NavMesh/)) to identify sections in the world that allow agent to spawn and move. 
Certain area with height above threshold are neglected, such as ceiling, bed and table.

Note that currently this method does not guarantee agent stability after spawn. The agent might be tilted if spawned at carpet edge. 

In order for NavMesh to function properly, there are few setup for the Virtual World.
1. Config NavMesh in `DefaultEngine.ini`
```buildoutcfg
[/Script/NavigationSystem.RecastNavMesh]
bAutoDestroyWhenNoNavigation=False
bForceRebuildOnLoad=True
RuntimeGeneration=Dynamic
```
2. Add a `NavMeshBoundsVolume` Actor to the virtual world if there is not.
3. In Virtual World, `NavMeshBoundsVolume` uses the bounding box of all actors in the world with tag `architecture` or `furniture`
