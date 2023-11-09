# Content Guidelines

These guidelines roughly follow the conventions established in the `StarterContent` scenes that ship with the Unreal Engine. That being said, we tailor these conventions to fit our use case where it makes sense to do so.

## Outliner

For each scene, the _Outliner_ pane in the Unreal Editor is organized as follows.

```
my_scene_0000                         # We name each scene (or "map" or "level" in Unreal parlance) using a
|                                     # lower_case_with_underscore naming convention and a four digit suffix
|                                     # (e.g., apartment_0000, apartment_0001, etc).
│                                     #
├── Debug                             # Actors that are useful for debugging are cleanly separated from the rest of the
|   |                                 # scene here.
│   │                                 #
│   ├── my_debug_actor_0000           #
│   ├── my_debug_actor_0001           #
│   └── ...                           #
│                                     #
├── Meshes                            # All Actors that represent scene geometry are kept here.
|   |                                 #
│   ├── 01_my_semantic_category       # We create a subdirectory for each semantic category using a lower_case_with_underscore
|   |   |                             # naming convention and a two digit prefix. We assign each actor to a particular
|   |   |                             # semantic category by placing the Actor in a subdirectory. Note that it is
|   |   |                             # possible for an individual StaticMeshComponent on an Actor to override the
|   |   |                             # Actor's semantic category.
|   |   |                             # 
|   |   |                             # We choose to encode semantic annotations via this folder structure, because it
|   |   |                             # makes it especially easy to  browse each scene in the Unreal Editor. For example,
|   |   |                             # using this folder structure, a user can easily select all objects in a particular
|   |   |                             # semantic category and make them invisible, thereby making it easy to browse the
|   |   |                             # rest of the scene.
|   |   |                             #
│   │   ├── my_actor_0000             #
│   │   ├── my_actor_0001             #
│   │   └── ...                       #
│   ├── 02_my_semantic_category       #
│   └── ...                           #
│                                     #  
├── Navigation                        # All navigation-related Actors are kept here.
│   |                                 #
│   ├── nav_mesh_bounds_volume_0000   #
│   ├── mav_mesh_modifier_volume_0000 #
│   ├── mav_mesh_modifier_volume_0001 #
│   ├── ...                           #
│   └── RecastNavMesh-Default         # This Actor is created automatically, and therefore does not adhere to our
|                                     # naming conventions.
│                                     #
├── Rendering                         #
│   └── HDRI                          # All HDRI-related Actors are kept here.
│       |                             #
│       └── hdri_backdrop_0000        #
│                                     #
└── Settings                          # All settings-related Actors are kept here.
    |                                 #
    ├── player_start_0000             #
    └── post_process_volume_0000      #
```

- The _Outliner_ view should be as consistent and human-readable as possible (e.g., consistent names, consistent numbers of digits, etc).
- Naming conventions should be consistent across scenes.
- The type of each actor in `my_scene_0000/Meshes` should be `StaticMeshActor` if it is not articulated, and `Actor` if it is articulated.

## StaticMeshActors

The _Components_ pane within the _Details_ pane for each `StaticMeshActor` is organized as follows.

```
my_actor_0000
└── StaticMeshComponent               # Each StaticMeshActor has an immutable root component named
    |                                 # "StaticMeshComponent". If the StaticMeshActor is composed of multiple  
    |                                 # StaticMeshComponents, the StaticMeshComponents can be added as children
    |                                 # of the top-level StaticMeshComponent, using the names mesh_0000,
    |                                 # mesh_0001, etc. This approach enables the StaticMeshComponents to be 
    |                                 # moved together in the editor, and enables them to move together during a
    |                                 # a physics simulation, without needing to explicitly merge all the 
    |                                 # actor's geometry together into a single StaticMeshComponent.
    |                                 #
    ├── mesh_0000                     #
    ├── mesh_0001                     #
    └── ...                           #
```

- If a `StaticMeshComponent` exists only to group other `StaticMeshComponents` together, then the mesh assigned to it should be `/Plugins/CoreUtils/Meshes/SM_Dummy`. This step is necessary for the group to be simulated correctly.
- The pivot location of each `StaticMeshActor` should be set according to the following rules. The xy-coordinates of the actor's pivot should equal the xy-coordinates of the actor's axis-aligned bounding box center, and the z-coordinate of the pivot should equal the minimum z-coordinate of itS axis-aligned bounding box.
