# Content Guidelines

These guidelines roughly follow the conventions established in the `StarterContent` scenes that ship with the Unreal Engine, but we tailor the conventions to fit our use case where it makes sense to do so.

## Outliner

For each scene, the _Outliner_ pane in the Unreal Editor is organized as follows.

```
my_scene_0000                         # We name each scene (or "map" or "level" in Unreal parlance) using a
|                                     # lower_case_with_underscore naming convention and a four digit suffix
|                                     # (e.g., "apartment_0000", "apartment_0001", etc).
│                                     #
├── Debug                             # Actors that are useful for debugging are kept here.
│   │                                 #
│   ├── my_debug_actor_0000           #
│   ├── my_debug_actor_0001           #
│   └── ...                           #
│                                     #
├── Meshes                            # All Actors that represent scene geometry are kept here.
|   |                                 #
│   ├── 01_my_semantic_category       # We create a subdirectory for each semantic category using a lower_case_with_underscore
|   |   |                             # naming convention and a two digit prefix. We assign each actor to a particular
|   |   |                             # semantic category by placing the Actor in that category's subdirectory. Note
|   |   |                             # that it is possible for an individual StaticMeshComponent on an Actor to override
|   |   |                             # the actor's semantic category.
|   |   |                             #
|   |   |                             # We choose to encode semantic annotations via this folder structure, because it
|   |   |                             # makes it especially easy to browse each scene in the Unreal Editor. For example,
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
├── Navigation                        # All navigation-related Actors are kept here. We allow content here to to deviate
|   |                                 # from our usual naming convention.
│   |                                 #
│   ├── NavMeshBoundsVolume_00        #
│   ├── NavMeshModifierVolume_00      #
│   ├── NavMeshModifierVolume_01      #
│   ├── ...                           #
│   └── RecastNavMesh-Default         # RecastNavMesh actors are created automatically, and therefore we do not expect
|                                     # them to adhere to our naming conventions.
│                                     #
├── Rendering                         # All rendering-related Actors are kept here. We allow content here to to deviate
|   |                                 # from our usual naming conventions.
│   ├── Fog                           #
│   |   └── ExponentialHeightFog      #
│   ├── HDRI                          #
│   |   └── HDRIBackdrop              #
│   ├── Lights                        #
│   |   ├── DirectionalLight          #
│   |   └── SkyLight                  #
│   └── Sky                           #
│       ├── SkyAtmosphere             #
│       └── VolumetricCloud           #
│                                     #
└── Settings                          # All settings-related Actors are kept here. We allow content here to to deviate
    |                                 # from our usual naming conventions.
    |                                 #
    ├── PlayerStart                   #
    └── PostProcessVolume             #
```

- The _Outliner_ view should be as consistent and human-readable as possible (e.g., consistent names, consistent numbers of digits, etc).
- Naming conventions should be consistent across scenes.
- The type of each actor in `my_scene_0000/Meshes` should be `StaticMeshActor` if it is not articulated, and `Actor` if it is articulated. It is necessary to use the `Actor` type for articulated actors so they can be simulated correctly. But it is not strictly necessary to use the `StaticMeshActor` type for non-articulated actors. That being said, `StaticMeshActors` have a different icon than `Actors` in the _Outliner_ pane, so we use `StaticMeshActor` for non-articulated actors to make it easy to distinguish between articulated and non-articulated actors in the _Outliner_ pane.

## StaticMeshActors

For each `StaticMeshActor` in the `my_scene_0000/Meshes` directory, the _Components_ pane within the _Details_ pane is organized as follows.

```
my_actor_0000
└── StaticMeshComponent               # Each StaticMeshActor has an immutable root component named
    |                                 # "StaticMeshComponent". If the StaticMeshActor is composed of multiple  
    |                                 # StaticMeshComponents, the StaticMeshComponents can be added as children
    |                                 # of the top-level StaticMeshComponent, using the names "mesh_0000",
    |                                 # "mesh_0001", etc. This approach enables the StaticMeshComponents to be 
    |                                 # moved together in the editor, and enables them to move together during a
    |                                 # a physics simulation, without needing to explicitly merge all the 
    |                                 # actor's geometry together into a single StaticMeshComponent.
    |                                 #
    ├── mesh_0000                     #
    ├── mesh_0001                     #
    └── ...                           #
```

- If a parent `StaticMeshComponent` exists only to group other child `StaticMeshComponents` together, then the mesh assigned to the parent should be `/Game/Common/Meshes/SM_Dummy`. The "Simulate Phyiscs" option should be enabled for the parent but not for the children. This is necessary for the `StaticMeshComponents` to be simulated correctly.
- The pivot location of each `StaticMeshActor` should be set according to the following rules. The xy-coordinates of the actor's pivot should equal the xy-coordinates of the actor's axis-aligned bounding box center, and the z-coordinate of the pivot should equal the minimum z-coordinate of its axis-aligned bounding box.

## Actors

For each `Actor` in `my_scene_0000/Meshes`, the _Components_ pane within the _Details_ pane is organized as follows.

```
my_actor_0000
└── DefaultSceneRoot                  # Each Actor has an immutable root component named "DefaultSceneRoot".
    |                                 # If the StaticMeshActor is composed of multiple StaticMeshComponents,  
    |                                 # the additional StaticMeshComponents can be added as children of the
    |                                 # top-level StaticMeshComponent, using the names "mesh_0000", "mesh_0001",
    |                                 # etc. This approach enables the child StaticMeshComponents to be easily 
    |                                 # selected and moved together in the editor, and enables them to move
    |                                 # together during a physics simulation, without needing to explicitly
    |                                 # merge all the actor's geometry together into a single StaticMeshComponent. 
    |                                 #
    ├── urdf_joint_0000               # A UrdfJointComponent is a PhysicsConstraintComponent with additional state
    |                                 # and functionality. Each joint connects a parent and a child component, and
    |                                 # the joint component itself should be a sibling of the child component it is
    |                                 # connecting. We do not allow joints to connect sibling components, even
    |                                 # though this is permitted in Unreal, because this type of joint is not always
    |                                 # easy to simulate in other physics engines.
    |                                 #
    ├── urdf_joint_0001               #
    ├── ...                           #
    |                                 #
    ├── group_0000                    # A StaticMeshComponent that is only used to group other StaticMeshComponents
    |   |                             # together.
    |   |                             #
    |   ├── mesh_0000                 # A StaticMeshComponent that contains non-trivial geometry.
    |   ├── mesh_0001                 #
    |   └── ...                       #
    ├── group_0001                    #
    |   ├── mesh_0002                 #
    |   ├── mesh_0003                 #
    |   └── ...                       #
    └── ...                           #
```

- All rules for `StaticMeshActors` described above also apply to `Actors`.
- When configuring a joint, the convention in Unreal is for _Component 1_ to be the child and _Component 2_ to be the parent. This convention is relevant, e.g., when the "Parent Dominates" flag is enabled on the joint.
