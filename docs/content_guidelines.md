# Content Guidelines

We represent each SPEAR scene as a distinct Unreal _map_. The Unreal documentation sometimes refers to a _map_ as a _level_. However, we prefer the term _scene_ in this document because we think it is more familiar to the embodied AI community.

## Filesystem

Each scene should be organized on the filesystem as follows. If a scene is organized in this way, then our tools will be able to do everything they need to do, i.e., export the scene for use in third-party physics simulators, and generate data files that will enable our `spear` Python module to load the scene.

```
SpearSim                               # Our top-level Unreal project folder: spear/cpp/unreal_projects/SpearSim
|                                      #
└── Content                            #
    ├── Common                         # Assets that are referenced in multiple scenes are kept in the Common directory.
    |                                  # If a collection of scenes needs to refer to additional common assets, those assets
    |                                  # should be kept in sibling directories to Common (e.g., the kujiale scenes refer to
    |                                  # assets in {Kujiale, Megascans, MSPresets}, the debug scenes refer to assets in 
    |                                  # {StarterContent}, etc).
    |                                  #
    └── Scenes                         # We name each scene using a lower_case_with_underscore naming convention and a four
        |                              # digit suffix (e.g., "apartment_0000", "apartment_0001", etc).
        |                              #
        |                              # Each scene inside the Scenes directory is allowed to refer to assets in common
        |                              # directories, but is not allowed to refer to assets in other scene directories,
        |                              # e.g., my_scene_0000 is not allowed to refer to assets in the my_scene_0001 directory
        |                              # below. This restriction makes it easier to support an editing workflow where a
        |                              # developer only needs to download a single subdirectory in order to obtain a
        |                              # complete self-contained copy of a scene.
        |                              #
        ├── my_scene_0000              #
        |   ├── ...                    #
        |   ├── Maps                   #
        |   |   └── my_scene_0000.umap # There should be exactly one umap file with the same name as the scene in the scene's
        |   |                          # Maps directory. Our code imposes this requirement to support loading scenes by name.
        |   |                          #
        |   └── ...                    #
        ├── my_scene_0001              #
        └── ...                        #
```

## Unreal Editor

All options should be set to their default values unless noted below, or unless there is an obvious reason to deviate.

### Outliner pane

For each scene, the _Outliner_ pane should be organized as follows.

```
my_scene_0000                          #
├── Debug                              # Actors that are useful for debugging are kept here.
│   │                                  #
│   ├── my_debug_actor_0000            #
│   ├── my_debug_actor_0001            #
│   └── ...                            #
├── Meshes                             # All Actors that represent non-trivial scene geometry are kept here.
|   |                                  #
│   ├── 0001_my_semantic_category      # We create a subdirectory for each semantic category using a lower_case_with_underscore
|   |   |                              # naming convention and a four digit prefix that represents the category's integer ID.
|   |   |                              # We assign each Actor to a particular semantic category by placing the Actor in that
|   |   |                              # category's subdirectory. Semantic ID 0 is reserved and should not be used here. If an
|   |   |                              # Actor has multiple StaticMeshComponents that should have different categories, the
|   |   |                              # category of individual StaticMeshComponents can be overridden. In this case, the Actor
|   |   |                              # should be assigned to whatever subdirectory is most convenient, i.e., requires the least
|   |   |                              # number of per-component overrides.
|   |   |                              #
|   |   |                              # We choose to encode semantic annotations via this directory structure, because it
|   |   |                              # makes it especially easy to browse each scene in the Unreal Editor. For example,
|   |   |                              # using this directory structure, a user can easily select all objects in a particular
|   |   |                              # semantic category and make them invisible, thereby making it easier to browse the
|   |   |                              # rest of the scene.
|   |   |                              #
│   │   ├── my_actor_0000              #
│   │   ├── my_actor_0001              #
│   │   └── ...                        #
│   ├── 0002_my_semantic_category      #
│   └── ...                            #
├── Navigation                         # All navigation-related Actors are kept here. There must be at least one RecastNavMesh 
|   |                                  # Actor in the scene in order to use the navmesh functionality in our Python API.
│   |                                  #
│   ├── NavMeshBoundsVolume_00         #
│   ├── NavMeshModifierVolume_00       #
│   ├── NavMeshModifierVolume_01       #
│   ├── ...                            #
│   └── RecastNavMesh-Default          # RecastNavMesh Actors are created automatically, and therefore we do not expect
|                                      # them to adhere to our usual naming conventions.
│                                      #
├── Rendering                          # All rendering-related Actors are kept here. For convenience, we allow content here to 
|   |                                  # deviate from our usual naming conventions.
│   |                                  #
│   ├── Fog                            #
│   |   └── ExponentialHeightFog       #
│   ├── HDRI                           #
│   |   └── HDRIBackdrop               #
│   ├── Lights                         # Only outdoor lights are allowed here. All indoor lights should be represented as a
│   |   |                              # LightComponent attached to a StaticMeshActor or Actor in the Meshes directory. This
│   |   |                              # convention is advantageous because it will force the LightComponent to move together
│   |   |                              # with the StaticMeshComponents that represent the light.
│   |   |                              #
│   |   ├── DirectionalLight           #
│   |   └── SkyLight                   #
│   └── Sky                            #
│       ├── SkyAtmosphere              #
│       └── VolumetricCloud            #
└── Settings                           # All settings-related Actors are kept here.  For convenience, we allow content here to 
    |                                  # deviate from our usual naming conventions.
    |                                  #
    ├── PlayerStart                    #
    └── PostProcessVolume              #
```

- The _Outliner_ pane should be organized as consistently as possible across scenes, and should be as human-readable as possible (e.g., consistent names, consistent numbers of digits, etc).
- The type of each actor in `my_scene_0000/Meshes` should be `StaticMeshActor` if it is not articulated, and `Actor` if it is articulated. It is necessary to use the `Actor` type for articulated actors so they can be correctly physically simulated. Although it is not strictly necessary to use the `StaticMeshActor` type for non-articulated actors, `StaticMeshActors` and `Actors` have different icons in the _Outliner_ pane, so we use different types to visually differentiate them.

### Details pane

#### `StaticMeshActor`

For each `StaticMeshActor` in the `my_scene_0000/Meshes` directory, the _Components_ pane within the _Details_ pane should be organized as follows.

```
my_actor_0000                          #
└── StaticMeshComponent                # Each StaticMeshActor has a root component named "StaticMeshComponent".
    |                                  #
    |                                  # If the StaticMeshActor is composed of multiple StaticMeshComponents, the additional
    |                                  # StaticMeshComponents can be added as children of the top-level StaticMeshComponent,
    |                                  # using the names "mesh_0000", "mesh_0001", etc. This approach enables the child
    |                                  # StaticMeshComponents to be easily selected and moved together in the editor, and
    |                                  # enables them to move together during a physics simulation, without needing to
    |                                  # explicitly merge all the actor's geometry together into a single StaticMeshComponent. 
    |                                  #
    ├── mesh_0000                      #
    ├── mesh_0001                      #
    ├── ...                            #
    └── metadata_0000                  # Each StaticMeshActor must have a MetadataComponent that is a child of the top-level
                                       # StaticMeshComponent in order to enable various SPEAR functionality.
```

- Extra layers of grouping hierarchy can be implemented by creating a tree of `StaticMeshComponents`. See the `Actor` section below for a sensible naming convention.
- The pivot location of each `StaticMeshActor` should be set according to the following rules. The xy-coordinates of the actor's pivot should equal the xy-coordinates of the actor's axis-aligned bounding box center, and the z-coordinate of the pivot should equal the minimum z-coordinate of its axis-aligned bounding box. Having a consistent convention here makes it easier to programmatically spawn objects, and this is the convention for various props that ship with the Unreal Engine (e.g., the props in our `debug_0000` scene).
- If the parent `StaticMeshComponent` exists only to group other child `StaticMeshComponents` together, and it is desired for the group to be physically simulated, then the mesh assigned to the parent component should be `/Game/Common/Meshes/SM_Dummy`, and the _Simulate Phyiscs_ option should be enabled for the parent but not for the children. This is necessary for the group to be correctly physically simulated.
- For each `StaticMeshComponent`, the _Collision Presets_ option should be set to _Default_. This configures the collision behavior of the component to be determined by the options on the underlying mesh, rather than the options on the component itself.

#### `Actor`

For each `Actor` in `my_scene_0000/Meshes`, the _Components_ pane within the _Details_ pane should be organized as follows.

```
my_actor_0000                          #
└── DefaultSceneRoot                   # Each Actor has a root component named "DefaultSceneRoot".
    |                                  #
    |                                  # We expect Actors to be articulated, and therefore they will be composed of multiple
    |                                  # StaticMeshComponents. The additional StaticMeshComponents should be added as children
    |                                  # of a parent StaticMeshComponent. Each StaticMeshComponent that is only used to group
    |                                  # other StaticMeshComponents together should be should be named "group_0000",
    |                                  # "group_0001", etc. Each StaticMeshComponent that contains non-trivial scene geometry 
    |                                  # should be named "mesh_0000", "mesh_0001", etc.
    |                                  #
    ├── group_0000                     #
    |   ├── mesh_0000                  #
    |   ├── mesh_0001                  #
    |   └── ...                        #
    ├── group_0001                     #
    |   ├── mesh_0002                  #
    |   ├── mesh_0003                  #
    |   └── ...                        #
    ├── metadata_0000                  #
    ├── urdf_joint_0000                # A UrdfJointComponent represents a joint, and is implemented as a derived class of
    |                                  # PhysicsConstraintComponent with additional state and functionality. Each joint connects
    |                                  # a parent and a child component, and the joint component itself should be a sibling of
    |                                  # the child component it is connecting. In our guidelines, we do not allow joints to
    |                                  # connect sibling components, or grandparents and grandchildren, even though this would
    |                                  # be permitted in Unreal, because this type of joint cannot always be simulated
    |                                  # efficiently in other physics engines.
    |                                  #
    ├── urdf_joint_0001                #
    └── ...                            #
```

- All rules for `StaticMeshActors` described above also apply to `Actors`.
- When configuring a joint, the convention in Unreal is for _Component 1_ to be the child and _Component 2_ to be the parent. This convention is relevant, e.g., when the _Parent Dominates_ option is enabled on the joint.

### Content Browser pane

For each scene, the _Content Browser_ pane should be organized as follows.

```
All                                    #
└── Content                            #
    └── Scenes                         #
        ├── my_scene_0000              #
        |   ├── Debug                  #
        |   ├── Maps                   #
        |   ├── Materials              #
        |   └── Meshes                 #
        ├── my_scene_0001              #
        └── ...                        #
```

- The `Debug`, `Maps`, `Materials`, `Meshes` directories can be organized using whatever naming conventions make sense for a particular scene, but whatever convention is chosen, it should be as consistent as possible across scenes. For example, in the `kujiale` scenes, we name each asset in the _Content Browser_ according to the asset's globally unique ID in our internal systems. In general, it is acceptable if the _Content Browser_ sacrifices some human-readability in exchange for convenience. In contrast, the _Outliner_ should be completely human-readable.
- Individual assets should be named using a prefix that indicates the asset type (e.g., `M_` for materials, `MI_` for material instances, `SM_` for static meshes, `T_` for textures, etc). See [here](https://docs.unrealengine.com/5.2/en-US/recommended-asset-naming-conventions-in-unreal-engine-projects) for more specific guidelines.
- The _Content Browser_ should not contain unreferenced assets, except for in the `Debug` directory.

### Static Mesh editor

- The _Collision Presets_ option should be set to _BlockAll_.
- The _Customized Collision_ option should be enabled.

## Guidelines for Kujiale scenes

The following guidelines are specific to the `kujiale` scenes.

### Content Browser pane

For each scene, the _Content Browser_ pane is organized as follows.

```
All                                    #
└── Content                            #
    ├── Kujiale                        # Kujiale scenes are allowed to refer to assets in {Kujiale, Megascans, MSPresets}.
    |                                  #
    ├── Megascans                      #
    ├── MSPresets                      #
    |                                  #
    └── Scenes                         #
        ├── kujiale_0000               #
        |   ├── Maps                   #
        |   |   └── kujiale_0000.umap  #
        |   ├── Materials              # Each material is kept here in a directory named according to the material's
        |   |   |                      # globally unique ID in our internal systems.
        |   |   |                      #
        |   |   ├── 54213fb3254e415... #
        |   |   └── ...                #
        |   └── Meshes                 #
        |       ├── Base               # We organize "base" meshes according to their semantic category. This human-
        |       |   |                  # readable convention is possible because of how these meshes are generated and
        |       |   |                  # stored in our internal systems.
        |       |   |                  #
        |       |   ├── Cabinet        #
        |       |   ├── Ceiling        #
        |       |   ├── Door           #
        |       |   └── ...            #
        |       └── Clutter            # For "clutter" meshes, it is not straightforward to organize them by semantic
        |           |                  # category, so instead we organize them according to their globally unique ID.
        |           |                  #
        |           ├── 6AMHYPETD5T... #
        |           └── ...            #
        └── ...                        #
```
