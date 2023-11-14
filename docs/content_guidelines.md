# Content Guidelines

We represent each SPEAR scene as a distinct Unreal _map_. The Unreal documentation sometimes refers to a _map_ as a _level_. However, we prefer the term _scene_ in this document because we think it is more familiar to the embodied AI community.

## Filesystem

For each scene, we expect it to be organized on the filesystem as follows. If a scene is organized in this way, then our tools will be able to do everything they need to do, i.e., export the scene for use in third-party physics simulators, and generate an appropriate data file that will enable our `spear` Python module to load the scene.

```
SpearSim                               # Our top-level Unreal project folder: spear/cpp/unreal_projects/SpearSim
├── ...                                #
├── Content                            #
|   ├── Common                         # Common assets are kept in the {Common, Kujiale, StarterContent} directories.
|   |                                  #
|   ├── Kujiale                        #
|   ├── Scenes                         # Each scene inside the Scenes directory is allowed to refer to assets in the {Common,
|   |   |                              # Kujiale, StarterContent} directories, but is not allowed to refer to assets in other 
|   |   |                              # scene directories, e.g., my_scene_0000 is not allowed to refer to assets in the
|   |   |                              # my_scene_0001 directory below. This restriction makes it easier to support an editing 
|   |   |                              # workflow where a developer only needs to download a single subdirectory in order to 
|   |   |                              # obtain a self-contained copy of a scene.
|   |   |                              #
|   |   ├── ...                        #
|   |   ├── my_scene_0000              #
|   |   |   ├── ...                    #
|   |   |   ├── Maps                   #
|   |   |   |   └── my_scene_0000.umap # There must be exactly one umap file with the same name as the scene in the scene's
|   |   |   |                          # Maps directory. Our code imposes this requirement to support loading scenes by name.
|   |   |   |                          #
|   |   |   └── ...                    #
|   |   ├── my_scene_0001              #
|   |   |   ├── ...                    #
|   |   |   ├── Maps                   #
|   |   |   |   └── my_scene_0001.umap #
|   |   |   └── ...                    #
|   |   └── ...                        #
|   └── StarterContent                 #
└── ...                                #
```

## Unreal Editor

A scene can be loaded in the Unreal Editor by opening `spear/cpp/unreal_projects/SpearSim/SpearSim.uproject`, and then double-clicking on `/Game/Scenes/my_scene_0000/Maps/my_scene_0000` in the _Content Browser_.

### Outliner pane

For each scene, the _Outliner_ pane is organized as follows.

```
my_scene_0000                          # We name each scene (using a lower_case_with_underscore naming convention and a four digit
|                                      # suffix (e.g., "apartment_0000", "apartment_0001", etc).
│                                      #
├── Debug                              # Actors that are useful for debugging are kept here.
│   │                                  #
│   ├── my_debug_actor_0000            #
│   ├── my_debug_actor_0001            #
│   └── ...                            #
├── Meshes                             # All Actors that represent scene geometry are kept here.
|   |                                  #
│   ├── 01_my_semantic_category        # We create a subdirectory for each semantic category using a lower_case_with_underscore
|   |   |                              # naming convention and a two digit prefix. We assign each actor to a particular
|   |   |                              # semantic category by placing the Actor in that category's subdirectory. Note
|   |   |                              # that it is possible for an individual StaticMeshComponent on an Actor to override
|   |   |                              # the actor's assigned semantic category.
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
│   ├── 02_my_semantic_category        #
│   └── ...                            #
├── Navigation                         # All navigation-related Actors are kept here. For convenience, we allow content here to 
|   |                                  # deviate from our usual naming conventions.
│   |                                  #
│   ├── NavMeshBoundsVolume_00         #
│   ├── NavMeshModifierVolume_00       #
│   ├── NavMeshModifierVolume_01       #
│   ├── ...                            #
│   └── RecastNavMesh-Default          # RecastNavMesh actors are created automatically, and therefore we do not expect
|                                      # them to adhere to our naming conventions.
│                                      #
├── Rendering                          # All rendering-related Actors are kept here. For convenience, we allow content here to 
|   |                                  # deviate from our usual naming conventions.
│   |                                  #
│   ├── Fog                            #
│   |   └── ExponentialHeightFog       #
│   ├── HDRI                           #
│   |   └── HDRIBackdrop               #
│   ├── Lights                         #
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

- The _Outliner_ view should be as consistent and human-readable as possible (e.g., consistent names, consistent numbers of digits, etc).
- Naming conventions should be consistent across scenes.
- The type of each actor in `my_scene_0000/Meshes` should be `StaticMeshActor` if it is not articulated, and `Actor` if it is articulated. It is necessary to use the `Actor` type for articulated actors so they can be simulated correctly. Although it is not strictly necessary to use the `StaticMeshActor` type for non-articulated actors, they have a different icon than `Actors` in the _Outliner_ pane, so we use `StaticMeshActor` for non-articulated actors to visually distinguish them from articulated actors.

### Details pane

#### `StaticMeshActor`

For each `StaticMeshActor` in the `my_scene_0000/Meshes` directory, the _Components_ pane within the _Details_ pane is organized as follows.

```
my_actor_0000
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
                                       # StaticMeshComponent in order to show up in semantic segmentation images, to be referred
                                       # to by name in our spear Python module, and to precisely control how our export tools
                                       # should export this StaticMeshActor to other physics simulators.
```

- If the parent `StaticMeshComponent` exists only to group other child `StaticMeshComponents` together, and it is desired for the group to participate in a physics simulation, then the mesh assigned to the parent should be `/Game/Common/Meshes/SM_Dummy`, and the _Simulate Phyiscs_ flag should be enabled for the parent but not for the children. This is necessary for the group to be simulated correctly.
- Extra layers of grouping hierarchy can be implemented by creating a tree of `StaticMeshComponents`. See the `Actor` section below for a sensible naming convention.
- The pivot location of each `StaticMeshActor` should be set according to the following rules. The xy-coordinates of the actor's pivot should equal the xy-coordinates of the actor's axis-aligned bounding box center, and the z-coordinate of the pivot should equal the minimum z-coordinate of its axis-aligned bounding box. This is the convention for various props that ship with the Unreal Engine (e.g., the props in our `debug_0000` scene).

#### `Actor`

For each `Actor` in `my_scene_0000/Meshes`, the _Components_ pane within the _Details_ pane is organized as follows.

```
my_actor_0000
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
    ├── metadata_0000                  # Each Actor must have a MetadataComponent that is a child of the top-level Component
    |                                  # in order to show up in the spear Python module's semantic segmentation images, to be 
    |                                  # referred to by name in the spear Python module, and to precisely control how our export
    |                                  # tools should export this Actor to other physics simulators.
    |                                  #
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
- When configuring a joint, the convention in Unreal is for _Component 1_ to be the child and _Component 2_ to be the parent. This convention is relevant, e.g., when the _Parent Dominates_ flag is enabled on the joint.
