# Content Guidelines

These conventions roughly follow the `StarterContent` scenes that ship with the Unreal Engine, although we have tailored the conventions to fit our desired use-cases where it made sense to do so.

## Outliner

The _Outliner_ view in the Unreal Editor is organized as follows for each scene. 

```
my_scene_name_0000                  # Our convention is to give each Actor a consistent human-readable name, using a
|                                   # consistent naming convention, and using integer indices with a consistent number
|                                   # of digits. We strongly prefer to maintain consistency of these conventions
|                                   # across scenes.
│                                   #
├── Debug                           # Actors that are useful for debugging are cleanly separated from the rest of the
|   |                               # scene here.
│   │                               #
│   ├── my_debug_actor_0000         #
│   ├── my_debug_actor_0001         #
│   ├── ...                         #
│   └── my_debug_actor_AAAA         #
│                                   #
├── Meshes                          # All meshes that might participate in the physics simulation are kept here. All
|   |                               # meshes that have a non-zero semantic category are also kept here. Typically there 
|   |                               # is one semantic category per Actor, but occasionally we override the semantic 
|   |                               # category for an individual StaticMeshComponent. We choose to encode semantic
|   |                               # annotations via this folder structure because it makes it especially easy to 
|   |                               # browse each scene in the Unreal Editor. For example, using this folder structure,
|   |                               # a user can easily select all objects in a particular semantic category and make
|   |                               # them invisible, thereby making it easy to browse the rest of the scene.
|   |                               #
│   ├── 01_my_semantic_category     #
│   │   ├── my_actor_0000           #
│   │   ├── my_actor_0001           #
│   │   ├── ...                     #
│   │   └── my_actor_AAAA           #
│   ├── 02_my_semantic_category     #
│   ├── ...                         #
│   └── SS_my_semantic_category     #
│                                   #  
├── Navigation                      # All navigation-mesh-related Actors are kept here.
│   |                               #
│   ├── NavMeshBoundsVolume         #
│   ├── NavMeshModifierVolume_00    #
│   ├── NavMeshModifierVolume_01    #
│   ├── ...                         #
│   ├── NavMeshModifierVolume_NN    #
│   └── RecastNavMesh-Default       #
│                                   #
├── Rendering                       #
│   └── HDRI                        # All HDRI-related Actors are kept here.
│       |                           #
│       └── HDRIBackdrop            #
│                                   #
└── Settings                        # All settings-related Actors are kept here.
    |                               #
    ├── PlayerStart                 #
    └── PostProcessVolume           #
```
