# Content Guidelines

These guidelines roughly follow the conventions established in the `StarterContent` scenes that ship with the Unreal Engine. That being said, we tailor these conventions to fit our use case where it makes sense to do so.

## Outliner

- The _Outliner_ view should be consistent and human-readable so it is as easy as possible to browse
- Each actor should have a meaningful name using a consistent naming convention with a consistent number of digits
- Naming conventions should be consistent across scenes

The _Outliner_ view in the Unreal Editor is organized as follows for each scene.

```
my_scene_name_0000                    # We name each scene (or "map" or "level" in Unreal parlance) using a
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

