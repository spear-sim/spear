# Data Structure for Kujiale Scenes

### Design Philosophy

* uasset naming - follow unreal style
* umap folder - only available in editor. Target designing it easy for Tech Artists to use.
    * Use separated folder for StaticMeshActors in categories that require special action.
* actor name - contain sufficient information to make it easy to identify in editor, and code should not rely on it.
* actor tags - primary place to store important information accessed by code, such as semantic label, instanceId, model
  functions(architecture, light, table)

### Content Structure

```
Content
├── Scenes
│   ├── debug_0001
│   │   ├── Maps
│   │   │   └── debug_0001.umap - level 
│   │   ├── Materials
│   │   │   ├── MaterialId_<MaterialId>
│   │   │   │   ├── M_<MaterialId>.uasset - material instance from BaseMaterial
│   │   │   │   ├── T_<MaterialId>_diffuse.uasset
│   │   │   │   └── MI_<MaterialId>_index001.uasset - material instance from M_<MaterialId> with different world offset for z-fighting issue
│   │   │   └── ...
│   │   └── Meshes
│   │       ├── Architecture
│   │       │   ├── <Category>
│   │       │   │   ├── SM_obj<SubMeshId>.uasset - StaticMesh that must live with current scene. pivot point is origin of the scene.
│   │       │   │   └── ...
│   │       │   └── ...
│   │       └── Furniture
│   │           ├── <MeshId>
│   │           │   ├── SM_<MeshId>_obj<SubMeshId>.uasset - StaticMesh that can live in any scene. Pivot point is within mesh geometry.
│   │           │   ├── SM_<MeshId>_obj2.uasset
│   │           │   └── ...
│   │           └── ...
│   ├── kujiale_0000
│   └── ...
└── Shared
    └── kujiale
        ├── IES
        │   ├── 1.uasset - IES texture file   
        │   └── ...
        ├── Materials
        │   ├── BaseMaterial.uasset  - material template for all kujiale materials. should not be used directly.
        │   ├── TranslucentMaterial.uasset  - material template for all translucent kujiale materials. should not be used directly.
        │   └── Functions/MF_Base.uasset
        ├── Meshes
        │   └── SM_Small_Sphere.uasset - dummpy StaticMesh with nearly invisible geometry and collision. 
        ├── PhysicalMaterials
        │   ├── PM_1000.uasset - physical material
        │   └── ...
        └── Textures
            ├──  normal.uasset - default textures for BaseMaterial.uasset
            └──  ...
```

### Umap Structure

```
umap
├── BaseSetting
│   ├── PostProcessVolume
│   └── PlayerStart
├── Lighting
│   ├── DirectionalLight
│   └── HDRIBackdrop
├── NavMeshes
│   ├── NavMeshBoundsVolume
│   └── NavModifierVolume
└── StaticMeshes
    ├── Architecture
    │   ├── Ceilings
    │   │   ├── Archetecture_Ceiling
    │   │   ├── Archetecture_obj<InstanceId>
    │   │   ├── Archetecture_obj1
    │   │   └── ...
    │   ├── Floors
    │   │   ├── Archetecture_Floor
    │   │   ├── Archetecture_obj2
    │   │   └── ...
    │   └── Walls
    │       ├── Archetecture_obj3
    │       └── ...
    ├── ArticulatedMeshes
    │   └── ArticulatedMesh_obj4
    ├── Lights -- 
    │   ├── CeilingLight
    │   ├── Chandelier
    │   ├── Lamp
    │   │   ├── SM_<meshId>_obj_<instanceId>
    │   └── WallLight
    ├── Table
    └── Other
```

```
ArticulatedMeshActor
└── DefaultRootComponent- dummpy SceneComponent
    ├── Static -- static component
    │   ├── Architecture1
    │   ├── <meshId>_obj1
    │   └── ...
    ├── Animation_<AnimationId> - movable section of articluated object
    │   ├── Architecture2
    │   ├── <meshId>_obj2
    │   ├── <meshId>_obj3
    │   └── ...
    ├── PCC_Animation_<AnimationId> - physical constraint component
    ├── Animation_2
    └── PCC_Animation_2
```

```
LightMeshActor
└── StaticMeshComponent - dummy StaticMeshComponent
    ├── obj1_1 - StaticMeshComponent
    ├── obj2_1
    ├── ...
    └── PointLight - light component
```

### TODO
1. actor name convention
2. actor tag convention
