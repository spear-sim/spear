# Virtual World Manager

Virtual World Manager is an Unreal Plugin providing API for Virtual World loading, and modification such as asset
layout, lighting, etc at run time.

### Loading Virtual World for standalone executables

Use SceneManager to download .pak files for VirtualWorld then move the .pak files to `<path_to_standalone>/RobotProject/Content/Paks`. Then start the simulator and .pak files will be
mounted.
* `VWLevelmanager::GetAllMapsInPak`: find all available level names from mounted .pak.

### Non-photorealistic Rendering
Apply post process materials to change the rendering configuration without changing the scene.

Example: 
Add PostProcessMaterial to PostProcessVolume to change the rendering style for entire scene
```commandline
    for (TActorIterator<APostProcessVolume> it(this->GetWorld()); it; ++it) {
            (*it)->AddOrUpdateBlendable(level_manager_->getPostProcessMaterial(EPostProcessMaterialType::CelShader,1);
        }
        count++;
    }
```

Add PostProcessMaterial to SceneCapture to change the rendering style for single image
```commandline
    CaptureComponent->GetCaptureComponent2D()->AddOrUpdateBlendable(level_manager_->getPostProcessMaterial(EPostProcessMaterialType::CelShader,1);
```

### more coming