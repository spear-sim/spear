# SPEAR: A Simulator for Photorealistic Embodied AI Research

![control_sample_projects](https://github.com/user-attachments/assets/83e502ca-aa8b-44c5-b57d-86ca3b8958e7)

_SPEAR is a Python library that can connect to, and programmatically control, any Unreal Engine (UE) application via a modular plugin architecture. SPEAR exposes over 14K unique UE functions, representing an order-of-magnitude increase in programmable functionality over existing simulators. We demonstrate the flexibility of SPEAR by using it to control 6 distinct embodied agents (each with a different action space) across several Epic Games sample projects: a person and a car from `CitySample` (top); a flying robot from `StackOBot` (bottom far left); multiple agents in a resource collecting game called `CropoutSample` (bottom center left); as well as a person with parkour skills and a quadruped robot from `GameAnimationSample` (bottom right)._

## Abstract

Interactive simulators have become powerful tools for training embodied agents and generating synthetic visual data, but existing photorealistic simulators suffer from limited generality, programmability, and rendering speed. We address these limitations by introducing _SPEAR: A Simulator for Photorealistic Embodied AI Research_. At its core, SPEAR is a Python library that can connect to, and programmatically control, any Unreal Engine (UE) application via a modular plugin architecture. SPEAR exposes over 14K unique UE functions to Python, representing an order-of-magnitude increase in programmable functionality over existing UE-based simulators. Additionally, a single SPEAR instance can render 1920x1080 photorealistic beauty images directly into a user's NumPy array at 56 frames per second -- an order of magnitude faster than existing UE plugins -- while also providing ground truth image modalities that are not available in any existing UE-based simulator (e.g., a non-diffuse intrinsic image decomposition, material IDs, and physically based shading parameters). Finally, SPEAR introduces an expressive high-level programming model that enables users to specify complex graphs of UE work with arbitrary data dependencies among work items, and to execute these graphs deterministically within a single UE frame. We demonstrate the utility of SPEAR through a diverse collection of example applications: controlling multiple embodied agents with distinct action spaces (e.g., humans, cars, and robots) across several in-the-wild UE projects; rendering photorealistic city-scale environments; manipulating UE's procedural content generation system; rendering synchronized multi-view images of detailed human faces; and running an interactive co-simulation with the MuJoCo physics simulator.

The code and assets in this repository are released under an [MIT License](LICENSE.txt) and a [CC0 License](http://creativecommons.org/publicdomain/zero/1.0) respectively.

## Overview

![hypersim](https://github.com/user-attachments/assets/a1246233-68a9-41d0-ba4b-10710b58b74f)

_SPEAR includes a customizable camera sensor that can render 1920x1080 photorealistic beauty images (left) directly into a user's NumPy array at 56 frames per second -- an order of magnitude faster than existing UE plugins -- while also providing ground truth image modalities that are not available in any existing UE-based simulator. For example, the SPEAR camera sensor can render all of the image modalities in the Hypersim dataset, i.e., depths, surface normals, instance and semantic IDs (right top), and a non-diffuse intrinsic image decomposition (right bottom), as well as material IDs and physically based shading parameters._

![electric_23](https://github.com/user-attachments/assets/bb45e91d-d94e-4912-a433-85f1f107484e)

_We demonstrate the flexibility of SPEAR by using it to programmtically manipulate the `ElectricDreams` sample project from Epic Games. **(a):** We control UE's procedural content generation (PCG) system by translating the main PCG entity in this scene (the rock structure in the center of each image) from left to right. Notice how the rock structure automatically harmonizes with the rest of the scene in a convincing way (e.g., the water adjusts around the rock, logs appear and connect with nearby structures), even when it is being driven by our simple programmatic control. **(b):** We simulate time-of-day changes by controlling the orientation of the scene's sky light._

![mujoco](https://github.com/user-attachments/assets/23629609-47ff-45c5-92f1-0243d0aac858)

_SPEAR can be used in co-simulation applications with external physics simulators. In this application, we interactively control the MuJoCo physics simulator using the default MuJoCo viewer, e.g., by applying a force to the leftmost chair (red arrow). In real-time as the MuJoCo simulation is running, we query the state of the MuJoCo scene (inset images), and we use SPEAR to update the state of a corresponding UE scene (large images)._

![metahumans](https://github.com/user-attachments/assets/9b57b1cb-f89a-4dce-bd82-d7fedf0cdc0c)

_We demonstrate the generality of the SPEAR camera sensor by using it to render synchronized multi-view images of a detailed human character in the `MetaHumans` sample project from Epic Games._

## A Simple SPEAR Program

We demonstrate several fundamental concepts in the SPEAR programming model with a simple example program that spawns a set of coordinate axes in an indoor environment.
In our programming model, graphs of UE work are specified as transactions. In particular, the user specifies a transaction by defining a `begin_frame` context followed by an `end_frame` context. Within each context, the user specifies a graph of UE work simply by implementing it as Python code. Any C++ function or variable that is visible to UE's reflection system (e.g., `SetActorScale3D`, `RootComponent`) can be accessed as though it was a native Python function or attribute. For improved efficiency, we provide an asynchronous variant for each function in SPEAR (e.g., `call_async.K2_GetComponentLocation`) that avoids synchronizing with UE.

```python
import spear

# create an instance representing a UE application; this will either launch a new UE
# application or connect to an existing one depending on how user_config.yaml is defined
config = spear.get_config(user_config_files=["user_config.yaml"])
instance = spear.Instance(config=config)
game = instance.get_game()

# the code in each instance.begin_frame() block executes sequentially at the beginning of
# a single UE frame; as soon as each Python function returns, its side effects on the UE
# application are complete and immediately observable; this programming model enables UE
# work to be executed deterministically in a single frame, even when there are complex data
# dependencies among work items
with instance.begin_frame():

    # spawn object
    bp_axes_uclass = game.unreal_service.load_class(uclass="AActor", name="/SpContent/Blueprints/BP_Axes.BP_Axes_C")
    bp_axes = game.unreal_service.spawn_actor(uclass=bp_axes_uclass, location={"X": -10.0, "Y": 280.0, "Z": 50.0})

    # set scale by calling the AActor::SetActorScale3D C++ function
    bp_axes.SetActorScale3D(NewScale3D={"X": 4.0, "Y": 4.0, "Z": 4.0})

    # get the AActor::RootComponent property on bp_axes as a Python object with its own functions
    root_component = bp_axes.RootComponent.get()

    # use our async API to issue a non-blocking call to the USceneComponent::K2_GetComponentLocation C++ function
    future = root_component.call_async.K2_GetComponentLocation()

# each instance.begin_frame() block must be paired with a corresponding instance.end_frame()
# block; the code in the end_frame() block executes in the same UE frame as the begin_frame()
# block, except it executes at the end of the frame instead of the beginning; this is useful,
# e.g., in embodied AI applications, where it is desirable to provide a control action at the
# beginning of a simulation frame and retrieve an observation at the end of the same frame
with instance.end_frame():

    # get the result from our previous non-blocking call
    location = future.get()
    spear.log("location: ", location)

# close the instance (optional)
instance.close()

spear.log("Done.")
```

![programming_model_12](https://github.com/user-attachments/assets/af0cdb80-0923-4569-9569-5d4a5dbb4065)

_An Unreal scene before (left) and after (right) running the example program above._

## Exposing New Functions and Variables to SPEAR

It is trivial to expose new C++ functions and variables to UE's reflection system, and therefore to SPEAR, simply by adding a `UFUNCTION` or `UPROPERTY` annotation next to the function or variable in any C++ header file.

```cpp
// MyBlueprintFunctionLibrary.h

#pragma once

#include "Containers/UnrealString.h"              // FString
#include "HAL/Platform.h"                         // TEXT
#include "Kismet/BlueprintFunctionLibrary.h"      // UBlueprintFunctionLibrary
#include "MyBlueprintFunctionLibrary.generated.h" // required by Unreal

UCLASS()
class UMyBlueprintFunctionLibrary : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()
public:
    UFUNCTION(BlueprintCallable)
    static FString MyFunction(const FVector& Vec) {
        return FString::Printf(TEXT("Vec is [%.1f %.1f %.1f]"), Vec.X, Vec.Y, Vec.Z);
    }

    UPROPERTY(BlueprintReadWrite)
    uint32 MyProperty = 42;
};
```

Once the `UFUNCTION` and `UPROPERTY` annotations have been added to the C++ header above, `MyFunction` and `MyProperty` can be accessed from Python as follows.

```python
with instance.begin_frame():

    # get the default object for the UMyBlueprintFunctionLibrary class
    my_blueprint_function_library = game.get_unreal_object(uclass="UMyBlueprintFunctionLibrary")

    # returns "Vec is [1.0 2.0 3.0]"
    return_value = my_blueprint_function_library.MyFunction(Vec={"X": 1.0, "Y": 2.0, "Z": 3.0})

    # returns 42
    my_property = my_blueprint_function_library.MyProperty.get()

    # set MyProperty to 43
    my_blueprint_function_library.MyProperty = 43

with instance.end_frame():
    pass
```

## More Documentation

- Our [Getting Started](docs/getting_started.md) tutorial explains how to set up your development environment.
- Our [Running our Example Applications](docs/running_our_example_applications.md) tutorial explains how to run our example applications.
- Our [Importing and Exporting Assets](docs/importing_and_exporting_assets.md) tutorial explains how to import and export assets.

## Citation

If you find SPEAR useful in your research, please cite this repository as follows:

```
@misc{spear,
    author       = {Mike Roberts AND Renhan Wang AND Rushikesh Zawar AND Rachith Prakash
                    AND Quentin Leboutet AND Stephan Richter AND Matthias M{\"u}ller
                    AND German Ros AND Rui Tang AND Stefan Leutenegger AND Yannick
                    Hold-Geoffroy AND Kalyan Sunkavalli AND Vladlen Koltun},
    title        = {{SPEAR}: {A} Simulator for Photorealistic Embodied {AI} Research},
    howpublished = {\url{https://github.com/spear-sim/spear}}
}
```
