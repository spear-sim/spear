# SPEAR: A Simulator for Photorealistic Embodied AI Research

_Figure: SPEAR is a Python library that can connect to, and programmatically control, any Unreal Engine (UE) application via a modular plugin architecture. SPEAR exposes over 14K unique UE functions, representing an order-of-magnitude increase in programmable functionality over existing simulators. We demonstrate the flexibility of SPEAR by using it to control 6 distinct embodied agents (each with a different action space) across several Epic Games sample projects: a person and a car from `CitySample` (top); a flying robot from `StackOBot` (bottom far left); multiple agents in a resource collecting game called `CropoutSample}` (bottom center left); as well as a person with parkour skills and a quadruped robot from `GameAnimationSample` (bottom right)._

## Abstract

Interactive simulators have become powerful tools for training embodied agents and generating synthetic visual data, but existing photorealistic simulators suffer from limited generality, programmability, and rendering speed. We address these limitations by introducing _SPEAR: A Simulator for Photorealistic Embodied AI Research_. At its core, SPEAR is a Python library that can connect to, and programmatically control, any Unreal Engine (UE) application via a modular plugin architecture. SPEAR exposes over 14K unique UE functions to Python, representing an order-of-magnitude increase in programmable functionality over existing UE-based simulators. Additionally, a single SPEAR instance can render 1920x1080 photorealistic beauty images directly into a user's NumPy array at 56 frames per second -- an order of magnitude faster than existing UE plugins -- while also providing ground truth image modalities that are not available in any existing UE-based simulator (e.g., a non-diffuse intrinsic image decomposition, material IDs, and physically based shading parameters). Finally, SPEAR introduces an expressive high-level programming model that enables users to specify complex graphs of UE work with arbitrary data dependencies among work items, and to execute these graphs deterministically within a single UE frame. We demonstrate the utility of SPEAR through a diverse collection of example applications: controlling multiple embodied agents with distinct action spaces (e.g., humans, cars, and robots) across several in-the-wild UE projects; rendering photorealistic city-scale environments; manipulating UE's procedural content generation system; rendering synchronized multi-view images of detailed human faces; and running an interactive co-simulation with the MuJoCo physics simulator.

The code and assets in this repository are released under an [MIT License](LICENSE.txt) and a [CC0 License](http://creativecommons.org/publicdomain/zero/1.0) respectively.

## Overview

![hypersim](https://github.com/user-attachments/assets/88210f82-7436-407c-906d-cf4c2bff74de)

_Figure: SPEAR includes a customizable camera entity that can render a superset of the ground truth modalities available in the Hypersim dataset, and can render 1080p photorealistic beauty images (top row) directly into a user's NumPy array at 55 frames per second. The SPEAR camera can also render fine-grained 24-bit entity IDs that can be used for both material segmentation and object segmentation tasks (middle row), and a non-Lambertian intrinsic image decomposition consisting of diffuse reflectance, diffuse illumination, and a non-diffuse residual term (bottom row)._

![time_of_day](https://github.com/user-attachments/assets/9370572f-df34-4973-86bc-9ebcf5d54bec)

_Figure: By calling existing Unreal Engine C++ functions that are available through SPEAR's general-purpose Python API, it is straightforward to control the lighting in any scene. Here, we programmatically control the lighting to simulate time-of-day changes in Epic Games' `ElectricDreams` sample project._

![pcg](https://github.com/user-attachments/assets/f545b1e5-c344-42e7-8ab9-8d6de36842b2)

_Figure: It is also straightforward to use SPEAR to programmatically interact with Unreal's Procedural Content Generation (PCG) system for dynamically generating scene geometry. Here, we horizontally translate a PCG entity (the big rock structure in the approximate center of each image) across the scene from left to right in the `ElectricDreams` sample project via a simple Python program. Notice how the main rock structure harmonizes with the rest of the scene in a convincing way (e.g., the water and ground adjusts around the main rock, logs appear and connect with nearby structures), even when it is being driven by simple programmatic control._

![metahumans](https://github.com/user-attachments/assets/9b57b1cb-f89a-4dce-bd82-d7fedf0cdc0c)

_Figure: SPEAR includes a customizable multi-view camera entity that can render a scene from multiple views at the same time in an Unreal simulation. Here, we render synchronized images in Epic Games' `Metahumans` sample project._

## A Simple SPEAR Program

We demonstrate the SPEAR programming model with a simple example program that spawns an object, prints some debug information about the object, and adjusts its size. We can see the effect of running this program in the figure below.

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

![before_after](https://github.com/user-attachments/assets/d1a6b42e-45d1-460a-82f0-86dd3a679554)

_Figure: An Unreal scene before (left) and after (right) running the example program above, demonstrating that the `BP_Axes` entity is spawned as expected after running the program._

## Exposing New Functions and Variables to SPEAR

SPEAR can call any function and access any variable that is exposed to Unreal's visual scripting system, i.e., Blueprints. New C++ functions and variables can be exposed to Blueprints (and therefore to SPEAR) simply by adding `UFUNCTION(...)` and `UPROPERTY(...)` annotations to a C++ header in any Unreal project or plugin as follows. No additional registration steps or code boilerplate is required.

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

After the `UFUNCTION(...)` and `UPROPERTY(...)` annotations have been added to the C++ header above, `MyFunction` and `MyProperty` can be accessed from Python via SPEAR as follows.

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

This extensibility model makes it trivial for a user to expose custom C++ functionality to SPEAR without needing to modify SPEAR code, since the user's custom functionality can be defined in their own projects and plugins.

## More Documentation

- Our [Getting Started](docs/getting_started.md) tutorial explains how to set up your development environment.
- Our [Running our Example Applications](docs/running_our_example_applications.md) tutorial explains how to run our example applications.
- Our [Importing and Exporting Assets](docs/importing_and_exporting_assets.md) tutorial explains how to import and export assets.

## Citation

If you find SPEAR useful in your research, please cite this repository as follows:

```
@misc{spear,
    author       = {Mike Roberts AND Rachith Prakash AND Renhan Wang AND Quentin Leboutet AND
                    Stephan R. Richter AND Stefan Leutenegger AND Rui Tang AND Matthias
                    M{\"u}ller AND German Ros AND Vladlen Koltun},
    title        = {{SPEAR}: {A} Simulator for Photorealistic Embodied AI Research},
    howpublished = {\url{http://github.com/spear-sim/spear}}
}
```
