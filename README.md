# SPEAR: A Simulator for Photorealistic Embodied AI Research

![control_samples](https://github.com/user-attachments/assets/ea378a71-8ea6-45f2-86b0-ef18a05fb219)

_Figure: The SPEAR Python library can call any C++ function, and can access any C++ variable, on any game entity, and any game subsystem, in any Unreal Engine application, provided the function or variable has been exposed to Unreal's visual scripting system, i.e., Blueprints. We demonstrate this unique capability by controlling 4 embodied agents, each with a distinct action space, in several sample projects that are freely available from Epic Games (i.e., a person from `CitySample` in the top left, a car from `CitySample` in the top right, a flying robot from `StackOBot` in the bottom right, and a person with parkour skills from `GameAnimationSample` in the bottom right). SPEAR is capable of controlling the agents in these projects, and performing custom high-speed rendering, without requiring any code or data modifications to the projects themselves, aside from adding a single-line declaration to each project definition file. No other changes are required to enable comprehensive programmatic control of complex in-the-wild Unreal projects from Python via SPEAR._

![hypersim](https://github.com/user-attachments/assets/88210f82-7436-407c-906d-cf4c2bff74de)

_Figure: SPEAR includes a customizable camera entity that can render a superset of the ground truth modalities available in the Hypersim dataset, and can render 1080p photorealistic beauty images (top row) directly into a user's NumPy array at 55 frames per second. The SPEAR camera can also render fine-grained 24-bit entity IDs that can be used for both material segmentation and object segmentation tasks (middle row), and a non-Lambertian intrinsic image decomposition consisting of diffuse reflectance, diffuse illumination, and a non-diffuse residual term (bottom row)._

![time_of_day](https://github.com/user-attachments/assets/9370572f-df34-4973-86bc-9ebcf5d54bec)

_Figure: By calling existing Unreal functions that are available through SPEAR's general-purpose Python API, it is straightforward to control the lighting in any scene. Here, we programmatically control the lighting to simulate time-of-day changes in the `ElectricDreams` sample project that is freely available from Epic Games._

![pcg](https://github.com/user-attachments/assets/f545b1e5-c344-42e7-8ab9-8d6de36842b2)

_Figure: It is also straightforward to use SPEAR to programmatically interact with Unreal's Procedural Content Generation (PCG) system for dynamically generating scene geometry. Here, we horizontally translate a PCG entity (the big rock structure in the approximate center of each image) across the scene from left to right in the `ElectricDreams` sample project via a short Python program. Notice how the main rock structure harmonizes with the rest of the scene in a convincing way (e.g., the water and ground adjusts around the main rock, logs appear and connect with nearby structures), even when it is being driven by simple programmatic control._

![metahumans](https://github.com/user-attachments/assets/9b57b1cb-f89a-4dce-bd82-d7fedf0cdc0c)

_Figure: SPEAR includes a customizable multi-view camera entity that can render a scene from multiple views at exactly the same time in the Unreal simulation. Here, we render synchronized images from the `Metahumans` sample project available from Epic Games._

## Abstract

Interactive simulators have become powerful tools for training embodied agents and generating synthetic visual data, but existing photorealistic simulators suffer from limited generality, expressiveness, and performance. We address these limitations by introducing _SPEAR: A Simulator for Photorealistic Embodied AI Research_.

At its core, SPEAR is a Python library that can connect to, and programmatically control, any Unreal Engine (UE) application via a set of modular UE C++ plugins. In contrast to existing Python libraries for accessing the Unreal Engine, SPEAR offers several novel features that are useful in embodied AI, robotics, and computer vision applications.

- SPEAR can call any C++ function, and can access any C++ variable, on any game entity, and any game subsystem, provided the function or variable has been exposed to Unreal's visual scripting system, referred to in the UE ecosystem as the _Blueprint System_ or simply as _Blueprints_. There are over 13K functions and over 44K variables that are already exposed to Blueprints (and are therefore accessible in SPEAR) in the UE codebase, and it is trivial to expose new functions and variables by adding a single-line annotation next to a function or variable in a C++ header (see example below).
- SPEAR provides efficient zero-copy NumPy interoperability, e.g., SPEAR can copy 1080p photorealistic beauty images (see figure above) from the GPU directly into a user's NumPy array at 55 frames per second without requiring any intermediate data copying.
- SPEAR includes a camera entity that can render a superset of the ground truth modalities available in the Hypersim dataset (see figure above), including fine-grained 24-bit entity IDs that can be used for both material segmentation and object segmentation tasks, and a non-Lambertian intrinsic image decomposition consisting of diffuse reflectance, diffuse illumination, and a non-diffuse residual term.
- SPEAR can programmatically control standalone shipping games that are already running, live simulations running inside the Unreal Editor, Unreal's path tracer, and the Unreal Editor itself, all through a unified Pythonic interface.
- SPEAR gives users precise control over how their UE work is executed across UE frames, while also allowing users to execute complex work graphs (i.e., with arbitrary data dependencies among work items) deterministically within a single frame.
- SPEAR can be used in any Python environment, even on a remote machine, and does not need to be invoked from inside the Unreal Editor.

The code and assets in this repository are released under an [MIT License](LICENSE.txt) and a [CC0 License](http://creativecommons.org/publicdomain/zero/1.0) respectively.

## A Simple SPEAR Program

We demonstrate the SPEAR programming model with a simple example program that spawns an object, prints some debug information about the object, and adjusts its size. We can see the effect of running this program in the figure below.

```python
import pprint
import spear

# create an instance representing a UE application; this will either launch a new UE
# application or connect to an existing one depending on how user_config.yaml is defined
config = spear.get_config(user_config_files=["user_config.yaml"])
spear.configure_system(config=config)
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

    # print all available functions and properties and other debug info for bp_axes
    bp_axes.print_debug_info()

    # get all object properties for bp_axes as nested Python dictionaries
    spear.log("bp_axes.get_properties():")
    pprint.pprint(bp_axes.get_properties())

    # get scale by calling the AActor::GetActorScale3D C++ function
    scale = bp_axes.GetActorScale3D()
    spear.log("scale: ", scale)

    # set scale by calling the AActor::SetActorScale3D C++ function
    bp_axes.SetActorScale3D(NewScale3D={"X": 4.0, "Y": 4.0, "Z": 4.0})

    # get the RootComponent property on bp_axes as a Python object with its own functions
    root_component = bp_axes.RootComponent.get()

    # get all object properties for root_component as nested Python dictionaries
    spear.log("root_component.get_properties():")
    pprint.pprint(root_component.get_properties())

# each instance.begin_frame() block must be paired with a corresponding instance.end_frame()
# block; the code in the end_frame() block executes in the same UE frame as the begin_frame()
# block, except it executes at the end of the frame instead of the beginning; this is useful,
# e.g., in embodied AI applications, where it is desirable to provide a control action at the
# beginning of a simulation frame and retrieve an observation at the end of the same frame
with instance.end_frame():
    pass

# close the instance (optional)
instance.close()

spear.log("Done.")
```

![before_after](https://github.com/user-attachments/assets/d1a6b42e-45d1-460a-82f0-86dd3a679554)

_Figure: An Unreal scene before (left) and after (right) running the example program above, demonstrating that the `BP_Axes` entity is spawned as expected after running the program._

## Exposing New Functions and Variables to SPEAR

SPEAR can call any function and access any variable that is exposed to Unreal's visual scripting system, i.e., Blueprints. New functions and variables can be exposed to Blueprints (and therefore to SPEAR) simply by adding `UFUNCTION(...)` and `UPROPERTY(...)` annotations to a C++ header in any Unreal project or plugin as follows. No additional registration steps or code boilerplate is required.

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
    static FString MyFunction(const FString& UserString) { return FString::Printf(TEXT("UserString is %s"), *UserString); }

    UPROPERTY(BlueprintReadWrite)
    uint32 MyProperty = 42;
};
```

After the `UFUNCTION(...)` and `UPROPERTY(...)` annotations have been added to the C++ header above, `MyFunction` and `MyProperty` can be accessed in Python using SPEAR as follows.

```python
with instance.begin_frame():

    # get the default object for the UMyBlueprintFunctionLibrary class
    my_blueprint_function_library = game.get_unreal_object(uclass="UMyBlueprintFunctionLibrary")

    # returns "UserString is Hello World"
    return_value = my_blueprint_function_library.MyFunction(UserString="Hello World")

    # returns 42
    my_property = my_blueprint_function_library.MyProperty.get()

    # set MyProperty to 43
    my_blueprint_function_library.MyProperty = 43

with instance.end_frame():
    pass
```

This extensibility model makes it trivial for a user to expose custom C++ functionality to SPEAR without needing to modify SPEAR code, since the user's custom C++ functionality can be defined in their own projects and plugins.

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
