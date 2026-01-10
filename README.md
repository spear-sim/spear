![teaser](https://github.com/user-attachments/assets/88210f82-7436-407c-906d-cf4c2bff74de)

# SPEAR: A Simulator for Photorealistic Embodied AI Research

Interactive simulators have become powerful tools for training embodied agents and generating synthetic visual data, but existing simulators suffer from limited generality, expressiveness, and performance. We address these limitations by introducing _SPEAR: A Simulator for Photorealistic Embodied AI Research_.

At its core, SPEAR is a Python library that can connect to, and programmatically control, any Unreal Engine (UE) application via a set of modular UE C++ plugins. In contrast to existing Python libraries for accessing the Unreal Engine, SPEAR offers several novel features that are useful in embodied AI, robotics, and computer vision applications.

1. SPEAR can call any C++ function, and can access any C++ variable, on any game entity, and any game subsystem, provided the function or variable has been exposed to Unreal's visual scripting language. There are over 13K functions and over 44K variables that are already exposed in this way in the UE codebase, and it is trivial to expose new functions and variables by adding a single-line annotation next to a function or variable in a C++ header (see example below).
2. SPEAR provides fast NumPy interoperability, e.g., SPEAR can copy rendered images from the GPU directly into a user's NumPy array at 55 fps at 1080p resolution without requiring any intermediate data copying.
3. SPEAR includes a camera entity that can render a strict superset of the data modalities available in the Hypersim dataset (see image above), including fine-grained 24-bit entity IDs that can be used for material segmentation and object segmentation tasks.
4. SPEAR can programmatically control standalone shipping games, live simulations running inside the Unreal Editor, Unreal's path tracer, and the Unreal Editor itself, all through a clean, unified, and Pythonic interface.
5. The SPEAR Python library can be used in any Python environment, even on a remote machine, and does not need to be invoked from inside the Unreal Editor.

The code and assets in this repository are released under an [MIT License](LICENSE.txt) and a [CC0 License](http://creativecommons.org/publicdomain/zero/1.0) respectively.

## A Simple SPEAR Program

```python
import pprint
import spear

# create instance
config = spear.get_config(user_config_files=["user_config.yaml"])
spear.configure_system(config=config)
instance = spear.Instance(config=config)
game = instance.get_game()

# the code in an instance.begin_frame() block executes at the start of a single UE frame
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

    # print all available functions and properties and other debug info for root_component
    root_component.print_debug_info()

    # get all object properties for root_component as nested Python dictionaries
    spear.log("root_component.get_properties():")
    pprint.pprint(root_component.get_properties())

# the code in an instance.end_frame() block executes at the end of the same UE frame as above
with instance.end_frame():
    pass

spear.log("Done.")
```

## Exposing Functions and Variables

SPEAR can call any function and access any variable that is exposed to Unreal's visual scripting system. Exposing new functions and variables can be achieved simply by adding `UFUNCTION()` and `UPROPERTY()` annotations to a C++ header in an Unreal project or plugin as follows. No other registration steps or code boilerplate is required.

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
    UFUNCTION()
    static FString MyFunction(const FString& UserString) { return FString::Printf(TEXT("UserString is %s"), *UserString); }
    UPROPERTY()
    static uint32 MyProperty = 42;
};
```

This function and variable can be accessed through SPEAR as follows.

```python
with instance.begin_frame():

    # get the default object for the UMyBlueprintFunctionLibrary class, which can can be used to call
    # static C++ functions and access static C++ variables on the UMyBlueprintFunctionLibrary class
    my_blueprint_function_library = game.get_unreal_object(uclass="UMyBlueprintFunctionLibrary")

    # returns "UserString is Hello world"
    return_string = my_blueprint_function_library.MyFunction(UserString="Hello World")

    # returns 42
    my_int = my_blueprint_function_library.MyProperty.get()

    # MyInt is now 42
    my_blueprint_function_library.MyProperty = 0

with instance.end_frame():
    pass
```

## Further Documentation

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
