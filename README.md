![teaser](https://github.com/user-attachments/assets/88210f82-7436-407c-906d-cf4c2bff74de)

# SPEAR: A Simulator for Photorealistic Embodied AI Research

Interactive simulators are becoming powerful tools for training embodied agents and generating synthetic visual data, but existing simulators suffer from limited generality, expressiveness, and performance. We address these limitations by introducing _SPEAR: A Simulator for Photorealistic Embodied AI Research_.

At its core, SPEAR is a Python library that can connect to, and programmatically control, any Unreal Engine (UE) application via a set of modular UE C++ plugins. In contrast to existing Python libraries for accessing the Unreal Engine, SPEAR offers several novel features that are useful in embodied AI, robotics, and computer vision training pipelines.

1. SPEAR can call any C++ function, and can access any C++ variable, on any game entity, and any engine subsystem, in the entire Unreal Engine codebase, provided the function or variable has been exposed to Unreal's visual scripting language. There are over 13K functions such functions and over 44K such variables in the UE codebase, and new functions and variables can be exposed by adding a single-line annotation next to a function or variable declaration in a C++ header.
2. SPEAR provides fast NumPy interoperability, e.g., SPEAR can copy rendered images directly from the GPU into a user's NumPy array at 55 fps at 1080p resolution without requiring any intermediate data copying.
3. SPEAR includes a camera sensor entity that can render a strict superset of the data modalities available in the Hypersim (see image above), including fine-grained 24-bit entity IDs that can be used for material segmentation and object segmentation tasks.
4. SPEAR can control standalone games, live simulations running inside the Unreal Editor, and the Unreal Editor itself, all through a clean, unified, and Pythonic interface.
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

with instance.begin_frame():

    # spawn object
    bp_axes_uclass = game.unreal_service.load_class(uclass="AActor", name="/SpContent/Blueprints/BP_Axes.BP_Axes_C")
    bp_axes = game.unreal_service.spawn_actor(uclass=bp_axes_uclass, location={"X": -10.0, "Y": 280.0, "Z": 50.0})

    # print all available functions and properties and other debug info for bp_axes
    bp_axes.print_debug_info()

    # get all object properties as nested Python dictionaries
    spear.log("bp_axes.get_properties():")
    pprint.pprint(bp_axes.get_properties())

    # get scale
    scale = bp_axes.GetActorScale3D()
    spear.log("scale: ", scale)

    # set scale and get it again to verify that it has been updated
    bp_axes.SetActorScale3D(NewScale3D={"X": 4.0, "Y": 4.0, "Z": 4.0})
    scale = bp_axes.GetActorScale3D()
    spear.log("scale: ", scale)

    # get the RootComponent property on bp_axes as a Python object
    root_component = bp_axes.RootComponent.get()

    # print all available functions and properties and other debug info for root_component
    root_component.print_debug_info()

    spear.log("root_component.get_properties():")
    pprint.pprint(root_component.get_properties())

with instance.end_frame():
    pass

spear.log("Done.")
```

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

## Documentation

- Our [Getting Started](docs/getting_started.md) tutorial explains how to set up your development environment.
- Our [Running our Example Applications](docs/running_our_example_applications.md) tutorial explains how to run our example applications.
- Our [Importing and Exporting Assets](docs/importing_and_exporting_assets.md) tutorial explains how to import and export assets.
