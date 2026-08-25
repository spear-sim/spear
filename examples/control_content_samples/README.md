# Control to `ContentSamples` Project

In this example application, we demonstrate how to control the `ContentSamples` project, which is a freely available sample project from Epic Games.

## Animation Basics

In a pre-processing step, we need to install the SPEAR plugins into the `ContentSamples` project. We must also build the project, which will build the SPEAR plugin binaries and copy them into the project directory.

```console
python ../../tools/install_plugins_in_external_project.py --external-project-dir path/to/ContentSamples

python ../../tools/run_uat.py --unreal-engine-dir path/to/UE_5.5 --unreal-project-dir path/to/ContentSamples -build
```

Next, we open the Unreal Editor and wait for the default map to fully load. Then we press play in the editor and wait for the Unreal simulation to load and warm up. Once the simulation is fully loaded and warmed up, we are ready to control the project via SPEAR.

```console
python animation_basics.py
```

You should see a character spawn (if it hasn't already spawned from a previous run) near the default player starting location, and the character's feet should speed up and slow down in a periodic pattern.
