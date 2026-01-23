# Control the `ElectricDreamsEnv` Project

In this example application, we demonstrate how to control the `ElectricDreamsEnv` project, which is a freely available sample project from Epic Games.

In a pre-processing step, we need to install the SPEAR plugins in `ElectricDreamsEnv` by adding an `AdditionalPluginDirectories` entry to `ElectricDreamsEnv.uproject`. We could do this by editing the project directly, but here we use one of our command-line tools for convenience. We must also build the project, which will build the SPEAR plugin binaries and copy them into the project directory.

```console
python ../../tools/install_plugins_in_external_project.py --external-project-dir path/to/ElectricDreamsEnv

python ../../tools/run_build.py --unreal-engine-dir path/to/UE_5.5 --unreal-project-dir path/to/ElectricDreamsEnv --build-target ElectricDreamsSampleEditor -build
```

Next, we open the Unreal Editor, open the `Levels/PCG/ElectricDreams_PCGCloseRange` map, and wait for the map to fully load. Then we press play in the editor and wait for the Unreal simulation to load and warm up. Once the simulation is fully loaded and warmed up, we are ready to control the project via SPEAR.

```console
# control light
python control_light.py

# control PCG assembly
python control_pcg_assembly.py
```

You should see the lighting change, or a PCG assembly move, depending on which SPEAR program you run. Additionally, an `images` directory of rendered images will begin to populate.
