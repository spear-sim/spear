# Control the `CropoutSample` Project

In this example application, we demonstrate how to control the `CropoutSample` project, which is a freely available sample project from Epic Games.

In a pre-processing step, we need to install the SPEAR plugins in `CropoutSample` by adding an `AdditionalPluginDirectories` entry to `CropoutSampleProject.uproject`. We could do this by editing the `uproject` file directly, but here we use one of our command-line tools for convenience. We must also build the project, which will build the SPEAR plugin binaries and copy them into the project directory.

```console
python ../../tools/install_plugins_in_external_project.py --external-project-dir path/to/CropoutSampleProject

python ../../tools/run_uat.py --unreal-engine-dir path/to/UE_5.5 --unreal-project-dir path/to/CropoutSampleProject -build
```

Next, we open the Unreal Editor and wait for the default map to fully load. Then we press play in the editor and wait for the Unreal simulation to load and warm up. Once the simulation is fully loaded and warmed up, we are ready to control the project via SPEAR.

```console
python run.py
```

You should see the amount of resources increase in the game's UI, a partially constructed building should appear, some additional characters should spawn, and the characters should start building. Additionally, an `images` directory of rendered images will begin to populate.
