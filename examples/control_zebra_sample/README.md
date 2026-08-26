# Control the `ZebraSample` Project

In this example application, we demonstrate how to control the `ZebraSample` project, which is a freely available sample project from Epic Games.

In a pre-processing step, we need to install the SPEAR plugins into the `ZebraSample` project. We must also build the project, which will build the SPEAR plugin binaries and copy them into the project directory.

```console
python ../../tools/install_plugins_in_external_project.py --external-project-dir path/to/ZebraSample

python ../../tools/run_uat.py --unreal-engine-dir path/to/UE_5.8 --unreal-project-dir path/to/ZebraSample -build
```

Our next step is to symlink a small amount of additional content into the `ZebraSample` project.

```console
python ../../tools/update_symlink_for_external_content.py --unreal-project-dir path/to/ZebraSample --create --external-content-dir data --unreal-project-content-dir SPEAR
```

Our next step is to open `ZebraSample.uproject` in the Unreal Editor, open the `/Game/SPEAR/LV_Demo_CRC` map, and wait for the map to fully load. Then we press play in the editor and wait for the Unreal simulation to load and warm up. Once the simulation is fully loaded and warmed up, we are ready to control the character via SPEAR.

```console
python run.py
```

You should see the character's face and body move in a periodic pattern.
