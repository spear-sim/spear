# Control the `MetaHumans` Project

In this example application, we demonstrate how to control the `MetaHumans` project, which is a freely available sample project from Epic Games.

In a pre-processing step, we need to install the SPEAR plugins in `MetaHumans` by adding an `AdditionalPluginDirectories` entry to `MetaHumans.uproject`. We could do this by editing the project directly, but here we use one of our command-line tools for convenience. We must also build the project, which will build the SPEAR plugin binaries and copy them into the project directory.

```console
python ../../tools/install_plugins_in_external_project.py --external-project-dir path/to/MetaHumans

python ../../tools/run_uat.py --unreal-engine-dir path/to/UE_5.5 --unreal-project-dir path/to/MetaHumans -build
```

Our next step is to launch the Unreal Editor via the command-line, which is necessary to override some of the project settings in `MetaHumans`. Alternatively, we could override these settings by editing the files in the `MetaHumans/Config` directly, but we choose to launch the editor via the command-line so we can avoid modifying the project any more than necessary.

```console
python ../../run_editor.py --unreal-engine-dir path/to/UE_5.5 --unreal-project-dir path/to/MetaHumans
```

Finally, in the Unreal Editor, we wait for the default map to fully load. Then we press play in the editor and wait for the Unreal simulation to load and warm up. Once the simulation is fully loaded and warmed up, we are ready to launch our SPEAR programs.

```console
python run.py
```

You should see a MetaHumans character start talking. Additionally, an `images` directory of rendered images will begin to populate.
