# Control the `CitySample` Project

In this example application, we demonstrate how to control the `CitySample` project, which is a freely available sample project from Epic Games.

In a pre-processing step, we need to install the SPEAR plugins in `CitySample` by adding an `AdditionalPluginDirectories` entry to `CitySample.uproject`. We could do this by editing the project directly, but here we use one of our command-line tools for convenience. We must also build the project, which will build the SPEAR plugin binaries and copy them into the project directory.

```console
python ../../tools/install_plugins_in_external_project.py --external-project-dir path/to/CitySample

python ../../tools/run_uat.py --unreal-engine-dir path/to/UE_5.5 --unreal-project-dir path/to/CitySample -build
```

Our next step is to launch the Unreal Editor via the command-line, which is necessary to override some of the project settings in `CitySample`. Alternatively, we could override these settings by editing the files in the `CitySample/Config` directly, but we choose to launch the editor via the command-line so we can avoid modifying the project any more than necessary.

```console
python ../../run_editor.py --unreal-engine-dir path/to/UE_5.5 --unreal-project-dir path/to/CitySample --config-file user_config.yaml
```

Finally, in the Unreal Editor, we open the `Map/Small_City_LVL` map and wait for the map to fully load. Then press play in the editor and wait for the Unreal simulation to launch and warm up. This step can take a long time (e.g., up to 15 minutes). Once the simulation is fully loaded and warmed up, we are ready to launch our SPEAR programs.

```console
# control the default CitySample character
python control_character.py

# control a car
python control_character.py
```

You should see the default CitySample character start walking, or a car start driving, depending on which SPEAR program you run. Additionally, an `images` directory of rendered images will begin to populate.
