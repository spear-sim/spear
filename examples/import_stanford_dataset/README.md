# Import Stanford Dataset

In this example application, we demonstrate how to import custom objects from the Stanford 3D Scanning Repository. In order to run this example, you need to build a PAK file containing the Stanford assets as follows.

```console
# download mesh data
python download_dataset.py

# create symlink within the SpearSim project named Stanford
python ../../tools/update_symlink_for_external_content.py --create --external-content-dir stanford --unreal-project-content-dir Stanford

# import mesh data into the Stanford directory and generate list of assets to include within PAK file
python ../../tools/run_editor_script.py --unreal-engine-dir path/to/UE_5.5 --script /absolute/path/to/spear/examples/import_stanford_dataset/import_dataset.py

# build PAK file containing the Stanford assets
python ../../tools/build_pak.py --unreal-engine-dir path/to/UE_5.5 --pak-file pak/stanford.pak --cook-dirs-file csv/cook_dirs.csv --include-assets-file csv/include_assets.csv

# remove Stanford symlink to restore the SpearSim project to its original state
python ../../tools/update_symlink_for_external_content.py --remove --external-content-dir stanford --unreal-project-content-dir Stanford
```

Once you have built a PAK file, you can run this example as follows.

```console
python run.py
```

You should see a game window appear with the imported custom objects placed in the scene.
