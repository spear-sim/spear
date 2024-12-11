# Import Stanford Dataset

In this example application, we demonstrate how to import custom objects from the Stanford 3D Scanning Repository.

Before running this example, rename `user_config.yaml.example` to `user_config.yaml` and modify the contents appropriately for your system, as described in our [Getting Started](../../docs/getting_started.md) tutorial.

### Important configuration options

You can control the behavior of this example by setting the following parameters in your `user_config.yaml` file, e.g.,
  - `SPEAR.PAKS_DIR` is a directory containing scene data in the form of PAK files.
  - `SPEAR.PAKS_VERSION_TAG` is the name of a custom version tag for the PAK file we generate in this example.

### Building a PAK file

In order to run this example, you need to build a PAK file containing the Stanford assets as follows.

```console
# download mesh data
python download_dataset.py

# create symlink within the SpearSim project named Stanford
python ../../tools/update_symlinks_for_external_content.py --create --external_content_dir stanford --unreal_project_content_dir Stanford

# import mesh data into the Stanford directory and generate list of assets to include within PAK file
python ../../tools/run_editor_script.py --script /absolute/path/to/spear/examples/import_stanford_dataset/import_dataset.py --unreal_engine_dir path/to/UE_5.5

# build PAK file containing the assets within the Stanford directory
python ../../tools/build_pak.py --paks_dir path/to/spear-paks --version_tag examples --pak_file stanford.pak --cook_dirs_file cook_dirs.csv --include_assets_file include_assets.csv --unreal_engine_dir path/to/UE_5.5

# remove Stanford symlink to restore the SpearSim project to its original state
python ../../tools/update_symlinks_for_external_content.py --remove --external_content_dir stanford --unreal_project_content_dir Stanford
```

### Running the example

Once you have built a PAK file, you can run this example as follows.

```console
python run.py
```

When running this example, the `SPEAR.PAKS_DIR` and `SPEAR.PAKS_VERSION_TAG` parameters in your `user_config.yaml` file need to match the `--paks_dir` and `--version_tag` command-line arguments you specified when building a PAK file.

You should see a game window appear with the imported custom objects placed in the scene.
