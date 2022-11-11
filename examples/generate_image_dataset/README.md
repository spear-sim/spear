# Generate Image Dataset

This example demonstrates how to generate image datasets for SPEAR scenes;

There are two steps;
1. Generate random poses
2. Generate images

## Generate random poses

For this scenario, you will need to run 

```bash
python generate_poses.py --executable_content_paks_dir <path_to_directory_containing_pak_files_downloaded_using_scene_manager> --num_poses_per_scene <required_number> --poses_file <path_to_output_poses_file>
```
NOTE: If `<path_to_executable_dir>/WindowsNoEditor/<project_name>.exe` is your executable, `executable_content_paks_dir` is `<path_to_executable_dir>/WindowsNoEditor/<project_name>/Content/Paks`.

This file creates a csv file that contains generated random poses for all scenes in `executable_content_paks_dir`.

## Generate images

For this scenario, you will need to run

```bash
python generate_images.py --poses_file <path_to_input_poses_file> --output_dir <path_to_output_dir_to_store_images>
```

NOTE: In both these steps, ensure that the required pak files are in the executable_content_paks_dir.

```
<executable_content_paks_dir>
|-- 235114...
| |-- paks
| | |-- Windows
| | | |-- 235114...
| | | | |-- 235114..._Windows.pak
|-- 237001...
| |-- paks
| | |-- Windows
| | | |-- 2357001...
| | | | |-- 2357001..._Windows.pak
|-- ...
```
