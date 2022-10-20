# Generate Image Dataset

This example demonstrates how to generate image datasets with InteriorSim scenes;

There are two steps;
1. Generate random poses
2. Generate images

## Generate random poses

For this scenario, you will need to run 

```bash
python generate_poses.py --paks_dir <path_to_directory_containing_pak_files_downloaded_using_scene_manager> --executable_content_dir <path_to_content_directory_of_executable> --num_poses_per_scene <required_number> --poses_file <path_to_output_poses_file>
```
NOTE: If `<path_to_executable_dir>/WindowsNoEditor/<project_name>.exe` is your executable, executable_content_dir is `<path_to_executable_dir>/WindowsNoEditor/<project_name>/Content`.

This file creates a csv file that contains generated random poses for all scenes in `pak_dir`.

## Generate images

For this scenario, you will need to run

```bash
python generate_images.py --paks_dir <path_to_pak_file> --executable_content_dir <path_to_content_directory_of_executable> --poses_file <path_to_poses_file> --output_dir <path_to_output_dir>
```
NOTE: If `<path_to_executable_dir>/WindowsNoEditor/<project_name>.exe` is your executable, executable_content_dir is `<path_to_executable_dir>/WindowsNoEditor/<project_name>/Content`.

NOTE: The python files assume a certain directory structure for `paks_dir`. If your `paks_dir` is present in `<path_to_paks_dir>`, an example of the directory structure for a single scene's pak file is `<path_to_paks_dir>/235114808/paks/Windows/235114808/235114808_Windows.pak`. If `paks_dir` contain multiple files, it will be like below;

```
<path_to_paks_dir>
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
