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
