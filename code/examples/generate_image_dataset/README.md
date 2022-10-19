# Generate Image Dataset

This example demonstrates how to generate image datasets with InteriorSim scenes;

There are two steps;
1. Generate random poses
2. Generate images

## Generate random poses

For this scenario, you will need to run 

```bash
python generate_poses.py --pak_file <path_to_pak_file> --executable_content_dir <path_to_content_directory_of_executable> --num_images <number_of_images_to_generate> --poses_file <path_to_output_poses_file>
```
NOTE: If `<path_to_executable_dir>/WindowsNoEditor/<project_name>.exe` is your executable, executable_content_dir is `<path_to_executable_dir>/WindowsNoEditor/<project_name>/Content`.

This file creates a csv file that contains random generated poses.

## Generate images

For this scenario, you will need to run

```bash
python generate_images.py --pak_file <path_to_pak_file> --executable_content_dir <path_to_content_directory_of_executable> --poses_file <path_to_poses_file> --output_dir <path_to_output_dir>
```
NOTE: If `<path_to_executable_dir>/WindowsNoEditor/<project_name>.exe` is your executable, executable_content_dir is `<path_to_executable_dir>/WindowsNoEditor/<project_name>/Content`.
