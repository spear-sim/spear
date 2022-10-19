# GENERATE IMAGE DATASET

This example demonstrates two ways of generating images;
1. generating images from random poses
2. generating images from pre-defined poses

## Generate images from random poses

For this scenario, you will need to run 

```bash
python generate_images.py --pak_file <path_to_pak_file> --executable_content_dir <path_to_executable\'s_content_directory> --output_dir <path_to_output_dir>
```
NOTE: If <path_to_executable_dir>/WindowsNoEditor/<project_name>.exe is your executable, executable_content_dir is '<path_to_executable_dir>/WindowsNoEditor/<project_name>/Content'."

This file creates a `poses.txt` file in the `output_dir` and contains all poses corresponding to the generated images.

## Generate images from pre-defined poses

For this scenario, you will need to run

```bash
python teleport.py --pak_file <path_to_pak_file> --executable_content_dir <path_to_executable\'s_content_directory> --poses_file <path_to_poses_file> --output_dir <path_to_output_dir>
```
NOTE: If <path_to_executable_dir>/WindowsNoEditor/<project_name>.exe is your executable, executable_content_dir is '<path_to_executable_dir>/WindowsNoEditor/<project_name>/Content'."
