## INFO ON RUNNING THIS EXPERIMENT

### NOTE: all caps words are params from user_config.yaml, e.g. `IMAGE_SAMPLING_AGENT_CONTROLLER`, `IMAGE_TYPE`

### user_config.yaml
1. modify user_config.yaml with paths specific to your system.
2. In `IMAGE_SAMPLING_AGENT_CONTROLLER`, there are two `ACTION_MODE`;
    - sample_images: This mode is used when we run sample_images.py script. Here, a new set of poses are generated and `IMAGE_TYPE` images are captured.
    - replay_sampled_images: This mode is used when we run replay_sampled_images.py. Here, already generated poses are used to sample images. `IMAGE_TYPE` images are captured.
3. You can leave all params starting with `EXPORT_NAV_DATA*` as False.
4. `DEBUG_POSES_DIR` and `DEBUG_POSES_NUM` can be set to any dummy values if `EXPORT_NAV_DATA*` params are False.

### sample_images.py
1. This script generates new set of poses and captures `IMAGE_TYPE` images and writes it to `output_dir`. 
2. generated poses are written as `poses.txt` to respective scenes in `output_dir`.

### ImageSamplingAgentController.h and ImageSamplingAgentController.cpp
1. These files contain UE side code responsible for the task of image sampling.

### Steps to generate images
1. Build RobotProject standalone executable in Development mode. Make sure to pass `-pak` param to your BuildCookRun command if you are planning to use the UE executable with scene pak files.
2. run sample_images.py to capture images. In `user_config.yaml`, you can set `IMAGE_TYPE` to `rgb` to capture rgb images. segmentation images are disabled as the code is undergoing modification.