# Imitation Learning

This example demonstrates how to generate data to train a control policy using imitation learning and finally evaluate the trained policy in SPEAR;

There are three steps;
1. Generate random pairs of initial and goal positions
2. Generate dataset
3. Train a policy 
4. Evaluate the policy

It is here assumed that you already followed the [getting started tutorial](https://github.com/isl-org/spear/blob/main/docs/getting_started.md) and hence have working SPEAR pipeine. 

## Generate random pairs of initial and goal positions

As a starting point, you will need to run the `generate_episodes.py` script to generate a set of training (resp. testing) episodes:

```bash
python generate_episodes.py --num_episodes_per_scene <num_train_poses> --episodes_file <path_to_output_episodes_folder/train_episodes.csv> 
python generate_episodes.py --num_episodes_per_scene <num_test_poses> --episodes_file <path_to_output_episodes_folder/test_episodes.csv> 
```
As mentioned in the [https://github.com/isl-org/OpenBot/tree/master/policy#data-collection](OpenBot public reporitory), the common split between training and test data is 80% - 20%. You should adjust the `<num_train_poses>` and `<num_test_poses>` accordingly.

## Generate dataset

Once a suitable set of start-goal tuples are available and properly divided in a training set and a test set, execute the data generation script by calling

```bash
# activate the spear environment
conda activate spear-env

python generate_dataset.py --iterations 1000 --scene_id "235554..." --create_video --create_plot --rendering_mode baked
```

This will have an openbot agent follow collision-fre trajectories between the differnt start-goal coordinates. 
The generated dataset will have the folowing structrure, to comply with the [OpenBot training pipeline](https://github.com/isl-org/OpenBot/tree/master/policy):

```
dataset
|-- videos
| |-- 0_235114..._compressed.mp4
| |-- ...
|-- train_data
| |-- 235114...
| | |-- episode_0
| | | |-- images
| | | | |-- 0.jpeg
| | | | |-- ...
| | | | |-- 999.jpeg
| | | |-- sensor_data
| | | | |-- ctrlLog.txt
| | | | |-- goalLog.txt
| | | | |-- rgbFrames.txt
| | | | |-- poseData.txt
| | | | |-- waypointData.txt
| | |-- episode_1
| |-- ...
|-- test_data
| |-- 235114...
| | |-- episode_0
| | | |-- images
| | | | |-- 0.jpeg
| | | | |-- ...
| | | | |-- 999.jpeg
| | | |-- sensor_data
| | | | |-- ctrlLog.txt
| | | | |-- goalLog.txt
| | | | |-- rgbFrames.txt
| | | | |-- poseData.txt
| | | | |-- waypointData.txt
| | |-- episode_1
|-- ...
```

## Train a control policy

As the structure of the data collected with SPEAR natively complies with the OpenBot training pipeline, you may simply follow the guidelines of the [poicy training tutorial](https://github.com/isl-org/OpenBot/tree/master/policy#policy-training) in the OpenBot public repository to be able to generate your own `.tflite` policy out of the data you collected.

## Evaluate the control policy

Once the training step is completed, place your `.tflite` file in the `models` folder and execute the following command:

```bash
# activate the spear environment
conda activate spear-env

python run_trained_policy.py --control_policy <policy_name> --iterations 1000 --scene_id "235554..." --create_video --create_plot --rendering_mode baked
```

The result will be stored in the `eval` folder

```
eval
|-- videos
| |-- 0_235114..._compressed.mp4
| |-- ...
|-- run_0_235114.../data
| |-- images
| | |-- 0.jpeg
| | |-- ...
| | |-- 999.jpeg
| |-- results
| | |-- resultLog.txt
| | |-- trajectoryLog.txt
|-- ...
```
