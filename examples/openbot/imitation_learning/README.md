# Imitation Learning

This example demonstrates how to generate data to train a control policy using imitation learning and finally evaluate the trained policy in SPEAR;

There are three steps;
1. Generate dataset
2. Train a policy 
3. Evaluate the policy

NOTE: In these steps, ensure that the required pak files are in the executable_content_paks_dir.

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

## Generate dataset

For this scenario, you will need to run

```bash
python generate_dataset.py --iterations 1000 --runs 100 --scene_id "235554..." "235576..." "235114..." 
```

The generated dataset will have the folowing structrure, to comply with the [OpenBot training pipeline](https://github.com/isl-org/OpenBot/tree/master/policy):

```
dataset/uploaded
|-- train_data
| |-- run_235114..._0/data
| | |-- images
| | | |-- 0.jpeg
| | | |-- ...
| | | |-- 999.jpeg
| | |-- sensor_data
| | | |-- ctrlLog.txt
| | | |-- goalLog.txt
| | | |-- rgbFrames.txt
| | | |-- poseData.txt
| | | |-- waypointData.txt
| |-- ...
|-- test_data
| |-- run_235114..._80/data
| | |-- images
| | | |-- 0.jpeg
| | | |-- ...
| | | |-- 999.jpeg
| | |-- sensor_data
| | | |-- ctrlLog.txt
| | | |-- goalLog.txt
| | | |-- rgbFrames.txt
| | | |-- poseData.txt
| | | |-- waypointData.txt
|-- ...
```

## Train a control policy

As the structure of the data collected with SPEAR natively complies with the OpenBot training pipeline, you may simply follow the guidelines of the [poicy training tutorial](https://github.com/isl-org/OpenBot/tree/master/policy#policy-training) in the OpenBot public repository to be able to generate your own `.tflite` policy out of the data you collected.

## Evaluate the control policy

For this scenario, you will need to run

```bash
python run_trained_policy.py --policy <policy_name> --iterations 1000 --runs 100 --scene_id "235554..." "235576..." "235114..." 
```


