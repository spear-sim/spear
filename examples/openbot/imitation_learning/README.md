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
python generate_dataset.py --iterations 1000 --runs 100 --scenes "<scene_id_1>" "<scene_id_2>" "<scene_id_n>" -create_video
```


## Train a control policy


## Evaluate the control policy

For this scenario, you will need to run

```bash
python run_trained_policy.py --policy <path/to/the/policy> --iterations 1000 --runs 100 --scenes "<scene_id_1>" "<scene_id_2>" "<scene_id_n>" -create_video
```


