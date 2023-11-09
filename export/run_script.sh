#!/usr/bin/env bash
eval "$(conda shell.bash hook)"
conda activate mujoco_export_pipeline

NUM_WORKERS=35
SCENE_NAME=kujiale_0000_auto

OMP_NUM_THREADS=4 python Python_Export_Pipeline/processing.py -n ${NUM_WORKERS} --scene scenes/${SCENE_NAME}
