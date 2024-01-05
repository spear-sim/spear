#!/usr/bin/env bash
eval "$(conda shell.bash hook)"
conda activate spear-env

NUM_WORKERS=30
SCENE_NAME=apartment_0000

OMP_NUM_THREADS=4 python Python_Export_Pipeline/processing.py -n ${NUM_WORKERS} --scene scenes/${SCENE_NAME}
