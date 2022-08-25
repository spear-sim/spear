#!/bin/bash

#SBATCH -p g24
#SBATCH --gres=gpu:1
#SBATCH -c 14

source /home/qleboute/anaconda3/bin/activate interiorsim-env
python run.py -i 1000 -r 100 -s "Infer" -m "239784069" -c 4 -p "5_envs"
