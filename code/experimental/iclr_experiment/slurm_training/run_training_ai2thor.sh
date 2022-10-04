#!/bin/bash

#SBATCH -p g24
#SBATCH --gres=gpu:8
#SBATCH -c 80
#SBATCH -w isl-gpu3

cd ../segmentation
source $HOME/anaconda3/bin/activate pytorch-env

torchrun --nproc_per_node=8 train.py --epochs 100 --data-path=$WORK --lr 0.02 --dataset ai2thor --output-dir=$WORK/ai2thor/output_final2 -b 4 --model deeplabv3_resnet101 --aux-loss
