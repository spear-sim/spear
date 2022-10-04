#!/bin/bash

#SBATCH -p gpu
#SBATCH --gres=gpu:8
#SBATCH -c 112
#SBATCH -w isl-gpu13

cd ../segmentation
source $HOME/anaconda3/bin/activate pytorch-env

torchrun --nproc_per_node=8 train.py --epochs 100 --data-path=/export/share/datasets/nyu_depth_v2/labeled --lr 0.05 --dataset nyuv2 --output-dir=$WORK/nyuv2/output_final -b 4 --model deeplabv3_resnet101 --aux-loss --weights-backbone ResNet101_Weights.IMAGENET1K_V2
 