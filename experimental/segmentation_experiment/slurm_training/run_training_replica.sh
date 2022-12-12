#!/bin/bash

#SBATCH -p gpu
#SBATCH --gres=gpu:8
#SBATCH -c 112
#SBATCH -w isl-gpu15

cd ../segmentation
source $HOME/anaconda3/bin/activate pytorch-env

torchrun --nproc_per_node=8 train.py --epochs 100 --data-path=$WORK --lr 0.02 --dataset replica --output-dir=$WORK/replica/output_final -b 4 --model deeplabv3_resnet101 --aux-loss --weights-backbone ResNet101_Weights.IMAGENET1K_V2
