#!/bin/bash

#python3 evaluate_ate_scale.py  --scale 1 --plot PLOT /home/nuc/fig/V1_03/Monocular/ablation_FF0.1_BA0.0_NF_1000_run_5.txt /home/nuc/orb_slam3/ORBSLAM/ORBSLAM/evaluation/Ground_truth/EuRoC_left_cam/V103_GT_1E9.txt

python3 evaluate_ate_scale.py  --scale 1 --plot PLOT /home/nuc/orb_slam3/ORBSLAM/ORBSLAM/evaluation/Ground_truth/EuRoC_left_cam/V103_GT_1E9.txt /home/nuc/fig/V1_03/Monocular/ablation_FF0.1_BA0.0_NF_1000_run_5.txt  --verbose2

#evo_ape tum /home/nuc/fig/V1_03/Monocular/ablation_FF0.1_BA0.0_NF_1000_run_5.txt /home/nuc/orb_slam3/ORBSLAM/ORBSLAM/evaluation/Ground_truth/EuRoC_left_cam/V103_GT_1E9.txt  -as -va -p

#evo_ape tum /home/nuc/orb_slam3/ORBSLAM/ORBSLAM/evaluation/Ground_truth/EuRoC_left_cam/V103_GT_1E9.txt /home/nuc/fig/V1_03/Monocular/ablation_FF0.1_BA0.0_NF_1000_run_10.txt -as -va -p
