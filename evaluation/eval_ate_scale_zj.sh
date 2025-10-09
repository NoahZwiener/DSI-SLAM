#!/bin/bash
#pathDatasetEuroc='/home/nuc/orb_slam3/ORB_SLAM3/dataset' #Example, it is necesary to change it by the dataset path

#------------------------------------
# Monocular Examples
#echo "Launching MH01 with Monocular sensor"
#./Examples/Monocular/mono_euroc ./Vocabulary/ORBvoc.txt ./Examples/Monocular/EuRoC.yaml ./dataset/MH_01_easy ./Examples/Monocular/#EuRoC_TimeStamps/MH01.txt dataset-MH01_mono
#/home/nuc/orb_slam3/ORB_SLAM3/evaluation
#python2 zj.py \
#     /home/nuc/orb_slam3/ORB_SLAM3/evaluation/Ground_truth/EuRoC_imu/MH_GT.txt \
#     /home/nuc/orb_slam3/ORB_SLAM3/f_dataset-MH01_mono.txt \
#     --scale 1\
#     --plot 1\
    
python2 evaluate_ate_scale.py\
	/home/nuc/orb_slam3/ORB_SLAM3/evaluation/Ground_truth/EuRoC_imu/MH_GT.txt \
	/home/nuc/orb_slam3/ORB_SLAM3/f_dataset-MH01_mono.txt \
	--scale 1\
	--plot 1\
	--save plot.png



