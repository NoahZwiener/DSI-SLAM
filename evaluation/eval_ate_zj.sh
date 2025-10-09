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
python3 time_trans.py
evo_ape euroc /home/nuc/orb_slam3/ORB_SLAM3/dataset/MH_01_easy/mav0/state_groundtruth_estimate0/data.csv\
		 /home/nuc/orb_slam3/ORB_SLAM3/f_dataset-MH01_mono1.txt\
		 -va --plot --plot_mode xyz --save_results a.zip

   
#python3 evaluate_ate_scale.py\
#	/home/nuc/orb_slam3/ORB_SLAM3/evaluation/Ground_truth/EuRoC_imu/MH_GT.txt \
#	/home/nuc/orb_slam3/ORB_SLAM3/f_dataset-MH01_mono.txt \
#	--scale 1\
#	--save plot.png



