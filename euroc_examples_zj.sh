#!/bin/bash
pathDatasetEuroc='/home/nuc/orb_slam3/ORB_SLAM3/dataset' #Example, it is necesary to change it by the dataset path

#------------------------------------
# Monocular Examples
echo "Launching MH01 with Monocular sensor"
./Examples/Monocular/mono_euroc ./Vocabulary/ORBvoc.txt ./Examples/Monocular/EuRoC.yaml ./dataset/MH_01_easy ./Examples/Monocular/EuRoC_TimeStamps/MH01.txt dataset-MH01_mono
