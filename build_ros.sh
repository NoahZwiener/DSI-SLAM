echo "Building ROS nodes"

#cd Examples_old/ROS/ORB_SLAM3
cd Examples/ROS/ORB_SLAM3
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release -DUSE_SALIENCY=ON -DUSE_SALIENCY_EKF=ON
make -j8
