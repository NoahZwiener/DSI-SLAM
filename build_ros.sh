echo "Building ROS nodes"

#cd Examples_old/ROS/ORB_SLAM3
cd Examples/ROS/ORB_SLAM3
rm -rf build
mkdir build
cd build
# cmake ..
cmake .. -DROS_BUILD_TYPE=Release -DUSE_SALIENCY=ON
make -j8
