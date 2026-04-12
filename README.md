# DSI-SLAM

DSI-SLAM is a saliency-aware fork of ORB-SLAM3 that supports monocular, stereo, RGB-D, and inertial SLAM. The codebase keeps the original map load/save and multi-sensor capabilities while adding optional saliency weighting for feature selection and optimization, plus utilities for exporting and visualizing saliency maps.

---
# 1. Prerequisites
We have tested the library in **Ubuntu 20.04** with Intel NUC ().

Following are thirdparty required with version:
```
C++11;
Pangolin 0.6;
Opencv 4.2(default in ROS Noetic);
Eigen 3.3;
ceres-solver-2.0.0;
Python 3.8;
ROS Noetic;
Third-party sources (DBoW2, g2o, Sophus) are included and built during setup.
```

# 2. Building DSI-SLAM3 library and examples

Clone the repository:
```
git clone https://github.com/NoahZwiener/DSI-SLAM.git DSI-SLAM
```

## Quick Script:
- `build.sh` : build the *Thirdparty* libraries and *DSI-SLAM*. 
- `build_slam.sh` : build only *DSI-SLAM*.
- `build_fast.sh` : Incremental build only *DSI-SLAM*.
- `build_ros.sh` : the ROS nodes inside Examples/ROS/ORB_SLAM3(Must build after build.sh or build_slam.sh)

 Execute:
```
cd DSI_SLAM
chmod +x build.sh
./build.sh
```

This will create **libORB_SLAM3.so**  at *lib* folder and the executables in *Examples* folder.

## Manual build
```
tar -xf Vocabulary/ORBvoc.txt.tar.gz -C Vocabulary
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DUSE_SALIENCY=ON
make -j$(nproc)
```

Set `-DUSE_SALIENCY=OFF` to disable saliency weighting. Use ./build_fast.sh when you already have a configured build/ directory and just need to re-run make. Run ./build_ros.sh after the core library is built to compile the ROS nodes inside Examples/ROS/ORB_SLAM3.


# 3. Running examples
Executables live under `Examples/<mode>/` and expect the vocabulary file, a settings YAML, and dataset paths:
- Monocular (KITTI format):  
  `./Examples/Monocular/mono_kitti Vocabulary/ORBvoc.txt Examples/Monocular/KITTI*.yaml path_to_sequence`
- Stereo (EuRoC format):  
  `./Examples/Stereo/stereo_euroc Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC.yaml dataset_path_to_mav0 ./Examples/Stereo/EuRoC_TimeStamps/path_to_times.txt` 

---
# 4. ROS Examples

## Building the nodes for mono, mono-inertial, stereo, stereo-inertial and RGB-D

1. Add the path including *Examples/ROS/ORB_SLAM3* to the ROS_PACKAGE_PATH environment variable. Open .bashrc file:
  ```
  gedit ~/.bashrc
  ```
and add at the end the following line. Replace PATH by the folder where you cloned ORB_SLAM3:

  ```
  export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:PATH/ORB_SLAM3/Examples/ROS
  ```
  
2. Execute `build_ros.sh` script after `build.sh`:

  ```
  chmod +x build_ros.sh
  ./build_ros.sh
  ```
  
## Running Monocular Node
For a monocular input from topic `/camera/image_raw` run node ORB_SLAM3/Mono. You will need to provide the vocabulary file and a settings file. See the monocular examples above.

  ```
  roscore

  rosrun ORB_SLAM3 Mono PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE
  e.g.
  rosrun ORB_SLAM3 Mono Vocabulary/ORBvoc.txt Examples/Monocular/EuRoC.yaml

  rosbag play dataset_path.bag
  ```

## Running Monocular-Inertial Node
For a monocular input from topic `/camera/image_raw` and an inertial input from topic `/imu`, run node ORB_SLAM3/Mono_Inertial. Setting the optional third argument to true will apply CLAHE equalization to images (Mainly for TUM-VI dataset).

  ```
  rosrun ORB_SLAM3 Mono_Inertial PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE
  ```

## Running Stereo Node
For a stereo input from topic `/camera/left/image_raw` and `/camera/right/image_raw` run node ORB_SLAM3/Stereo. You will need to provide the vocabulary file and a settings file. For Pinhole camera model, if you **provide rectification matrices** (see Examples/Stereo/EuRoC.yaml example), the node will recitify the images online, **otherwise images must be pre-rectified**. For FishEye camera model, rectification is not required since system works with original images:

  ```
  rosrun ORB_SLAM3 Stereo PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE 
  ```

## Running Stereo-Inertial Node
For a stereo input from topics `/camera/left/image_raw` and `/camera/right/image_raw`, and an inertial input from topic `/imu`, run node ORB_SLAM3/Stereo_Inertial. You will need to provide the vocabulary file and a settings file, including rectification matrices if required in a similar way to Stereo case:

  ```
  rosrun ORB_SLAM3 Stereo_Inertial PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE 	
  ```
  
## Running RGB_D Node
For an RGB-D input from topics `/camera/rgb/image_raw` and `/camera/depth_registered/image_raw`, run node ORB_SLAM3/RGBD. You will need to provide the vocabulary file and a settings file. See the RGB-D example above.

  ```
  rosrun ORB_SLAM3 RGBD PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE
  ```
---

# 4. Running DSI-SLAM with your camera

Directory `Examples` contains several demo programs and calibration files to run DSI-SLAM in all sensor configurations with Intel Realsense cameras T265 and D435i. 

- First, IntelRealsense should be installed(For example in catkin_ws)! Then launch camera:

```
cd ~/catkin_ws
source devel/setup.bash
roslaunch realsense2_camera rs_camera.launch
```
- Launch DSI-SLAM:
```
cd DSI-SLAM
rosrun ORB_SLAM3 Monocular Vocabulary/ORBvoc.txt Examples/Monocular/RealSense_D435i.yaml false
```
- If you want to use custom dataset, next steps are requested:
```
1. Record your dataset:
rosbag record -a
2. Use dataset:
rosbag play custom.bag 
```

# 5. Using Realtime Pose

DSI-SLAM support output camera pose in realtime. You can echo `rostopic list` to visit different topics. Then subscribe `/Sensor-Type/CameraPose` to use pose for downstream works.

Pose information can be obtain in : `rostopic echo /Mono/CameraPose` 

# 6. Evaluation
The `evaluation/` folder contains helper scripts (ATE computation, associations) for comparing estimated trajectories against ground truth (`evaluate_ate_scale.py`, `associate.py`, etc.).

The another way to evaluating is `evo`, which is a Python package for evaluating SLAM trajectories. 

Install it via pip:

```bash
pip install evo --upgrade --no-binary evo
```

## Example usage
1. Align the estimated trajectory with ground truth:
  ```bash
  evo_ape tum groundtruth.txt trajectory.txt --align
  ```
  Replace `tum` with the dataset format (e.g., `kitti`, `euroc`) and provide the respective ground truth and estimated trajectory files.

2. Plot the trajectories:
  ```bash
  evo_traj tum groundtruth.txt trajectory.txt --align --plot
  ```

3. Compute Absolute Trajectory Error (ATE):
  ```bash
  evo_ape tum groundtruth.txt trajectory.txt --align --save_results results.zip
  ```

4. Compare multiple trajectories:
  ```bash
  evo_res results1.zip results2.zip
  ```

Refer to the [evo documentation](https://github.com/MichaelGrupp/evo) for advanced options and detailed explanations.


# 7. Saliency extraction & visualization
1. Build the core library with `USE_SALIENCY=ON`.
2. Point `DSI_SLAM_ROOT_DIR` to this repository. Recommended:  
   `cmake -S Examples/SaliencyDraw -B Examples/SaliencyDraw/build -DDSI_SLAM_ROOT_DIR=$(pwd)`
3. Export per-keypoint saliency from an image,out put file is `/tmp/saliency.txt`:  
   `./Examples/SaliencyDraw/build/saliency_extractor path/to/image.png /tmp/saliency.txt`
4. Visualize it as a heatmap/3D surface:  
   `python3 Examples/SaliencyDraw/draw_saliency.py path/to/image.png /tmp/saliency.txt`

# 8. Running time analysis
A flag in `include\Config.h` activates time measurements. It is necessary to uncomment the line `#define REGISTER_TIMES` to obtain the time stats of one execution which is shown at the terminal and stored in a text file(`ExecTimeMean.txt`).


# 9. Calibration
You can find a tutorial for visual-inertial calibration and a detailed description of the contents of valid configuration files at  `Calibration_Tutorial.pdf`
