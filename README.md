# DSI-SLAM

DSI-SLAM is a saliency-aware fork of ORB-SLAM3 that supports monocular, stereo, RGB-D, and inertial SLAM. The codebase keeps the original map load/save and multi-sensor capabilities while adding optional saliency weighting for feature selection and optimization, plus utilities for exporting and visualizing saliency maps.

## Features
- ORB-SLAM3 pipeline for monocular, stereo, RGB-D, and IMU-assisted tracking with map serialization.
- Saliency-based keypoint weighting (enable with `-DUSE_SALIENCY=ON`; `build.sh` enables it by default).
- Ready-to-run dataset and live camera examples (EuRoC, KITTI, Intel RealSense D435i) plus ROS nodes.
- Helper scripts for trajectory evaluation and saliency visualization.

## Requirements
- Linux with a C++14 compiler and CMake ≥3.10.
- OpenCV ≥4.4, Eigen3 ≥3.1, Pangolin, and Boost serialization.
- Optional: `librealsense2` for RealSense examples; ROS (catkin/colcon) for the nodes under `Examples/ROS`; Python 3 with `numpy`, `matplotlib`, and `scipy` for saliency plotting.
- ORB vocabulary: extract `Vocabulary/ORBvoc.txt.tar.gz` before running examples.
- Third-party sources (DBoW2, g2o, Sophus) are included and built during setup.

## Build

### Quick script
```bash
./build.sh   # builds Thirdparty libs, unpacks Vocabulary/ORBvoc.txt, and configures with -DUSE_SALIENCY=ON
```

### Manual build
```bash
tar -xf Vocabulary/ORBvoc.txt.tar.gz -C Vocabulary
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DUSE_SALIENCY=ON
make -j$(nproc)
```
Set `-DUSE_SALIENCY=OFF` to disable saliency weighting.
Use `./build_fast.sh` when you already have a configured `build/` directory and just need to re-run `make`.
Run `./build_ros.sh` after the core library is built to compile the ROS nodes inside `Examples/ROS/ORB_SLAM3`.

## Running examples
Executables live under `Examples/<mode>/` and expect the vocabulary file, a settings YAML, and dataset paths:
- Monocular (KITTI format):  
  `./Examples/Monocular/mono_kitti Vocabulary/ORBvoc.txt Examples/Monocular/KITTI*.yaml path_to_sequence`
- Stereo (EuRoC):  
  `./Examples/Stereo/stereo_euroc Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC.yaml path_to_mav0 path_to_times.txt`
- Stereo-Inertial (TUM-VI/EuRoC): pass left/right image folders, timestamp files, and IMU data as shown in each binary’s usage (`stereo_inertial_tum_vi`, `stereo_inertial_euroc`, etc.).
- Live Intel RealSense: run the corresponding `*_realsense_*` executable with a RealSense YAML (requires `librealsense2`).

---

## ROS examples
ROS nodes under `Examples/ROS/ORB_SLAM3` expose the same modes when built inside a ROS workspace. Many binaries accept an optional final argument to write the estimated trajectory to disk.

### build 
  ```bash
  cd DSI-SLAM
  ./build.sh
  ./build_ros.sh
  ```

### Running Examples
- Monocular:  

  ```bash
  roscore
  rosrun ORB_SLAM3 Stereo_Inertial Vocabulary/ORBvoc.txt Examples/Stereo-Inertial/RealSense_D435i.yaml false
  rosbag play MY01.bag
  ```

Replace `Mono` with `Stereo`, `RGBD`, or `Stereo_Inertial` for other modes. Update the YAML file path as needed.

- To save the estimated trajectory, append the output file path as the final argument:  
  ```bash
  rosrun ORB_SLAM3 Mono Vocabulary/ORBvoc.txt Examples/ROS/ORB_SLAM3/Monocular.yaml /tmp/trajectory.txt
  ```

---

## Saliency extraction & visualization
1. Build the core library with `USE_SALIENCY=ON`.
2. Point `DSI_SLAM_ROOT_DIR` to this repository. Recommended:  
   `cmake -S Examples/SaliencyDraw -B Examples/SaliencyDraw/build -DDSI_SLAM_ROOT_DIR=$(pwd)`
3. Export per-keypoint saliency from an image:  
   `./Examples/SaliencyDraw/build/saliency_extractor path/to/image.png /tmp/saliency.txt`
4. Visualize it as a heatmap/3D surface:  
   `python3 Examples/SaliencyDraw/draw_saliency.py path/to/image.png /tmp/saliency.txt`

## Evaluation utilities
The `evaluation/` folder contains helper scripts (ATE computation, associations) for comparing estimated trajectories against ground truth (`evaluate_ate_scale.py`, `associate.py`, etc.).

The another way to evaluating is `evo`, which is a Python package for evaluating SLAM trajectories. 

Install it via pip:

```bash
pip install evo --upgrade --no-binary evo
```

### Example usage
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


## License
DSI-SLAM builds on ORB-SLAM3 (GPLv3). See `Dependencies.md` for third-party licenses bundled in `Thirdparty/` and linked libraries.
