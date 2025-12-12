# DSI-SLAM

DSI-SLAM is a saliency-aware fork of ORB-SLAM3 that supports monocular, stereo, RGB-D, and inertial SLAM. The codebase keeps the original map load/save and multi-sensor capabilities while adding optional saliency weighting for feature selection and optimization, plus utilities for exporting and visualizing saliency maps.

## Features
- ORB-SLAM3 pipeline for monocular, stereo, RGB-D, and IMU-assisted tracking with map serialization.
- Saliency-based keypoint weighting (enable with `-DUSE_SALIENCY=ON`; `build.sh` enables it by default).
- Ready-to-run dataset and live camera examples (EuRoC, TUM-VI, KITTI, Intel RealSense D435i/T265) plus ROS nodes.
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
- Monocular (TUM RGB-D format):  
  `./Examples/Monocular/mono_tum Vocabulary/ORBvoc.txt Examples/Monocular/TUM1.yaml path_to_sequence`
- Stereo (EuRoC):  
  `./Examples/Stereo/stereo_euroc Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC.yaml path_to_mav0 path_to_times.txt`
- Stereo-Inertial (TUM-VI/EuRoC): pass left/right image folders, timestamp files, and IMU data as shown in each binary’s usage (`stereo_inertial_tum_vi`, `stereo_inertial_euroc`, etc.).
- RGB-D:  
  `./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/TUM1.yaml path_to_sequence path_to_association_file`
- Live Intel RealSense: run the corresponding `*_realsense_*` executable with a RealSense YAML (requires `librealsense2`).

ROS nodes under `Examples/ROS/ORB_SLAM3` expose the same modes when built inside a ROS workspace. Many binaries accept an optional final argument to write the estimated trajectory to disk.

## Saliency extraction & visualization
1. Build the core library with `USE_SALIENCY=ON`.
2. Point `DSI_SLAM_ROOT_DIR` (alias `ORB_SLAM3_ROOT_DIR` for compatibility) to this repository. Recommended:  
   `cmake -S Examples/SaliencyDraw -B Examples/SaliencyDraw/build -DDSI_SLAM_ROOT_DIR=$(pwd)`
3. Export per-keypoint saliency from an image:  
   `./Examples/SaliencyDraw/build/saliency_extractor path/to/image.png /tmp/saliency.txt`
4. Visualize it as a heatmap/3D surface:  
   `python Examples/SaliencyDraw/draw_saliency.py path/to/image.png /tmp/saliency.txt`

## Evaluation utilities
The `evaluation/` folder contains helper scripts (ATE computation, associations) for comparing estimated trajectories against ground truth (`evaluate_ate_scale.py`, `associate.py`, etc.).

## License
DSI-SLAM builds on ORB-SLAM3 (GPLv3). See `Dependencies.md` for third-party licenses bundled in `Thirdparty/` and linked libraries.
