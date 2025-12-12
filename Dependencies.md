## List of Known Dependencies
### DSI-SLAM

This document lists all the pieces of code included in DSI-SLAM and linked libraries that are not the property of the authors of DSI-SLAM. The project is based on the ORB-SLAM3 framework.

---

### Code in **src** and **include** folders

* **ORBextractor.cc**  
  This is a modified version of `orb.cpp` from the OpenCV library. The original code is BSD licensed.

* **PnPsolver.h, PnPsolver.cc**  
  These are modified versions of `epnp.h` and `epnp.cc` by Vincent Lepetit.  
  The original code can be found in the following BSD-licensed computer vision libraries:  
  [OpenCV](https://github.com/Itseez/opencv/blob/master/modules/calib3d/src/epnp.cpp) and [OpenGV](https://github.com/laurentkneip/opengv/blob/master/src/absolute_pose/modules/Epnp.cpp).

* **MLPnPsolver.h, MLPnPsolver.cc**  
  These are modified versions of MLPnP by Steffen Urban. The original code is available [here](https://github.com/urbste/opengv).  
  The original code is BSD licensed.

* **ORBmatcher::DescriptorDistance**  
  Located in `ORBmatcher.cc`.  
  The code is from: http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel.  
  This code is in the public domain.

---

### Code in **Thirdparty** folder

* **DBoW2**  
  This is a modified version of [DBoW2](https://github.com/dorian3d/DBoW2) and [DLib](https://github.com/dorian3d/DLib).  
  All files are BSD licensed.

* **g2o**  
  This is a modified version of [g2o](https://github.com/RainerKuemmerle/g2o). All files included are BSD licensed.

* **Sophus**  
  This is a modified version of [Sophus](https://github.com/strasdat/Sophus). [MIT license](https://en.wikipedia.org/wiki/MIT_License).

---

### Library dependencies 

* **Pangolin (visualization and user interface)**.  
  [MIT license](https://en.wikipedia.org/wiki/MIT_License).

* **OpenCV**.  
  BSD license.

* **Eigen3**.  
  For versions greater than 3.1.1 is MPL2, earlier versions are LGPLv3.

* **ROS (Optional, only if you build Examples/ROS)**.  
  BSD license. In the manifest.xml the only declared package dependencies are roscpp, tf, sensor_msgs, image_transport, cv_bridge, which are all BSD licensed.

---

### Notes

- This project is based on ORB-SLAM3, and all code not explicitly mentioned in this document follows the original ORB-SLAM3 license.
- If you encounter any issues related to licensing while using this project, please feel free to contact the developers.