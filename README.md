# Realsense Elevation Mapping

This package is based on [ZJU-Robotics-Lab/GEM](https://github.com/ZJU-Robotics-Lab/GEM).

**Modified by Suqin He in 2021**

## Dependencies

This software is built on the ROS, which needs to be [installed](http://wiki.ros.org) first. Additionally, the Globally consistent dense Elevation Mapping depends on following software:

- [Navigation](http://wiki.ros.org/navigation?distro=melodic) (navigation library for costmap_2d required by Grid Map)
- [OpenCV](https://opencv.org/) (required by Grid Map)
- [Grid Map](https://github.com/anybotics/grid_map) (grid map library for mobile robots)
- [Point Cloud Library (PCL)](http://pointclouds.org/) (point cloud processing),
- [Eigen](http://eigen.tuxfamily.org) (linear algebra library, tested on 3.2.9 & 3.3.4; failed on 3.3.9).
- [CUDA](https://developer.nvidia.com/cuda-toolkit-archive) (gpu process)
- cudnn (pay attention to the version: CUDA 10.0 -> cudnn 7.4; CUDA 10.1 -> cudnn 7.6)

## Building

```bash
cd catkin_workspace/src
git clone https://github.com/hsqthu2012/realsense_elevation_mapping.git
cd ../
catkin build
```

## Basic Usage

A script is used to launch different nodes in a specific order to avoid crashing.
Execute the following command to start a simple demo.

### Visual Odometry

The odometry infomation is obtained using D455 visual odometry. The elevation map publishing rate is low due to the limit from visual odometry.

```bash
cd catkin_workspace/src/GEM/elevation_mapping/elevation_mapping_demos/launch
./timed_launch.sh
```

### Odometry using T265 (Recomended)

The odometry infomation is obtained using T265 sensor. The elevation map publishing rate is much higher.

```bash
cd catkin_workspace/src/GEM/elevation_mapping/elevation_mapping_demos/launch
./timed_launch_d455_t265.sh
```

## Important Parameters

In `GEM/elevation_mapping/elevation_mapping_demos/config/robots/realsense_robot.yaml`, the `camera_params_yaml` parameter should be modified to find the `yq_intrinsic.yaml`.

### Change Local Map Size

In `GEM/elevation_mapping/elevation_mapping_demos/config/elevation_maps/realsense_demo_map.yaml`, change the following parameters

- `length_in_x`
- `length_in_y`
- `resolution`

Also, in `realsense_start.launch` or `realsense_d455_t265_start.launch`, change the following parameters to modify point cloud size

- `clip_distance`
- `filter_limit_max`
- `leaf_size`

## Applicantion

Now this program has been tested on PC and Nvidia Jetson TX2.