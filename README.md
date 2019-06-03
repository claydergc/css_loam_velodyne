# CSS-LOAM

This repository contains the code of the [ROS Indigo version of loam_velodyne](http://docs.ros.org/indigo/api/loam_velodyne/html/files.html) written by J. Zhang in 2014, but with the feature extraction algorithm changed with the Curvature Scale Space (CSS) algorithm. The CSS feature extractor is more robust to the sensor range noise because of its scale space based on Gaussian smoothing. Therefore, it can return better results when working with LiDARS from which range accuracy is low. CSS features only replace sharp features, planar features are maintained as the original code.

This code was tested with the [KITTI Dataset] (http://www.cvlibs.net/datasets/kitti/eval_odometry.php) and consequently it works with the Velodyne HDL-64. To make it work with other Velodyne LiDARs it is important to consider the number of features that the CSS algorithm extracts, because with few features the trajectory may produce a high error.

## Dependency

- [ROS](http://wiki.ros.org/ROS/Installation) (tested with melodic)
- [PCL](https://github.com/PointCloudLibrary/pcl) (tested with version 1.8)

How to build with catkin:

```
$ cd ~/catkin_ws/src/
$ git https://github.com/claydergc/css_loam_velodyne.git
$ cd ~/catkin_ws
$ catkin_make -DCMAKE_BUILD_TYPE=Release 
$ source ~/catkin_ws/devel/setup.bash
```

Running:

The launch file contains a parameter in which you can set the number of the KITTI dataset sequence you want to run.

```
roslaunch css_loam_velodyne css_loam_velodyne.launch
```
