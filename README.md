# CSS-LOAM

This repository contains the code of the [Indigo version of loam_velodyne](http://docs.ros.org/indigo/api/loam_velodyne/html/files.html) written by J. Zhang in 2014, but with the feature extraction algorithm changed by the Curvature Scale Space (CSS) algorithm. CSS features only replaces sharp features, planar features extraction is maintained as the original code.

This code was tested with the [Raw KITTI Dataset](http://www.cvlibs.net/datasets/kitti/raw_data.php) and consequently it works with the Velodyne HDL-64. To make it work with other Velodyne LiDARs it is important to consider the number of features CSS extracts, because with few features the trajectory may diverge.

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
```
roslaunch css_loam_velodyne css_loam_velodyne.launch
```

In second terminal play a sample velodyne from the Raw KITTI Dataset, but with a lower rate because CSS is more computationally expensive than the original LOAM's feature extractor.
```
rosbag play -r 0.5 ~/Downloads/00.bag 
```

