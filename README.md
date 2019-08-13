# ndt_mapping_localization
C++ Implementation of the NDT mapping and localization algorithm for ADV on ROS.

![NDT Results](/image.png)

Dependecies :

- [robot_localization] (https://github.com/cra-ros-pkg/robot_localization)
- [PCL] (https://github.com/PointCloudLibrary/pcl)
- [OpenMP] (https://github.com/OpenMP)
- [ndt_omp] (https://github.com/koide3/ndt_omp)

Two packages available in this implementation :

vehicle_mapping : Pointcloud registration using the 3D NDT algorithm assisted by an EKF.
vehicle_localization : 6-DoF Localization using the 3D NDT algorithm assisted by an EKF.
