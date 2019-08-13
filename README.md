# ndt_mapping_localization
C++ Implementation of the NDT mapping and localization algorithm for ADV on ROS.

![NDT Results](/image.png)

Dependecies :

- **robot_localization** (https://github.com/cra-ros-pkg/robot_localization)
- **PCL** (https://github.com/PointCloudLibrary/pcl)
- **OpenMP** (https://github.com/OpenMP)
- **ndt_omp** (https://github.com/koide3/ndt_omp)

Two packages available in this implementation :

- **vehicle_mapping** : Pointcloud registration using the 3D NDT algorithm assisted by an EKF.
- **vehicle_localization** : 6-DoF Localization using the 3D NDT algorithm assisted by an EKF.

How to use :

- **vehicle_mapping** : 
  1/ Set up the topic names for GPS and IMU in the vehicle_state/launch/ekf and vehicle_state/param.ekf.yaml. For more information of how to use the robot_localization package, please see : http://docs.ros.org/lunar/api/robot_localization/html/index.html
