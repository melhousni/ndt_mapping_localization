# ndt_mapping_localization
C++ Implementation of the NDT mapping and localization algorithm for ADV on ROS.

![NDT Results](/image.png)

Dependecies :

- **ROS** (http://wiki.ros.org/ROS/Installation)
- **robot_localization** (https://github.com/cra-ros-pkg/robot_localization)
- **PCL** (https://github.com/PointCloudLibrary/pcl)
- **OpenMP** (https://github.com/OpenMP)
- **ndt_omp** (https://github.com/koide3/ndt_omp)

Compile :

cd ~/catkin_ws/src
git clone https://github.com/melhousni/ndt_mapping_localization
cd ..
catkin_make

Two packages available in this implementation :

- **vehicle_mapping** : Pointcloud registration using the 3D NDT algorithm assisted by an EKF.
- **vehicle_localization** : 6-DoF Localization using the 3D NDT algorithm assisted by an EKF.

How to use :

- **vehicle_mapping** : 
  1/ Set up the topic names for GPS and IMU data in the vehicle_state/launch/ekf.launch and vehicle_state/param.ekf.yaml. For more information of how to use the robot_localization package, please see : http://docs.ros.org/lunar/api/robot_localization/html/index.html
  2/ Set up the topic names for Pointcloud and EKF Odom data in vehicle_mapping/launch/ndt_mapping.launch
  3/ Set up the path to save the final pointcloud in vehicle_mapping/launch/ndt_mapping.launch
  4/ Use roslaunch vehicle_mapping ndt_mapping.launch to launch the mapping package
