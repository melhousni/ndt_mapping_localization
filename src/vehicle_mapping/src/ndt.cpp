/*
Mahdi ELHOUSNI. (melhousni@wpi.edu)
WPI - ECE - 2019.
*/

#include <ros/ros.h>
#include <ros/package.h>

#include <tf/transform_listener.h>

#include <signal.h>
#include <time.h>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <cmath>
#include <istream>
#include <chrono>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl_conversions/pcl_conversions.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <pclomp/ndt_omp.h>

using namespace std; 

static ros::Publisher ndt_pub;
static ros::Publisher guess_pub;
static ros::Publisher odom_pub;

static ros::Subscriber points_sub;
static ros::Subscriber odom_sub;

static int scoreThreshold; //threshold used to dissmiss bad ndt results

static bool init_map	= false; //did we start building the map
static bool init_odom = false; //did we capture the first odom sample
static bool filter = true; //filter top of the car
static bool noise; //do we want to clean to noise
static bool guess; //use or not initial guess for ndt

static double odom_x = 0;
static double odom_y = 0;
static double odom_z = 0;
static double odom_qx = 0;
static double odom_qy = 0;
static double odom_qz = 0;
static double odom_qw = 0;

static double init_odom_x = 0;
static double init_odom_y = 0;
static double init_odom_z = 0;
static double init_odom_qx = 0;
static double init_odom_qy = 0;
static double init_odom_qw = 0;
static double init_odom_qz = 0;

static double pose_x = 0;
static double pose_y = 0;
static double pose_z = 0;
static double pose_qx = 0;
static double pose_qy = 0;
static double pose_qz = 0;
static double pose_qw = 0;
static double pose_roll = 0;
static double pose_pitch = 0;
static double pose_yaw = 0;

static double ndt_pose_x = 0;
static double ndt_pose_y = 0;
static double ndt_pose_z = 0;
static double ndt_pose_roll = 0;
static double ndt_pose_pitch = 0;
static double ndt_pose_yaw = 0;

static double init_pose_x = 0;
static double init_pose_y = 0;
static double init_pose_z = 0;
static double init_pose_roll = 0;
static double init_pose_pitch = 0;
static double init_pose_yaw = 0;

static double epsilon, step, res, leaf, init_leaf, iter;

static std::string imu_topic, odom_topic, lidar_topic, path_to_save_log, path_to_save_pcd, path_to_rosbag;

static tf2::Quaternion ndt_quat;
static tf2::Quaternion init_quat;

static tf::TransformListener* pListener = NULL;

static Eigen::Matrix4f previnit     = Eigen::Matrix4f::Identity();
static Eigen::Matrix4f init_guess   = Eigen::Matrix4f::Identity();
static Eigen::Matrix4f prevTransf   = Eigen::Matrix4f::Identity();
static Eigen::Matrix4f Transf       = Eigen::Matrix4f::Identity();
static Eigen::Matrix4f guess_offset = Eigen::Matrix4f::Identity();
static Eigen::Matrix4f ed_guess     = Eigen::Matrix4f::Identity();

//initial and final pcds
static pcl::PointCloud<pcl::PointXYZI>::Ptr init_cloud   (new pcl::PointCloud<pcl::PointXYZI>);
static pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud  (new pcl::PointCloud<pcl::PointXYZI>);
static pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZI>);
static pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZI>);
static pcl::PointCloud<pcl::PointXYZI>::Ptr final_cloud  (new pcl::PointCloud<pcl::PointXYZI>);

static sensor_msgs::PointCloud2 laserCloudTemp;
static sensor_msgs::PointCloud2::ConstPtr input;

static void apply_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr& tmp){

  pcl::PointIndices::Ptr toremove (new pcl::PointIndices ());
  pcl::ExtractIndices<pcl::PointXYZI> filter (true);

  for (int p = 0; p < (*tmp).size(); p++)
  {
  float r = 2.0f;
  if (std::pow(tmp->points[p].x,2) + std::pow(tmp->points[p].y,2) < std::pow(r,2) && 1 < tmp->points[p].z < 5){
      toremove->indices.push_back(p);
  }
  }

  filter.setInputCloud(tmp);
  filter.setIndices(toremove);
  filter.setNegative(true);
  filter.filter(*tmp);

}
  
//noise removal
static void removeNoise(pcl::PointCloud<pcl::PointXYZI>::Ptr& tmp){

  pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
  sor.setInputCloud (tmp);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (*tmp);

}

void saveMapSig(int sig)
{

  if(noise){
  ROS_INFO("Removing Noise...");
  removeNoise(final_cloud);
  }

  ROS_INFO("Saving PointCloud...");
  pcl::io::savePCDFileASCII (path_to_save_pcd+"mymap.pcd", *final_cloud);
  ros::shutdown();
}

static void odom_callback(nav_msgs::Odometry msg){

  if(!init_odom){

  init_odom_x  = msg.pose.pose.position.x;
  init_odom_y  = msg.pose.pose.position.y;
  init_odom_z  = msg.pose.pose.position.z;
  init_odom_qx = msg.pose.pose.orientation.x;
  init_odom_qy = msg.pose.pose.orientation.y;
  init_odom_qz = msg.pose.pose.orientation.z;
  init_odom_qw = msg.pose.pose.orientation.w;

  init_odom = true;

  }

  odom_x  = msg.pose.pose.position.x;
  odom_y  = msg.pose.pose.position.y;
  odom_z  = msg.pose.pose.position.z;

  odom_qx = msg.pose.pose.orientation.x;
  odom_qy = msg.pose.pose.orientation.y;
  odom_qz = msg.pose.pose.orientation.z;
  odom_qw = msg.pose.pose.orientation.w;

}

static void points_callback(sensor_msgs::PointCloud2 msg){

  static tf::TransformBroadcaster br;

  static tf::Quaternion init_q;
  static tf::Vector3 init_t;

  static tf::Quaternion curr_q;
  static tf::Vector3 curr_t;

  init_q = tf::Quaternion (init_odom_qx, init_odom_qy, init_odom_qz, init_odom_qw);
  init_t = tf::Vector3 (init_odom_x, init_odom_y, init_odom_z);

  curr_q = tf::Quaternion (odom_qx, odom_qy, odom_qz, odom_qw);
  curr_t = tf::Vector3 (odom_x, odom_y, odom_z);

  br.sendTransform(tf::StampedTransform(tf::Transform(init_q, init_t), ros::Time::now(), "odom", "map"));

  //ros::Duration(0.5).sleep(); //uncomment if problems with tf arise

  geometry_msgs::PoseStamped odom_pose;
  odom_pose.header.frame_id = "odom";
  odom_pose.header.stamp = ros::Time();

  odom_pose.pose.position.x = odom_x;
  odom_pose.pose.position.y = odom_y;
  odom_pose.pose.position.z = odom_z;

  odom_pose.pose.orientation.w = odom_qw;
  odom_pose.pose.orientation.x = odom_qx;
  odom_pose.pose.orientation.y = odom_qy;
  odom_pose.pose.orientation.z = odom_qz;

  geometry_msgs::PoseStamped laser_pose;

  try{
    pListener->transformPose("map", odom_pose, laser_pose);
  } catch(tf::TransformException& ex){
    ROS_ERROR("%s", ex.what());
  }

  //current pose according to the /ekf_odom
  pose_x = laser_pose.pose.position.x; 
  pose_y = laser_pose.pose.position.y; 
  pose_z = laser_pose.pose.position.z; 

  pose_qx = laser_pose.pose.orientation.x; 
  pose_qy = laser_pose.pose.orientation.y; 
  pose_qz = laser_pose.pose.orientation.z; 
  pose_qw = laser_pose.pose.orientation.w; 

  tf::Quaternion q;
  q = tf::Quaternion(pose_qx, pose_qy, pose_qz, pose_qw);
  tf::Matrix3x3 m(q);
  m.getRPY(pose_roll, pose_pitch, pose_yaw);

  //initial guess of the transformation between two frames using /ekf_odm 
  Eigen::Translation3f init_translation (pose_x, pose_y, pose_z);
  Eigen::AngleAxisf init_rotation_x(pose_roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf init_rotation_y(pose_pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf init_rotation_z(pose_yaw, Eigen::Vector3f::UnitZ());
  init_guess = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();

  guess_offset = previnit.inverse()*init_guess;
  ed_guess = prevTransf*guess_offset;

  //check if ekf_odom is available
  bool f = (isnan(init_guess.array())).any();

	if (f)
	{
		ROS_INFO("Waiting for odom...");
		return;
	}

  static pcl::PointCloud<pcl::PointXYZI>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(msg, *tmp);

  //check if pcd is available
  if (tmp->size()==0){
    ROS_INFO("Waiting for PointCloud...");
    return;
  }

  //apply filter to remove rays that bounce of the car
  if(filter){
  	apply_filter(tmp);
  }

  if (init_map == false){

  ROS_INFO("Loading init pcd !\n");

  if(guess){
  	pcl::transformPointCloud (*tmp, *init_cloud, ed_guess);
  }
  else{
  	*init_cloud = *tmp;
  }

  //initialize the final and input pcd with the initial one
  *final_cloud = *init_cloud;
  *input_cloud = *init_cloud;
  init_map = true;

  pcl::toROSMsg(*input_cloud, laserCloudTemp);
  laserCloudTemp.header.frame_id = "map";
  ndt_pub.publish(laserCloudTemp);

  }

  //subsampling filter
  pcl::ApproximateVoxelGrid<pcl::PointXYZI> approximate_voxel_filter;
  approximate_voxel_filter.setLeafSize (leaf, leaf, leaf);

  *target_cloud = *tmp;

  //filtering to reduce computing time
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZI>);
  approximate_voxel_filter.setInputCloud (target_cloud);
  approximate_voxel_filter.filter (*filtered_cloud);

  //filtering to reduce computing time
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_2 (new pcl::PointCloud<pcl::PointXYZI>);
  approximate_voxel_filter.setInputCloud (input_cloud);
  approximate_voxel_filter.filter (*filtered_cloud_2);

  pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr ndt_omp(new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());

  double n = omp_get_max_threads();
  ndt_omp->setNumThreads(n);

  // ndt_omp->setNeighborhoodSearchMethod(pclomp::KDTREE);
  ndt_omp->setNeighborhoodSearchMethod(pclomp::DIRECT7);

  ndt_omp->setTransformationEpsilon (epsilon);
  ndt_omp->setStepSize (step);
  ndt_omp->setResolution (res);
  ndt_omp->setMaximumIterations (iter);

  ndt_omp->setInputSource (filtered_cloud);
  ndt_omp->setInputTarget (input_cloud);
  
  if(guess){
    ndt_omp->align (*output_cloud,ed_guess);
  }
  else{
    ndt_omp->align (*output_cloud);
  }
  
  if (ndt_omp->getFitnessScore() < scoreThreshold){

    //std::cout << "NDT has converged." << std::endl;
    //std::cout << "NDT score is " << ndt_omp->getFitnessScore() << "\n" << std::endl;

    Transf =  ndt_omp->getFinalTransformation();

    // Transforming unfiltered, input cloud using found transform.
    pcl::transformPointCloud (*target_cloud, *output_cloud, Transf);

    *final_cloud = *final_cloud + *output_cloud;
    *input_cloud = *final_cloud;

    prevTransf = Transf;

    pcl::toROSMsg(*input_cloud, laserCloudTemp);
    laserCloudTemp.header.frame_id = "map";
    ndt_pub.publish(laserCloudTemp);
    
    }

  else{

    std::cout << "***!***\n" << std::endl;
    std::cout << "Skipped this pcd because of bad ndt score." << std::endl;
    std::cout << "NDT score is " << ndt_omp->getFitnessScore () << "\n" << std::endl;
    std::cout << "***!***\n" << std::endl;

    Transf =  init_guess;

    pcl::transformPointCloud (*target_cloud, *output_cloud, Transf);

    *final_cloud = *final_cloud + *output_cloud;
    *input_cloud = *final_cloud;

    prevTransf = Transf;

  }

  previnit = init_guess;

  tf::Matrix3x3 TransfMat;

  TransfMat.setValue(static_cast<double>(Transf(0, 0)), static_cast<double>(Transf(0, 1)),
                     static_cast<double>(Transf(0, 2)), static_cast<double>(Transf(1, 0)),
                     static_cast<double>(Transf(1, 1)), static_cast<double>(Transf(1, 2)),
                     static_cast<double>(Transf(2, 0)), static_cast<double>(Transf(2, 1)),
	                   static_cast<double>(Transf(2, 2)));

  ndt_pose_x = Transf(0, 3);
  ndt_pose_y = Transf(1, 3);
  ndt_pose_z = Transf(2, 3);

  TransfMat.getRPY(ndt_pose_roll, ndt_pose_pitch, ndt_pose_yaw, 1);
  ndt_quat.setRPY(ndt_pose_roll, ndt_pose_pitch, ndt_pose_yaw);

  nav_msgs::Odometry ndt_odom;
  ndt_odom.header.stamp = laserCloudTemp.header.stamp;
  ndt_odom.header.frame_id = laserCloudTemp.header.frame_id;

  ndt_odom.pose.pose.position.x = ndt_pose_x;
  ndt_odom.pose.pose.position.y = ndt_pose_y;
  ndt_odom.pose.pose.position.z = ndt_pose_z;

  ndt_odom.pose.pose.orientation.x = ndt_quat.x();
  ndt_odom.pose.pose.orientation.y = ndt_quat.y();
  ndt_odom.pose.pose.orientation.z = ndt_quat.z();
  ndt_odom.pose.pose.orientation.w = ndt_quat.w();

  odom_pub.publish(ndt_odom);

  tf::Matrix3x3 InitMat;

  InitMat.setValue(static_cast<double>(ed_guess(0, 0)), static_cast<double>(ed_guess(0, 1)),
                   static_cast<double>(ed_guess(0, 2)), static_cast<double>(ed_guess(1, 0)),
                   static_cast<double>(ed_guess(1, 1)), static_cast<double>(ed_guess(1, 2)),
                   static_cast<double>(ed_guess(2, 0)), static_cast<double>(ed_guess(2, 1)),
                   static_cast<double>(ed_guess(2, 2)));

  init_pose_x = ed_guess(0, 3);
  init_pose_y = ed_guess(1, 3);
  init_pose_z = ed_guess(2, 3);

  InitMat.getRPY(init_pose_roll, init_pose_pitch, init_pose_yaw, 1);
  init_quat.setRPY(init_pose_roll, init_pose_pitch, init_pose_yaw);

  nav_msgs::Odometry init_odom;
  init_odom.header.stamp = laserCloudTemp.header.stamp;
  init_odom.header.frame_id = laserCloudTemp.header.frame_id;

  init_odom.pose.pose.position.x = init_pose_x;
  init_odom.pose.pose.position.y = init_pose_y;
  init_odom.pose.pose.position.z = init_pose_z;

  init_odom.pose.pose.orientation.x = init_quat.x();
  init_odom.pose.pose.orientation.y = init_quat.y();
  init_odom.pose.pose.orientation.z = init_quat.z();
  init_odom.pose.pose.orientation.w = init_quat.w();

  guess_pub.publish(init_odom);

  //check if ndt transform is available
  f = (isnan(Transf.array())).any();

	if (f)
	{
		ROS_INFO("Waiting for the ndt transform...");
		return;
	}

}

int main(int argc, char **argv){

  ros::init(argc, argv, "ndt_mapping");

  ros::NodeHandle nh("~");

  signal(SIGINT, saveMapSig);

  pListener = new (tf::TransformListener);

  nh.getParam("odom_topic", odom_topic);
  nh.getParam("lidar_topic", lidar_topic);

  nh.getParam("path_to_save_log", path_to_save_log);
  nh.getParam("path_to_save_pcd", path_to_save_pcd);
  nh.getParam("path_to_rosbag"  , path_to_rosbag);

  nh.getParam("noise" , noise);
  nh.getParam("guess" , guess);
  nh.getParam("filter", filter);

  nh.getParam("scoreThreshold", scoreThreshold);
  nh.getParam("epsilon", epsilon);
  nh.getParam("step", step);
  nh.getParam("iter", iter);
  nh.getParam("leaf", leaf);
  nh.getParam("res", res);

  odom_pub  = nh.advertise<nav_msgs::Odometry>("/ndt_odom", 1000);
  guess_pub = nh.advertise<nav_msgs::Odometry>("/guess_odom", 1000);
  ndt_pub 	= nh.advertise<sensor_msgs::PointCloud2>("/ndt_map", 1000);

  points_sub = nh.subscribe(lidar_topic, 10, points_callback); 
  odom_sub   = nh.subscribe(odom_topic, 10, odom_callback); 

  ros::spin();

  return 0;

}
