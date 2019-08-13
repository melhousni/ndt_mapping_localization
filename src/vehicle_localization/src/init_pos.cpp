/*
Mahdi ELHOUSNI. (melhousni@wpi.edu)
WPI - ECE - 2019.
*/

#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <gps_common/conversions.h>

#include <pclomp/ndt_omp.h>
#include <pcl/registration/ndt.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl_conversions/pcl_conversions.h>

using namespace std;

static ros::Publisher map_pub;
static ros::Publisher fscan_pub;
static ros::Publisher tmp_pub;
static ros::Publisher initguess_pub;
static ros::Publisher ndtodom_pub;
static ros::Publisher leftlane_pub;
static ros::Publisher leftestlane_pub;
static ros::Publisher rightlane_pub;
static ros::Publisher lanes_pub;
static ros::Publisher initpose_pub;
static ros::Publisher setpose_pub;

static ros::Subscriber gps_sub;
static ros::Subscriber imu_sub;
static ros::Subscriber pcd_sub;
static ros::Subscriber rviz_sub;

static bool load_lanes = false;
static bool load_map   = false;
static bool init_gps   = false;
static bool init_rviz  = false;
static bool init_pcd   = false;
static bool filter     = true;
static bool init 	     = false;
static bool s 		     = false;

static bool ndt;

static pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_aligned  (new pcl::PointCloud<pcl::PointXYZ>);
static pcl::PointCloud<pcl::PointXYZ>::Ptr tmp          (new pcl::PointCloud<pcl::PointXYZ>);
static pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_         (new pcl::PointCloud<pcl::PointXYZ>);

static pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud    (new pcl::PointCloud<pcl::PointXYZ>);

static pcl::PointCloud<pcl::PointXYZ>::Ptr leftest_lanes(new pcl::PointCloud<pcl::PointXYZ>);
static pcl::PointCloud<pcl::PointXYZ>::Ptr right_lanes	(new pcl::PointCloud<pcl::PointXYZ>);
static pcl::PointCloud<pcl::PointXYZ>::Ptr left_lanes   (new pcl::PointCloud<pcl::PointXYZ>);
static pcl::PointCloud<pcl::PointXYZ>::Ptr lanes        (new pcl::PointCloud<pcl::PointXYZ>);

static pcl::PointCloud<pcl::PointXYZ>::Ptr dismissed_cloud    (new pcl::PointCloud<pcl::PointXYZ>);

static sensor_msgs::PointCloud2 map_msg;
static geometry_msgs::PoseWithCovarianceStamped init_pose;

static sensor_msgs::PointCloud2 leftlane_msg;
static sensor_msgs::PointCloud2 leftestlane_msg;
static sensor_msgs::PointCloud2 rightlane_msg;
static sensor_msgs::PointCloud2 lanes_msg;

static nav_msgs::Odometry msg;
static sensor_msgs::PointCloud2 input;

static double latitude, longitude, altitude;
static double origin_latitude, origin_longitude, origin_altitude;

static double imu_qx = 0;
static double imu_qy = 0;
static double imu_qz = 0;
static double imu_qw = 0;

static double init_guess_x = 0;
static double init_guess_y = 0;
static double init_guess_z = 0;

static double init_guess_qx = 0;
static double init_guess_qy = 0;
static double init_guess_qz = 0;
static double init_guess_qw = 0;

static double init_guess_roll = 0;
static double init_guess_pitch = 0;
static double init_guess_yaw = 0;

static double ndt_pose_x = 0;
static double ndt_pose_y = 0;
static double ndt_pose_z = 0;

static double ndt_pose_qx = 0;
static double ndt_pose_qy = 0;
static double ndt_pose_qz = 0;
static double ndt_pose_qw = 0;

static double ndt_pose_pitch = 0;
static double ndt_pose_roll = 0;
static double ndt_pose_yaw = 0;

static double epsilon, step, res, leaf, scoreThreshold;
static int iter;

static tf2::Quaternion ndt_quat;

static string right_path;
static string left_path;
static string lane_path;
static string path;

static string lidar_topic;
static string imu_topic;
static string gps_topic;

static string method;

static Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity(); 
static Eigen::Matrix4f dismissed_guess = Eigen::Matrix4f::Identity(); 

double n = omp_get_max_threads();


//remove top of the car
static void apply_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr& tmp){

  pcl::PointIndices::Ptr toremove (new pcl::PointIndices ());
  pcl::ExtractIndices<pcl::PointXYZ> filter (true);

  for (int p = 0; p < (*tmp).size(); p++)
  {
    pcl::PointXYZ pt(tmp->points[p].x, tmp->points[p].y, tmp->points[p].z);
  float r = 2.0f;
  if (std::pow(pt.x,2) + std::pow(pt.y,2) < std::pow(r,2) && 1 < pt.z < 5){
      toremove->indices.push_back(p);
  }
  }

  filter.setInputCloud(tmp);
  filter.setIndices(toremove);
  filter.setNegative(true);
  filter.filter(*tmp);

}

//define region on interest for ndt
static void apply_filter_2(pcl::PointCloud<pcl::PointXYZ>::Ptr& tmp){

  pcl::PointIndices::Ptr tokeep (new pcl::PointIndices ());
  pcl::ExtractIndices<pcl::PointXYZ> filter (true);

  for (int p = 0; p < (*tmp).size(); p++)
  {
    pcl::PointXYZ pt(tmp->points[p].x, tmp->points[p].y, tmp->points[p].z);
  float r = 50.0f;
  if (std::pow(pt.x-init_guess_x,2) + std::pow(pt.y-init_guess_y,2) < std::pow(r,2)){
      tokeep->indices.push_back(p);
  }
  }

  filter.setInputCloud(tmp);
  filter.setIndices(tokeep);
  filter.filter(*tmp);

}

void imu_callback(sensor_msgs::Imu msg)
{
  imu_qx = msg.orientation.x;
  imu_qy = msg.orientation.y;
  imu_qz = msg.orientation.z;
  imu_qw = msg.orientation.w;
}



void map_callback(string path){

	if (!load_map){

	  ROS_INFO("Loading Map...");

		pcl::io::loadPCDFile<pcl::PointXYZ> (path, *map_cloud);

		if (map_cloud->size()==0){
	    	ROS_INFO("Map is empty...");
	    	return;
	  }

	  load_map = true;

	 	pcl::toROSMsg(*map_cloud, map_msg);
	  map_msg.header.frame_id = "map";
	  map_pub.publish(map_msg);
	  ROS_INFO("Map has been published.");

	}

}


void gps_callback(sensor_msgs::NavSatFix msg){

	if(!init_gps){

  	double origin_utm_y, origin_utm_x;
  	string origin_utm_zone;

  	gps_common::LLtoUTM(origin_latitude, origin_longitude, origin_utm_y, origin_utm_x, origin_utm_zone);

  	longitude = msg.longitude;
  	latitude 	= msg.latitude;
  	altitude 	= msg.altitude;

  	double utm_y, utm_x;
  	string utm_zone;

  	gps_common::LLtoUTM(latitude, longitude, utm_y, utm_x, utm_zone);

  	double d = sqrt(pow(utm_y-origin_utm_y,2)+pow(utm_x-origin_utm_x,2));

    init_guess_x 	= utm_x-origin_utm_x;
    init_guess_y 	= utm_y-origin_utm_y; //utm is in meters
    init_guess_z 	= altitude-origin_altitude; //altitude is in meters

    init_guess_qx 	= imu_qx;
    init_guess_qy 	= imu_qy;
    init_guess_qz 	= imu_qz;
    init_guess_qw 	= imu_qw;

  	init_gps = true;

  	ROS_INFO("Loaded GPS guess!");

	}

	nav_msgs::Odometry gps_odom;
  gps_odom.header.frame_id = "map";

  gps_odom.pose.pose.position.x = init_guess_x;
  gps_odom.pose.pose.position.y = init_guess_y;
  gps_odom.pose.pose.position.z = init_guess_z;

  gps_odom.pose.pose.orientation.x = init_guess_qx;
  gps_odom.pose.pose.orientation.y = init_guess_qy;
  gps_odom.pose.pose.orientation.z = init_guess_qz;
  gps_odom.pose.pose.orientation.w = init_guess_qw;

  initguess_pub.publish(gps_odom);

}

void rviz_callback(geometry_msgs::PoseWithCovarianceStamped msg){

  if(!init_rviz){

    init_guess_x = msg.pose.pose.position.x;
    init_guess_y = msg.pose.pose.position.y;
    init_guess_z = msg.pose.pose.position.z;

    init_guess_qx = msg.pose.pose.orientation.x;
    init_guess_qy = msg.pose.pose.orientation.y;
    init_guess_qz = msg.pose.pose.orientation.w;
    init_guess_qw = msg.pose.pose.orientation.z;

    init_rviz = true;

    ROS_INFO("Loaded RViz guess!");

  }

  nav_msgs::Odometry rviz_odom;
  rviz_odom.header.frame_id = "map";

  rviz_odom.pose.pose.position.x = init_guess_x;
  rviz_odom.pose.pose.position.y = init_guess_y;
  rviz_odom.pose.pose.position.z = init_guess_z;

  rviz_odom.pose.pose.orientation.x = init_guess_qx;
  rviz_odom.pose.pose.orientation.y = init_guess_qy;
  rviz_odom.pose.pose.orientation.z = init_guess_qz;
  rviz_odom.pose.pose.orientation.w = init_guess_qw;

  initguess_pub.publish(rviz_odom);

}

void getInitPos(){

  while(!s){

  tf::Quaternion q(init_guess_qx, init_guess_qy, init_guess_qz, init_guess_qw);
  tf::Matrix3x3 m(q);
  m.getRPY(init_guess_roll, init_guess_pitch, init_guess_yaw);

  //initial guess of the transformation between two frames using gps 
  Eigen::Translation3f init_translation (init_guess_x, init_guess_y, init_guess_z);
  Eigen::AngleAxisf init_rotation_x(init_guess_roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf init_rotation_y(init_guess_pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf init_rotation_z(init_guess_yaw, Eigen::Vector3f::UnitZ());
  init_guess = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();


  bool f = (isnan(init_guess.array())).any();

  if(method=="gps"){

    if (f)
      {
        ROS_INFO("Waiting for initial guess from gps...");
        return;
      }

  }else if(method=="rviz"){

    if (f)
      {
        ROS_INFO("Waiting for initial guess from RViz...");
        return;
      }
  }


  if (tmp_->size()==0)
  {
       ROS_INFO("Waiting for PointCloud...");
       return;
  }

  if (dismissed_cloud->points.size()>0){

    Eigen::Matrix4f T12 = dismissed_guess.inverse()*init_guess;
    pcl::transformPointCloud (*dismissed_cloud, *dismissed_cloud, T12);

    ROS_INFO("Adding previously dissmissed pcd to input... ");
    *tmp += *dismissed_cloud;

  }


  Eigen::Matrix4f Identity = Eigen::Matrix4f::Identity();
  // pcl::transformPointCloud (*tmp_, *tmp, init_guess);
  pcl::transformPointCloud (*tmp_, *tmp, Identity);

  if(filter = true){
  apply_filter(tmp);
  apply_filter_2(map_cloud);
  }


  static sensor_msgs::PointCloud2 laserCloudTemp;
  pcl::toROSMsg(*map_cloud, laserCloudTemp);
  laserCloudTemp.header.frame_id = "map";
  tmp_pub.publish(laserCloudTemp);


  //filtering to reduce computing time
  pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
  approximate_voxel_filter.setLeafSize (leaf, leaf, leaf);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_tmp (new pcl::PointCloud<pcl::PointXYZ>);
  approximate_voxel_filter.setInputCloud (tmp);
  approximate_voxel_filter.filter (*filtered_tmp);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_map (new pcl::PointCloud<pcl::PointXYZ>);
  approximate_voxel_filter.setInputCloud (map_cloud);
  approximate_voxel_filter.filter (*filtered_map);


  pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt_omp(new pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());

  ndt_omp->setNumThreads(n);
  ndt_omp->setNeighborhoodSearchMethod(pclomp::KDTREE);

  ndt_omp->setTransformationEpsilon (epsilon);
  ndt_omp->setStepSize (step);
  ndt_omp->setResolution (res);
  ndt_omp->setMaximumIterations (iter);

  ndt_omp->setInputSource (tmp);
  ndt_omp->setInputTarget (map_cloud);

  ndt_omp->align (*tmp_aligned,init_guess);

  if (ndt_omp->getFitnessScore() < scoreThreshold){

  Eigen::Matrix4f Transf = Eigen::Matrix4f::Identity();

  std::cout << "Normal Distributions Transform has converged after " << ndt_omp->getFinalNumIteration() << " iters" << std::endl;
  std::cout << "The score is " << ndt_omp->getFitnessScore () << "\n" << std::endl;

  Transf =  ndt_omp->getFinalTransformation();
  init_guess = Transf;

  std::cout << "The final transform from the NDT is : \n" << std::endl;
  std::cout << Transf << std::endl;
  std::cout << "\n" << std::endl;

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

  ndtodom_pub.publish(ndt_odom);

  init_pose.header.stamp = laserCloudTemp.header.stamp;
  init_pose.header.frame_id = laserCloudTemp.header.frame_id;

  init_pose.pose.pose.position.x = ndt_pose_x;
  init_pose.pose.pose.position.y = ndt_pose_y;
  init_pose.pose.pose.position.z = ndt_pose_z;

  init_pose.pose.pose.orientation.x = ndt_quat.x();
  init_pose.pose.pose.orientation.y = ndt_quat.y();
  init_pose.pose.pose.orientation.z = ndt_quat.z();
  init_pose.pose.pose.orientation.w = ndt_quat.w();

  initpose_pub.publish(init_pose);

  pcl::transformPointCloud (*tmp_, *tmp_aligned, Transf);
  
  static sensor_msgs::PointCloud2 fScan;
  pcl::toROSMsg(*tmp_aligned, fScan);
  fScan.header.frame_id = "map";
  fscan_pub.publish(fScan);

  bool l = (isnan(Transf.array())).any();

  if(!l) s = true;

  }
  else{

    ROS_INFO("Bad NDT score. Try again ...\n");

    dismissed_guess = init_guess;
    *dismissed_cloud += *tmp;

  }

  }

}

void pcd_callback(sensor_msgs::PointCloud2 input)
{
    if(!init_pcd){

      pcl::fromROSMsg(input, *tmp_);
      ROS_INFO("Loaded velodyne scan!");
      init_pcd = true;

    }

    if(tmp_->size()>0 && !s){

      init_pcd = true;
      getInitPos();

    }
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "init_pose");

  ros::NodeHandle nh("~");

  map_pub 		 = nh.advertise<sensor_msgs::PointCloud2>("/pcd_map", 1); //pcd map
  tmp_pub 		 = nh.advertise<sensor_msgs::PointCloud2>("/tmp_map", 1); //pcd roi
  fscan_pub 	 = nh.advertise<sensor_msgs::PointCloud2>("/fscan", 1);   //tranf init scan

  initguess_pub = nh.advertise<nav_msgs::Odometry>("/init_guess", 1);
  ndtodom_pub 	= nh.advertise<nav_msgs::Odometry>("/ndt_odom", 1);

  initpose_pub  = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initial_pose", 8);
  setpose_pub   = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/set_pose", 8);

  nh.getParam("scoreThreshold", scoreThreshold);
  nh.getParam("epsilon", epsilon);
  nh.getParam("step", step);
  nh.getParam("iter", iter);
  nh.getParam("leaf", leaf);
  nh.getParam("res", res);

  nh.getParam("origin_longitude", origin_longitude);
  nh.getParam("origin_latitude" , origin_latitude);
  nh.getParam("origin_altitude" , origin_altitude);

  nh.getParam("path", path);

  nh.getParam("lidar_topic", lidar_topic);
  nh.getParam("imu_topic", imu_topic);
  nh.getParam("gps_topic", gps_topic);

  nh.getParam("method", method);

  map_callback(path);

  imu_sub = nh.subscribe(imu_topic,   1, imu_callback);
  pcd_sub = nh.subscribe(lidar_topic, 1, pcd_callback);

  if(method=="gps"){
  gps_sub = nh.subscribe(gps_topic,   1, gps_callback);
  }else if(method=="rviz"){
  rviz_sub = nh.subscribe("/initialpose", 1, rviz_callback);
  }

  while(ros::ok()){

    ros::spin();

  }

  return 0;

}
