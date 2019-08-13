// MAHDI ELHOUSNI, WPI. 2019. melhousni@wpi.edu

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <pclomp/ndt_omp.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/ndt.h>

#include <pcl_conversions/pcl_conversions.h>

using namespace std;

static ros::Publisher edguess_pub;
static ros::Publisher ndtodom_pub;
static ros::Publisher setpose_pub;
static ros::Publisher initpose_pub;
static ros::Publisher scan_pub;

static ros::Subscriber initpose_sub;
static ros::Subscriber odom_sub;
static ros::Subscriber scan_sub;
static ros::Subscriber map_sub;

static bool init_odom = false;
static bool init_ndt  = false;
static bool filter    = true;
static bool init      = false;
static bool s         = false;

static tf::TransformListener* pListener = NULL;

static pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud    (new pcl::PointCloud<pcl::PointXYZ>);
static pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_   (new pcl::PointCloud<pcl::PointXYZ>);

static pcl::PointCloud<pcl::PointXYZ>::Ptr tmp          (new pcl::PointCloud<pcl::PointXYZ>);
static pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_         (new pcl::PointCloud<pcl::PointXYZ>);
static pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_aligned  (new pcl::PointCloud<pcl::PointXYZ>);

static pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud   (new pcl::PointCloud<pcl::PointXYZ>);

static nav_msgs::Odometry msg;
static sensor_msgs::PointCloud2 input;

static double score; 
static double scoreThreshold = 0.0;

static double init_guess_x;
static double init_guess_y;
static double init_guess_z;

static double init_guess_qx;
static double init_guess_qy;
static double init_guess_qz;
static double init_guess_qw;

static double init_guess_pitch;
static double init_guess_yaw;
static double init_guess_roll;

static double ed_guess_x;
static double ed_guess_y;
static double ed_guess_z;

static double ed_guess_qx;
static double ed_guess_qy;
static double ed_guess_qz;
static double ed_guess_qw;

static double ed_guess_pitch = 0;
static double ed_guess_yaw = 0;
static double ed_guess_roll = 0;

static double ndt_pose_x = 0;
static double ndt_pose_y = 0;
static double ndt_pose_z = 0;
static double ndt_pose_roll = 0;
static double ndt_pose_pitch = 0;
static double ndt_pose_yaw = 0;

static double init_odom_x = 0;
static double init_odom_y = 0;
static double init_odom_z = 0;

static double init_odom_qx = 0;
static double init_odom_qy = 0;
static double init_odom_qz = 0;
static double init_odom_qw = 0; 

static double odom_x = 0;
static double odom_y = 0;
static double odom_z = 0;

static double odom_pitch = 0;
static double odom_yaw = 0;
static double odom_roll = 0;

static double odom_qx = 0;
static double odom_qy = 0;
static double odom_qz = 0;
static double odom_qw = 0; 

static double pose_x = 0; 
static double pose_y = 0; 
static double pose_z = 0; 

static double pose_pitch = 0;
static double pose_yaw = 0;
static double pose_roll = 0;

static double pose_qx = 0; 
static double pose_qy = 0; 
static double pose_qz = 0; 
static double pose_qw = 0; 

static double epsilon, step, res, leaf;
static int iter;

static string lidar_topic, odom_topic;

static tf::Quaternion odom_quat;
static tf2::Quaternion ndt_quat, ed_quat;

static Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity(); 

double n = omp_get_max_threads();

//remove top of the car
static void apply_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr& tmp){

  pcl::PointIndices::Ptr toremove (new pcl::PointIndices ());
  pcl::ExtractIndices<pcl::PointXYZ> filter (true);

  for (int p = 0; p < (*tmp).size(); p++)
  {
    pcl::PointXYZ pt(tmp->points[p].x, tmp->points[p].y, tmp->points[p].z);
  double r = 2.0f;
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
  double r = 50.0f;
  if (std::pow(pt.x-init_guess_x,2) + std::pow(pt.y-init_guess_y,2) < std::pow(r,2)){
      tokeep->indices.push_back(p);
  }
  }

  filter.setInputCloud(tmp);
  filter.setIndices(tokeep);
  filter.filter(*tmp);

}

//define region of interest for lanes
static void apply_filter_3(pcl::PointCloud<pcl::PointXYZ>::Ptr& tmp, pcl::PointCloud<pcl::PointXYZ>::Ptr& rslt, double pose_x, double pose_y, double r){

  pcl::PointIndices::Ptr tokeep (new pcl::PointIndices ());
  pcl::ExtractIndices<pcl::PointXYZ> filter (true);

  for (int p = 0; p < (*tmp).size(); p++)
  {
    pcl::PointXYZ pt(tmp->points[p].x, tmp->points[p].y, tmp->points[p].z);
  // double r = 10.0f;
  if (std::pow(pt.x-pose_x,2) + std::pow(pt.y-pose_y,2) < std::pow(r,2)){
      tokeep->indices.push_back(p);
  }
  }

  filter.setInputCloud(tmp);
  filter.setIndices(tokeep);
  filter.filter(*rslt);

}

static void init_callback(geometry_msgs::PoseWithCovarianceStamped::ConstPtr init_pose){

  init_guess_x = init_pose->pose.pose.position.x;
  init_guess_y = init_pose->pose.pose.position.y;
  init_guess_z = init_pose->pose.pose.position.z;

  init_guess_qx = init_pose->pose.pose.orientation.x;
  init_guess_qy = init_pose->pose.pose.orientation.y;
  init_guess_qz = init_pose->pose.pose.orientation.z;
  init_guess_qw = init_pose->pose.pose.orientation.w;

}

static void map_callback(const sensor_msgs::PointCloud2::ConstPtr& input){

  pcl::fromROSMsg(*input, *map_cloud_);

}

// static void odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
static void odom_callback(const nav_msgs::Odometry msg)
{

  odom_x = msg.pose.pose.position.x;
  odom_y = msg.pose.pose.position.y;
  odom_z = msg.pose.pose.position.z;

  odom_qx = msg.pose.pose.orientation.x;
  odom_qy = msg.pose.pose.orientation.y;
  odom_qz = msg.pose.pose.orientation.z;
  odom_qw = msg.pose.pose.orientation.w;


}

static Eigen::Matrix4f guess_offset = Eigen::Matrix4f::Identity();
static Eigen::Matrix4f previnit     = Eigen::Matrix4f::Identity();
static Eigen::Matrix4f ed_guess     = Eigen::Matrix4f::Identity();
static Eigen::Matrix4f prevTransf   = Eigen::Matrix4f::Identity();

static tf::Quaternion init_q;
static tf::Vector3 init_t;

// static void ndt_odom_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
static void ndt_odom_callback(sensor_msgs::PointCloud2 input)
{

  tf::Matrix3x3 m(tf::Quaternion(odom_qx, odom_qy, odom_qz, odom_qw));
  m.getRPY(odom_roll, odom_pitch, odom_yaw);

  //initial guess of the transformation between two frames using gps 
  Eigen::Translation3f init_translation(odom_x, odom_y, odom_z);
  Eigen::AngleAxisf init_rotation_x(odom_roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf init_rotation_y(odom_pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf init_rotation_z(odom_yaw, Eigen::Vector3f::UnitZ());
  init_guess = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();

  if(!init_odom){

    tf::Matrix3x3 m(tf::Quaternion(init_guess_qx, init_guess_qy, init_guess_qz, init_guess_qw));
    m.getRPY(init_guess_roll, init_guess_pitch, init_guess_yaw);

    Eigen::Translation3f init_translation (init_guess_x, init_guess_y, init_guess_z);
    Eigen::AngleAxisf init_rotation_x(init_guess_roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf init_rotation_y(init_guess_pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf init_rotation_z(init_guess_yaw, Eigen::Vector3f::UnitZ());

    prevTransf = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();
    previnit = init_guess;

    init_odom = true;

  }

  guess_offset = previnit.inverse()*init_guess;
  ed_guess = prevTransf*guess_offset;


  pcl::fromROSMsg(input, *tmp);

  bool f = (isnan(init_guess.array())).any();

  if (f)
  {
    ROS_INFO("Waiting for initial guess from odom..");
    return;
  }

  if (!init_ndt){
    pcl::transformPointCloud (*tmp, *prev_cloud, guess_offset);
    init_ndt = true;
  } 

  // apply filter to remove rays that bounce of the car
  if(filter = true){
  apply_filter(tmp);
  apply_filter_3(map_cloud_, map_cloud, ed_guess(0, 3), ed_guess(1, 3), 50.0);
  }

  pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
  approximate_voxel_filter.setLeafSize (leaf, leaf, leaf);

  //filtering to reduce computing time
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_tmp (new pcl::PointCloud<pcl::PointXYZ>);
  approximate_voxel_filter.setInputCloud (tmp);
  approximate_voxel_filter.filter (*filtered_tmp);

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_map (new pcl::PointCloud<pcl::PointXYZ>);
  approximate_voxel_filter.setInputCloud (map_cloud);
  approximate_voxel_filter.filter (*filtered_map);

  //initializing NDT and setting up scale dependent parameters
  pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt_omp(new pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());

  ndt_omp->setNumThreads(n);
  ndt_omp->setNeighborhoodSearchMethod(pclomp::DIRECT7);

  ndt_omp->setTransformationEpsilon (epsilon);
  ndt_omp->setMaximumIterations (iter);
  ndt_omp->setResolution (res);
  ndt_omp->setStepSize (step);

  ndt_omp->setInputSource (tmp);
  ndt_omp->setInputTarget (filtered_map);

  ndt_omp->align (*tmp_aligned,ed_guess);

  score = ndt_omp->getFitnessScore();

  if(ndt_omp->getFitnessScore() < scoreThreshold){

  std::cout << "Normal Distributions Transform has converged after " << ndt_omp->getFinalNumIteration() << " iters" << std::endl;
  std::cout << "The score is " << ndt_omp->getFitnessScore() << "\n" << std::endl;

  Eigen::Matrix4f Transf = Eigen::Matrix4f::Identity();
  Transf =  ndt_omp->getFinalTransformation();
  prevTransf = Transf;

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
  ndt_odom.header.stamp = input.header.stamp;
  ndt_odom.header.frame_id = "map";

  ndt_odom.pose.pose.position.x = Transf(0, 3);
  ndt_odom.pose.pose.position.y = Transf(1, 3);
  ndt_odom.pose.pose.position.z = Transf(2, 3);

  ndt_odom.pose.pose.orientation.x = ndt_quat.x();
  ndt_odom.pose.pose.orientation.y = ndt_quat.y();
  ndt_odom.pose.pose.orientation.z = ndt_quat.z();
  ndt_odom.pose.pose.orientation.w = ndt_quat.w();

  ndtodom_pub.publish(ndt_odom);

  tf::Matrix3x3 InitMat;

  InitMat.setValue(static_cast<double>(ed_guess(0, 0)), static_cast<double>(ed_guess(0, 1)),
                   static_cast<double>(ed_guess(0, 2)), static_cast<double>(ed_guess(1, 0)),
                   static_cast<double>(ed_guess(1, 1)), static_cast<double>(ed_guess(1, 2)),
                   static_cast<double>(ed_guess(2, 0)), static_cast<double>(ed_guess(2, 1)),
                   static_cast<double>(ed_guess(2, 2)));

  ed_guess_x = ed_guess(0, 3);
  ed_guess_y = ed_guess(1, 3);
  ed_guess_z = ed_guess(2, 3);

  InitMat.getRPY(ed_guess_roll, ed_guess_pitch, ed_guess_yaw, 1);
  ed_quat.setRPY(ed_guess_roll, ed_guess_pitch, ed_guess_yaw);

  nav_msgs::Odometry ed_odom;
  ed_odom.header.stamp = input.header.stamp;
  ed_odom.header.frame_id = "map";

  ed_odom.pose.pose.position.x = ed_guess_x;
  ed_odom.pose.pose.position.y = ed_guess_y;
  ed_odom.pose.pose.position.z = ed_guess_z;

  ed_odom.pose.pose.orientation.x = ed_quat.x();
  ed_odom.pose.pose.orientation.y = ed_quat.y();
  ed_odom.pose.pose.orientation.z = ed_quat.z();
  ed_odom.pose.pose.orientation.w = ed_quat.w();

  edguess_pub.publish(ed_odom);

  //check if ndt transform is available
  f = (isnan(Transf.array())).any();

  if (f)
  {
    ROS_INFO("Waiting for the ndt transform...");
    return;  
  }

  static sensor_msgs::PointCloud2 cScan;
  pcl::toROSMsg(*tmp_aligned, cScan);
  cScan.header.frame_id = "map";
  scan_pub.publish(cScan);

  }
  else{

    ROS_INFO("Rejected because of the weak NDT score!\n");

    prevTransf = ed_guess;

    tf::Matrix3x3 GuessMat;

    GuessMat.setValue(static_cast<double>(ed_guess(0, 0)), static_cast<double>(ed_guess(0, 1)),
                      static_cast<double>(ed_guess(0, 2)), static_cast<double>(ed_guess(1, 0)),
                      static_cast<double>(ed_guess(1, 1)), static_cast<double>(ed_guess(1, 2)),
                      static_cast<double>(ed_guess(2, 0)), static_cast<double>(ed_guess(2, 1)),
                      static_cast<double>(ed_guess(2, 2)));

    ndt_pose_x = ed_guess(0, 3);
    ndt_pose_y = ed_guess(1, 3);
    ndt_pose_z = ed_guess(2, 3);

    cout << ndt_pose_x << endl;

    GuessMat.getRPY(ndt_pose_roll, ndt_pose_pitch, ndt_pose_yaw, 1);
    ndt_quat.setRPY(ndt_pose_roll, ndt_pose_pitch, ndt_pose_yaw);

    nav_msgs::Odometry ndt_odom;
    ndt_odom.header.stamp = input.header.stamp;
    ndt_odom.header.frame_id = "map";

    ndt_odom.pose.pose.position.x = ndt_pose_x;
    ndt_odom.pose.pose.position.y = ndt_pose_y;
    ndt_odom.pose.pose.position.z = ndt_pose_z;

    ndt_odom.pose.pose.orientation.x = ndt_quat.x();
    ndt_odom.pose.pose.orientation.y = ndt_quat.y();
    ndt_odom.pose.pose.orientation.z = ndt_quat.z();
    ndt_odom.pose.pose.orientation.w = ndt_quat.w();

    ndtodom_pub.publish(ndt_odom);

    //check if ndt transform is available
    f = (isnan(ed_guess.array())).any();

    if (f)
    {
      ROS_INFO("Waiting for the ekf guess...");
      return;  
    }

    pcl::transformPointCloud (*tmp, *tmp_aligned, ed_guess);

    static sensor_msgs::PointCloud2 cScan;
    pcl::toROSMsg(*tmp_aligned, cScan);
    cScan.header.frame_id = "map";
    scan_pub.publish(cScan);

  }


  previnit = init_guess;

}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "map_localization");

  ros::NodeHandle nh("~");

  pListener = new (tf::TransformListener);

  ROS_INFO("Localization module launched");

  initpose_sub    = nh.subscribe("/initial_pose", 1, init_callback);
  map_sub         = nh.subscribe("/pcd_map", 1, map_callback);

  scan_pub        = nh.advertise<sensor_msgs::PointCloud2>("/cscan", 1);

  edguess_pub     = nh.advertise<nav_msgs::Odometry>("/ed_guess", 1);

  setpose_pub     = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/set_pose", 8);
  ndtodom_pub     = nh.advertise<nav_msgs::Odometry>("/ndt_odom", 1);


  nh.getParam("scoreThreshold", scoreThreshold);
  nh.getParam("epsilon", epsilon);
  nh.getParam("step", step);
  nh.getParam("iter", iter);
  nh.getParam("leaf", leaf);
  nh.getParam("res", res);

  nh.getParam("lidar_topic", lidar_topic);
  nh.getParam("odom_topic" , odom_topic);

  bool f = isnan(init_guess_x) || isnan(init_guess_y) || isnan(init_guess_z);

  bool init = false;

  ROS_INFO("Waiting for init pose ...");

  while(!init) {
    ros::spinOnce();
    if (init_guess_x > 0){

      // init_q = tf::Quaternion (init_guess_qx, init_guess_qy, init_guess_qz, init_guess_qw);
      // init_t = tf::Vector3 (init_guess_x, init_guess_y, init_guess_z);

        init = true;  
    }
  }

  ROS_INFO("Received init pose!");

  odom_sub = nh.subscribe(odom_topic,    1, odom_callback);
  scan_sub = nh.subscribe(lidar_topic,   1, ndt_odom_callback);

  while(ros::ok()){

    ros::spin();

  }

  return 0;

}