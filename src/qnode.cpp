/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/projector_calibration/qnode.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

using namespace std;

namespace projector_calibration {

 /*****************************************************************************
  ** Implementation
  *****************************************************************************/

 QNode::QNode(int argc, char** argv ) :
	          init_argc(argc),
	          init_argv(argv)
 {init();}

 QNode::~QNode() {
  if(ros::isStarted()) {
   ros::shutdown(); // explicitly needed since we use ros::start();
   ros::waitForShutdown();
  }
  wait();
 }




 bool QNode::init() {
  ros::init(init_argc,init_argv,"projector_calibration");
  if ( ! ros::master::check() ) {
   return false;
  }
  ros::start(); // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle n;

  user_input = new User_Input();
  user_input->init();


  //  ros::param::param<double>("foo", bar, 42);

  // Add your ros communications here.
  //chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
  start();
  return true;
 }



 void QNode::writeToOutput(const std::stringstream& msg){

  ROS_INFO_STREAM(msg.str());

  logging_model.insertRows(logging_model.rowCount(),1);


  QVariant new_row(QString(msg.str().c_str()));
  logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
  Q_EMIT loggingUpdated(); // used to readjust the scrollbar
 }




 bool QNode::init(const std::string &master_url, const std::string &host_url) {
  std::map<std::string,std::string> remappings;
  remappings["__master"] = master_url;
  remappings["__hostname"] = host_url;
  ros::init(remappings,"projector_calibration");
  if ( ! ros::master::check() ) {
   return false;
  }
  ros::start(); // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle n;
  // Add your ros communications here.


  start();
  return true;
 }


 void QNode::imgCloudCB(const sensor_msgs::ImageConstPtr& img_ptr, const sensor_msgs::PointCloud2ConstPtr& cloud_ptr){
  // ROS_INFO("GOT KINECT DATA");

  pcl::fromROSMsg(*cloud_ptr, current_cloud);
  calibrator.setInputCloud(current_cloud);


  if (calibrator.isKinectTrafoSet() && pub_cloud_worldsystem.getNumSubscribers()>0){
   Cloud::Ptr msg = calibrator.cloud_moved.makeShared();
   msg->header.frame_id = "/openni_rgb_optical_frame";
   msg->header.stamp = ros::Time::now ();
   pub_cloud_worldsystem.publish(msg);
  }



  if (user_interaction_active && calibrator.projMatrixSet()){
   Cloud area;
   if (calibrator.getProjectionAreain3D(area)) {
    user_input->setCloud(calibrator.cloud_moved, area);
    cv::Point3f tip;
    if (user_input->getUserPositionTrivial(tip)){
     cv::Point2f px = applyPerspectiveTrafo(tip, calibrator.proj_Matrix);
     cv::circle(calibrator.projector_image, px, 10, CV_RGB(255,0,0), -1);
     Q_EMIT update_projector_image();
    }
   }
  }


  if (depth_visualization_active && calibrator.projMatrixSet()){
   projectCloudIntoProjector(calibrator.cloud_moved, calibrator.proj_Matrix, calibrator.projector_image);
   Q_EMIT update_projector_image();
  }




  //  ROS_INFO("GOT cloud with %zu points", current_cloud.size());
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img_ptr , sensor_msgs::image_encodings::BGR8);

  current_col_img = cv_ptr->image;

  Q_EMIT received_col_Image();
 }

 void QNode::run() {
  ros::Rate loop_rate(30);


  ROS_INFO("Starting to run");

  ros::NodeHandle nh;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> policy;
  message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/camera/rgb/image_color", 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh, "/camera/rgb/points", 1);
  message_filters::Synchronizer<policy> sync(policy(2), image_sub, cloud_sub);
  sync.registerCallback(boost::bind(&QNode::imgCloudCB,this, _1, _2));

  pub_cloud_worldsystem = nh.advertise<Cloud>("cloud_world", 1);
  pub_3d_calib_points = nh.advertise<Cloud>("calibration_points_3d", 1);

  //  ros::Subscriber sub = n.subscribe("chatter", 1000, &Listener::callback, &listener);

  while ( ros::ok() ) {

   ros::spinOnce();

   //   if (current_col_img.cols > 0){
   //    cv::imshow("kinect", current_col_img);
   //    cv::waitKey(1);
   //   }

   //   std_msgs::String msg;
   //   std::stringstream ss;
   //   ss << "hello world " << count;
   //   msg.data = ss.str();
   //   chatter_publisher.publish(msg);
   //   log(Info,std::string("I sent: ")+msg.data);
   //   ros::spinOnce();
   //   loop_rate.sleep();
   //   ++count;

   loop_rate.sleep();

  }
  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
  Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
 }


 void QNode::log( const LogLevel &level, const std::string &msg) {
  logging_model.insertRows(logging_model.rowCount(),1);
  std::stringstream logging_model_msg;
  switch ( level ) {
  case(Debug) : {
   ROS_DEBUG_STREAM(msg);
   logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
   break;
  }
  case(Info) : {
   ROS_INFO_STREAM(msg);
   logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
   break;
  }
  case(Warn) : {
   ROS_WARN_STREAM(msg);
   logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
   break;
  }
  case(Error) : {
   ROS_ERROR_STREAM(msg);
   logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
   break;
  }
  case(Fatal) : {
   ROS_FATAL_STREAM(msg);
   logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
   break;
  }
  }
  QVariant new_row(QString(logging_model_msg.str().c_str()));
  logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
  Q_EMIT loggingUpdated(); // used to readjust the scrollbar
 }

}  // namespace projector_calibration
