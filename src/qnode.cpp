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

  ROS_WARN("INIT");

  // reading the number of corners from file
  int check_width, check_height;
  ros::param::param<int>("projector_calibration/checkerboard_internal_corners_x", check_width, 10);
  ros::param::param<int>("projector_calibration/checkerboard_internal_corners_y", check_height, 6 );
  calibrator.checkboard_size = cv::Size(check_width, check_height);

  // reading the projector's size from file
  int proj_width, proj_height;
  ros::param::param<int>("projector_calibration/projector_px_width", proj_width, 1024);
  ros::param::param<int>("projector_calibration/projector_px_height", proj_height, 768 );
  calibrator.proj_size = cv::Size(proj_width, proj_height);

  //  ros::param::param<double>("foo", bar, 42);

  // Add your ros communications here.
  chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
  start();
  return true;
 }



 void QNode::writeToOutput(const std::stringstream& msg){
  logging_model.insertRows(logging_model.rowCount(),1);


  QVariant new_row(QString(msg.str().c_str()));
  logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
  emit loggingUpdated(); // used to readjust the scrollbar
 }

 void QNode::writeFooToList(){
  // logging_model.insertRows(logging_model.rowCount(),1);
  std::stringstream logging_model_msg;

  logging_model_msg << "[" << foo << "] ";

  writeToOutput(logging_model_msg);

  // QVariant new_row(QString(logging_model_msg.str().c_str()));
  // logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
  // emit loggingUpdated(); // used to readjust the scrollbar

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
  chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
  start();
  return true;
 }

 void QNode::run() {
  ros::Rate loop_rate(30);
//  int count = 0;


  ROS_INFO("Starting to run");

  typedef sync_policies::ApproximateTime<Image, PointCloud2> policy;
//  message_filters::Subscriber<Image> image_sub(nh, "/camera/rgb/image_color", 1);
//  message_filters::Subscriber<PointCloud2> cloud_sub(nh, "/camera/rgb/points", 1);
//  Synchronizer<policy> sync(policy(2), image_sub, cloud_sub);
//  sync.registerCallback(boost::bind(&imgCB, _1, _2));



  while ( ros::ok() ) {






//   std_msgs::String msg;
//   std::stringstream ss;
//   ss << "hello world " << count;
//   msg.data = ss.str();
//   chatter_publisher.publish(msg);
//   log(Info,std::string("I sent: ")+msg.data);
//   ros::spinOnce();
//   loop_rate.sleep();
//   ++count;
  }
  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
  emit rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
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
  emit loggingUpdated(); // used to readjust the scrollbar
 }

}  // namespace projector_calibration
