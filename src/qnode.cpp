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

namespace projector_calibration
{

 /*****************************************************************************
  ** Implementation
  *****************************************************************************/

 QNode::QNode(int argc, char** argv) :
  init_argc(argc), init_argv(argv)
 {
  init();
 }

 QNode::~QNode()
 {
  if (ros::isStarted())
   {
    ros::shutdown(); // explicitly needed since we use ros::start();
    ros::waitForShutdown();
   }
  wait();
 }

 void
 QNode::eval_projection()
 {

  cv::Mat bw;

  cv::cvtColor(current_col_img, bw, CV_BGR2GRAY);

  if (area_mask.cols == current_col_img.cols)
   {
    cv::Mat foo;
    bw.copyTo(foo, area_mask);
    bw = foo;
   }

  cv::Mat bright;

  //  ROS_INFO("threshold: %i",calibrator.eval_brightness_threshold );

  cv::threshold(bw, bright, calibrator.eval_brightness_threshold, 255,
    CV_THRESH_BINARY);

  cv::dilate(bright,bright, cv::Mat());

  cv::Mat can;

  cv::Canny(bright, can, 2, 10);



  cv::Mat col;
  current_col_img.copyTo(col);



/*
  vector<cv::Vec3f> circles;
  cv::HoughCircles(can, circles, CV_HOUGH_GRADIENT, 1, can.rows / 4, 200, 10);
  for (size_t i = 0; i < circles.size(); i++)
   {


    cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
    int radius = cvRound(circles[i][2]);

    ROS_INFO("radius: %f", radius);
    cv::circle(col, center, 1, cv::Scalar(255, 0, 0), -1, 8, 0);
    cv::circle(col, center, radius, cv::Scalar(0, 0, 255), 1, 8, 0);

    cv::circle(can, center, 1, cv::Scalar::all(125), -1, 8, 0);
    cv::circle(can, center, radius, cv::Scalar::all(125), 1, 8, 0);

   }

  cv::namedWindow("edges");
  cv::imshow("edges", can);

  cv::namedWindow("foo");
  cv::imshow("foo", col);

  cv::namedWindow("foreground");
  cv::imshow("foreground", bright);
  cv::waitKey(10);

  return;
*/
  vector<vector<cv::Point> > contours;
  vector<cv::Vec4i> hierarchy;

  cv::Mat cpy;
  bright.copyTo(cpy);

  cv::findContours(cpy, contours, hierarchy, CV_RETR_TREE,
    CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

  cv::namedWindow("foreground");
  cv::imshow("foreground", bright);
  cv::waitKey(100);


  // compute area for each contour:
  if (contours.size() != 1)
   {
    stringstream ss;
    ss << "Found " << contours.size()
      << " contours, adapt threshold or adjust mask";
    writeToOutput(ss);
    return;
   }

  assert(contours.size()==1);

  //  float area = cv::contourArea(contours[0]);

  cv::drawContours(bright, contours, 0, cv::Scalar::all(125), 2, 8, hierarchy,
    0, cv::Point());

  current_col_img.copyTo(cpy);

  cv::Moments mom = cv::moments(contours[0]);

  cv::circle(cpy, cv::Point2f(mom.m10 / mom.m00, mom.m01 / mom.m00), 10,
    CV_RGB(0,255,0), 2);


  cv::RotatedRect rrect = cv::fitEllipse(contours[0]);

  cv::circle(cpy, rrect.center, 6, CV_RGB(255,255,0),2);

//  cv::namedWindow("foo");
//  cv::imshow("foo", col);

  // get mean of all points:
  pcl_Point mean;

  /*mean.x = mean.y = mean.z = 0;
  int valid = 0;
  for (uint i = 0; i < contours[0].size(); ++i)
   {
    cv::Point px = contours[0][i];
    pcl_Point p = calibrator.cloud_moved.at(px.x, px.y);
    if (p.x != p.x)
     continue;

    valid++;
    add(mean, p);

   }
   */

  mean = calibrator.cloud_moved.at(rrect.center.x, rrect.center.y);

//  if (valid == 0)
//   {
//    stringstream ss;
//    ss << "Found no valid 3d points on white target!";
//    writeToOutput(ss);
//    return;
//   }

//  div(mean, valid);

  stringstream ss;
  ss << "Mean Point: " << mean.x << " " << mean.y << " " << mean.z;
  writeToOutput(ss);

  calibrator.projector_image.setTo(0);
  cv::Point2f px = applyPerspectiveTrafo(mean, calibrator.proj_Matrix);

  cv::circle(calibrator.projector_image, px, 1, CV_RGB(0,255,0), 1);
  cv::circle(calibrator.projector_image, px, 5, CV_RGB(0,0,255), 1);
  cv::circle(calibrator.projector_image, px, 10, CV_RGB(255,0,0), 1);

  Q_EMIT
  update_projector_image();

  cv::namedWindow("bar");
  cv::imshow("bar", cpy);
  cv::waitKey(10);

  //  Cloud bright =


 }

 bool
 QNode::init()
 {
  ros::init(init_argc, init_argv, "projector_calibration");
  if (!ros::master::check())
   {
    return false;
   }
  ros::start(); // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle n;

  user_input = new User_Input();
  user_input->init();
  train_background = false;
  foreGroundVisualizationActive = false;

  start();
  return true;
 }

 void
 QNode::writeToOutput(const std::stringstream& msg)
 {

  ROS_INFO_STREAM(msg.str());

  logging_model.insertRows(logging_model.rowCount(), 1);

  QVariant new_row(QString(msg.str().c_str()));
  logging_model.setData(logging_model.index(logging_model.rowCount() - 1),
    new_row);
  Q_EMIT loggingUpdated(); // used to readjust the scrollbar
 }

 bool
 QNode::init(const std::string &master_url, const std::string &host_url)
 {
  std::map<std::string, std::string> remappings;
  remappings["__master"] = master_url;
  remappings["__hostname"] = host_url;
  ros::init(remappings, "projector_calibration");
  if (!ros::master::check())
   {
    return false;
   }
  ros::start(); // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle n;
  // Add your ros communications here.


  start();
  return true;
 }

 void
 QNode::imgCloudCB(const sensor_msgs::ImageConstPtr& img_ptr,
   const sensor_msgs::PointCloud2ConstPtr& cloud_ptr)
 {
  //   ROS_INFO("GOT KINECT DATA");

  pcl::fromROSMsg(*cloud_ptr, current_cloud);

  if (current_cloud.size() == 0)
   return;

  calibrator.setInputCloud(current_cloud);

  if (calibrator.isKinectTrafoSet()
    && pub_cloud_worldsystem.getNumSubscribers() > 0)
   {
    Cloud::Ptr msg = calibrator.cloud_moved.makeShared();
    msg->header.frame_id = "/openni_rgb_optical_frame";
    msg->header.stamp = ros::Time::now();
    pub_cloud_worldsystem.publish(msg);
   }

  if (train_background && !calibrator.isKinectTrafoSet())
   {
    ROS_WARN("Kinect Transformation has to be set before background can be computed");
   }

  if (calibrator.isKinectTrafoSet() && train_background)
   {
    uint cnt = detector.addTrainingFrame(calibrator.cloud_moved);
    ROS_INFO("Background calibration: added traingframe %i of %i", cnt, train_frame_cnt);
    if (cnt == train_frame_cnt)
     {
      detector.computeBackground(0.007);
      train_background = false;
      ROS_INFO("computed Background");
     }
   }

  if (foreGroundVisualizationActive && detector.isInitiated())
   {
    ROS_INFO("foreground visulization active");

    float max_val = 0.1;

    Cloud changed =
      detector.removeBackground(calibrator.cloud_moved, 0.01, 0.3); // everything between 1 and 30cm
    projectCloudIntoProjector(changed, calibrator.proj_Matrix,
      calibrator.projector_image, max_val, 0);

    Cloud::Ptr msg = changed.makeShared();
    msg->header.frame_id = "/openni_rgb_optical_frame";
    msg->header.stamp = ros::Time::now();
    pub_colored_cloud.publish(msg);

    Q_EMIT update_projector_image();
   }

  if (user_interaction_active && calibrator.projMatrixSet())
   {
    Cloud area;
    if (calibrator.getProjectionAreain3D(area))
     {
      user_input->setCloud(calibrator.cloud_moved, area);
      cv::Point3f tip;
      if (user_input->getUserPositionTrivial(tip))
       {
        cv::Point2f px = applyPerspectiveTrafo(tip, calibrator.proj_Matrix);
        cv::circle(calibrator.projector_image, px, 10, CV_RGB(255,0,0), -1);
        Q_EMIT update_projector_image();
       }
     }
   }

  if (depth_visualization_active && calibrator.projMatrixSet())
   {
    projectCloudIntoProjector(calibrator.cloud_moved, calibrator.proj_Matrix,
      calibrator.projector_image, visual_z_max, min_dist);

    Cloud colored = colorizeCloud(calibrator.cloud_moved, visual_z_max);
    Cloud::Ptr msg = colored.makeShared();
    msg->header.frame_id = "/openni_rgb_optical_frame";
    msg->header.stamp = ros::Time::now();
    pub_colored_cloud.publish(msg);

    Q_EMIT update_projector_image();
   }

  //  ROS_INFO("GOT cloud with %zu points", current_cloud.size());
  cv_ptr = cv_bridge::toCvCopy(img_ptr, sensor_msgs::image_encodings::BGR8);

  current_col_img = cv_ptr->image;

  Q_EMIT received_col_Image();

  //  ROS_INFO("Leaving CB");

 }

 void
 QNode::run()
 {
  ros::Rate loop_rate(10);

  ROS_INFO("Starting to run");

  ros::NodeHandle nh;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
    sensor_msgs::PointCloud2> policy;
  message_filters::Subscriber<sensor_msgs::Image> image_sub(nh,
    "/camera/rgb/image_color", 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh,
    "/camera/rgb/points", 1);
  message_filters::Synchronizer<policy> sync(policy(2), image_sub, cloud_sub);
  sync.registerCallback(boost::bind(&QNode::imgCloudCB, this, _1, _2));

  pub_cloud_worldsystem = nh.advertise<Cloud> ("cloud_world", 1);
  pub_3d_calib_points = nh.advertise<Cloud> ("calibration_points_3d", 1);
  pub_colored_cloud = nh.advertise<Cloud> ("colored", 1);
  //  ros::Subscriber sub = n.subscribe("chatter", 1000, &Listener::callback, &listener);

  while (ros::ok())
   {
    ros::spinOnce();
    loop_rate.sleep();
   }
  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
  Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
 }

 void
 QNode::log(const LogLevel &level, const std::string &msg)
 {
  logging_model.insertRows(logging_model.rowCount(), 1);
  std::stringstream logging_model_msg;
  switch (level)
   {
  case (Debug):
   {
    ROS_DEBUG_STREAM(msg);
    logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
    break;
   }
  case (Info):
   {
    ROS_INFO_STREAM(msg);
    logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
    break;
   }
  case (Warn):
   {
    ROS_WARN_STREAM(msg);
    logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
    break;
   }
  case (Error):
   {
    ROS_ERROR_STREAM(msg);
    logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
    break;
   }
  case (Fatal):
   {
    ROS_FATAL_STREAM(msg);
    logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
    break;
   }
   }
  QVariant new_row(QString(logging_model_msg.str().c_str()));
  logging_model.setData(logging_model.index(logging_model.rowCount() - 1),
    new_row);
  Q_EMIT loggingUpdated(); // used to readjust the scrollbar
 }

} // namespace projector_calibration
