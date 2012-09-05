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

 void QNode::disc_evaluation(){
  Cloud corners;
  stringstream ss;
  calibrator.eval_projection_matrix_disc(corners, ss, &area_mask);

  writeToOutput(ss);

  Cloud::Ptr msg = corners.makeShared();
  msg->header.frame_id = "/openni_rgb_optical_frame";
  msg->header.stamp = ros::Time::now();
  pub_eval_marker.publish(msg);

  Q_EMIT update_projector_image();
 }

 void
 QNode::eval_projection()
 {

  Cloud corners;
  stringstream ss;

  calibrator.eval_projection_matrix_Checkerboard(corners, ss, &area_mask);


  writeToOutput(ss);

  Cloud::Ptr msg = corners.makeShared();
  msg->header.frame_id = "/openni_rgb_optical_frame";
  msg->header.stamp = ros::Time::now();
  pub_eval_marker.publish(msg);

  Q_EMIT update_projector_image();

 }

 bool
 QNode::init()
 {
  ros::init(init_argc, init_argv, "projector_calibration");
  if (!ros::master::check()){
   return false;
  }

  ros::start();
  ros::NodeHandle n;

  user_input = new User_Input();
  user_input->init();

  train_frame_cnt = 1;
  train_background = true;


  foreGroundVisualizationActive = false;
  modeler_cell_size = 1/100.0; // given in m


  mesh_visualizer = Mesh_visualizer(n);
  restart_modeler = true;

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

 bool QNode::init(const std::string &master_url, const std::string &host_url)
 {
  std::map<std::string, std::string> remappings;
  remappings["__master"] = master_url;
  remappings["__hostname"] = host_url;
  ros::init(remappings, "projector_calibration");
  if (!ros::master::check())
   {
   return false;
   }
  ros::start();
  ros::NodeHandle n;

  start();
  return true;
 }

 void QNode::imgCloudCB(const sensor_msgs::ImageConstPtr& img_ptr,
   const sensor_msgs::PointCloud2ConstPtr& cloud_ptr)
 {

  ros::Time now_callback = ros::Time::now();




  pcl::fromROSMsg(*cloud_ptr, current_cloud);
  cv_ptr = cv_bridge::toCvCopy(img_ptr, sensor_msgs::image_encodings::BGR8);


  current_col_img = cv_ptr->image;
  calibrator.setInputImage(current_col_img);

  if (current_cloud.size() == 0)
   return;

  calibrator.setInputCloud(current_cloud);

  if (calibrator.isKinectTrafoSet() && pub_cloud_worldsystem.getNumSubscribers() > 0)
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

   if (modeler.getTrainingCnt() == 0){
    float lx = 0.30;
    float ly = 0.20;

    //    modeler_cell_size = 0.0033;
    // modeler_cell_size = 0.01;
    modeler.init(modeler_cell_size, -lx,lx,-ly,ly);
   }


   ros::Time start_train = ros::Time::now();
   uint cnt = modeler.addTrainingFrame(calibrator.cloud_moved);
   ROS_INFO("Adding Frame: %f ms", (ros::Time::now()-start_train).toSec()*1000.0);

   // uint cnt = detector.addTrainingFrame(calibrator.input_cloud);
   // ROS_INFO("Background calibration: added traingframe %i of %i", cnt, train_frame_cnt);
   if (cnt == train_frame_cnt)
    {
    //detector.computeBackground(0.007);
    //    train_background = false;
    //    ROS_INFO("computed Background");
    //
    //    cv::imwrite("area.jpg",area_mask);
    //    cv::imwrite("m1.jpg",detector.mask);
    //
    //    if (area_mask.cols == current_col_img.cols){
    //     detector.applyMask(area_mask);
    //    }
    //
    //    cv::imwrite("m2.jpg",detector.mask);
    //
    //
    //    Cloud foo = detector.showBackground(calibrator.input_cloud);
    //    Cloud::Ptr msg = foo.makeShared();
    //    msg->header.frame_id = "/openni_rgb_optical_frame";
    //    msg->header.stamp = ros::Time::now();
    //    pub_background.publish(msg);

    // compute background method 2:
    ros::Time now = ros::Time::now();
    modeler.computeModel();
    ROS_INFO("Compute Model: %f ms", (ros::Time::now()-now).toSec()*1000.0);



    //    Cloud model = modeler.getModel();
    //    Cloud::Ptr msg = model.makeShared();
    //    msg->header.frame_id = "/openni_rgb_optical_frame";
    //    msg->header.stamp = ros::Time::now();
    //    pub_model.publish(msg);

    Q_EMIT model_computed();

    }
   }



  if (foreGroundVisualizationActive && detector.isInitiated())
   {
   // ROS_INFO("foreground visulization active");

   ROS_INFO("RANGE from %f to %f", min_dist, visual_z_max);

   Cloud changed =
     detector.removeBackground(calibrator.input_cloud, min_dist, visual_z_max);
   ROS_INFO("background: min %f, max: %f",  min_dist, visual_z_max);

   Cloud trafoed;
   pcl::getTransformedPointCloud(changed,calibrator.getCameraTransformation(),trafoed);


   Cloud::Ptr msg = trafoed.makeShared();
   msg->header.frame_id = "/openni_rgb_optical_frame";
   msg->header.stamp = ros::Time::now();
   pub_foreground.publish(msg);

   ROS_INFO("color_range: %f", color_range);

   projectCloudIntoImage(trafoed, calibrator.proj_Matrix,
     calibrator.projector_image, -1,-100,  color_range); // -1,-100: all values are accepted

   //   cv::dilate(calibrator.projector_image,calibrator.projector_image,cv::Mat(),cv::Point(-1,-1),2);


   //   imwrite("fg.jpg",*detector.getForeground());

   ROS_INFO("Publishing foreground with %zu points", changed.size());

   //   Cloud::Ptr msg = changed.makeShared();
   //   msg->header.frame_id = "/openni_rgb_optical_frame";
   //   msg->header.stamp = ros::Time::now();
   //   pub_colored_cloud.publish(msg);

   //   // background method 2:
   //   cv::Mat fg_2;
   //   modeler.getForeground(calibrator.cloud_moved, 0.1, fg_2);
   //   cv::imwrite("modeler.jpg", fg_2);

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

   ROS_INFO("color_range: %f", color_range);
   projectCloudIntoImage(calibrator.cloud_moved, calibrator.proj_Matrix, calibrator.projector_image, visual_z_max, 0, color_range);


   foo();
   Cloud colored = colorizeCloud(calibrator.cloud_moved,0, visual_z_max,color_range);
   Cloud::Ptr msg = colored.makeShared();
   msg->header.frame_id = "/openni_rgb_optical_frame";
   msg->header.stamp = ros::Time::now();
   pub_colored_cloud.publish(msg);

   Q_EMIT update_projector_image();
   }



  Q_EMIT received_col_Image();


  ROS_INFO("FULL Callback: %f ms", (ros::Time::now()-now_callback).toSec()*1000.0);


 }


 void QNode::imageCallback(const sensor_msgs::ImageConstPtr& msg)
 {

  ROS_INFO("got image");

  ros::Time cb_start = ros::Time::now();


  // < 1 ms
  cv_bridge::CvImagePtr depth_ptr = cv_bridge::toCvCopy(msg);
  cv::Mat * img = &depth_ptr->image;




  // TODO: apply bilinear filtering on image

  // create pointcloud

  float fx = 525.0;  // focal length x
  float fy = 525.0;  // focal length y
  float cx = 319.5;  // optical center x
  float cy = 239.5;  // optical center y

  Cloud cloud;
  cloud.points.resize(img->cols*img->rows);



  ros::Time start_creation = ros::Time::now();

  // 3 ms for creation of cloud
  int pos = 0;
  for (int y = 0; y< img->rows; ++y)
   for (int x = 0; x< img->cols; ++x){
    float d = img->at<float>(y,x);

    pcl_Point p;
    p.z = d;
    p.x = (x - cx) * p.z / fx;
    p.y = (y - cy) * p.z / fy;

    // cloud.push_back(p);

    cloud.points[pos++] = p;
   }

  cloud.width = img->cols;
  cloud.height = img->rows;

  ROS_INFO("create cloud: %f ms", (ros::Time::now()-start_creation).toSec()*1000.0);

  ros::Time start_set = ros::Time::now();
  calibrator.setInputCloud(cloud);
  //  ROS_INFO("setting and transforming cloud: %f ms", (ros::Time::now()-start_set).toSec()*1000.0);

  //   if (calibrator.isKinectTrafoSet() && train_background)
  {

   if (restart_modeler){
    restart_modeler = false;
    float lx = 0.30;
    float ly = 0.20;

    ros::Time start_init = ros::Time::now();
    modeler.init(modeler_cell_size, -lx,lx,-ly,ly);
    //     ROS_INFO("init modeller: %f ms", (ros::Time::now()-start_init).toSec()*1000.0);

   }


   ros::Time start_train = ros::Time::now();
   // uint cnt = modeler.addTrainingFrame(calibrator.cloud_moved);
   modeler.updateHeight(calibrator.cloud_moved);
   ROS_INFO("Frame Update: %f ms", (ros::Time::now()-start_train).toSec()*1000.0);




   // uint cnt = detector.addTrainingFrame(calibrator.input_cloud);
   // ROS_INFO("Background calibration: added traingframe %i of %i", cnt, train_frame_cnt);
   // if (cnt >= train_frame_cnt)
   {
    //detector.computeBackground(0.007);
    //    train_background = false;
    //    ROS_INFO("computed Background");
    //
    //    cv::imwrite("area.jpg",area_mask);
    //    cv::imwrite("m1.jpg",detector.mask);
    //
    //
    //    if (area_mask.cols == current_col_img.cols){
    //     detector.applyMask(area_mask);
    //    }
    //
    //    cv::imwrite("m2.jpg",detector.mask);
    //
    //
    //    Cloud foo = detector.showBackground(calibrator.input_cloud);
    //    Cloud::Ptr msg = foo.makeShared();
    //    msg->header.frame_id = "/openni_rgb_optical_frame";
    //    msg->header.stamp = ros::Time::now();
    //    pub_background.publish(msg);

    // compute background method 2:
    //ros::Time model_start = ros::Time::now();
    //modeler.computeModel();
    //  ROS_INFO("Compute Model: %f ms", (ros::Time::now()-model_start).toSec()*1000.0);

    //    Cloud model = modeler.getModel();
    //    Cloud::Ptr msg = model.makeShared();
    //    msg->header.frame_id = "/openni_rgb_optical_frame";
    //    msg->header.stamp = ros::Time::now();
    //    pub_model.publish(msg);

    Q_EMIT model_computed();
   }
  }

  //    ROS_INFO("Callback time: %f ms", (ros::Time::now()-cb_start).toSec()*1000.0);
 }

 void QNode::paramCallback(const projector_calibration::visualization_paramsConfig& config, uint32_t level){
  ROS_INFO("new config: %f cm, hist: %i", config.cell_length_cm, config.hist_length);
  modeler_cell_size = config.cell_length_cm/100.0;
  train_frame_cnt = config.hist_length;
  modeler.weight = config.update_weight;
  restart_modeler = true;
 }

 void
 QNode::run()
 {
  ros::Rate loop_rate(40);


  ROS_INFO("Starting to run");

  ros::NodeHandle nh;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
  sensor_msgs::PointCloud2> policy;
  message_filters::Subscriber<sensor_msgs::Image> image_sub(nh,
    "/camera/rgb/image_color", 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh,
    "/camera/rgb/points", 1);
  message_filters::Synchronizer<policy> sync(policy(2), image_sub, cloud_sub);
  //  sync.registerCallback(boost::bind(&QNode::imgCloudCB, this, _1, _2));




  dynamic_reconfigure::Server<projector_calibration::visualization_paramsConfig> srv;
  dynamic_reconfigure::Server<projector_calibration::visualization_paramsConfig>::CallbackType f;
  f = boost::bind(&QNode::paramCallback,this, _1, _2);
  srv.setCallback(f);




  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("camera/depth/image", 10, boost::bind(&QNode::imageCallback, this, _1));

  // TODO: subscribe to camera info

  pub_cloud_worldsystem = nh.advertise<Cloud> ("cloud_world", 1);
  pub_3d_calib_points = nh.advertise<Cloud> ("calibration_points_3d", 1);
  pub_colored_cloud = nh.advertise<Cloud> ("colored", 1);
  pub_eval_marker = nh.advertise<Cloud> ("disc_center", 1);
  pub_background = nh.advertise<Cloud> ("background", 1);
  pub_model = nh.advertise<Cloud> ("surface_model", 1);
  pub_foreground  = nh.advertise<Cloud> ("foreground", 1);
  //  ros::Subscriber sub = n.subscribe("chatter", 1000, &Listener::callback, &listener);



  ros::spin();

  //  while (ros::ok())
  //   {
  //   ros::spinOnce();
  //   loop_rate.sleep();
  //   }
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
