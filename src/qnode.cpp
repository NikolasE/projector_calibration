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
  init_argc(argc),
  init_argv(argv)
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
  // wait();
}

void QNode::disc_evaluation(){
  Cloud corners;
  stringstream ss;
  bool success = calibrator.eval_projection_matrix_disc(corners, ss, &area_mask);

  writeToOutput(ss);

  if (success && pub_eval_marker.getNumSubscribers() > 0){
    Cloud::Ptr msg = corners.makeShared();
    msg->header.frame_id = "/fixed_frame";
    msg->header.stamp = ros::Time::now();
    pub_eval_marker.publish(msg);
  }

  Q_EMIT update_projector_image();
}

void
QNode::eval_projection()
{

  Cloud corners;
  stringstream ss;

  calibrator.eval_projection_matrix_Checkerboard(corners, ss);


  writeToOutput(ss);

  if (pub_eval_marker.getNumSubscribers() > 0){
    Cloud::Ptr msg = corners.makeShared();
    msg->header.frame_id = "/fixed_frame";
    msg->header.stamp = ros::Time::now();
    pub_eval_marker.publish(msg);
  }

  Q_EMIT update_projector_image();

}

bool QNode::init()
{
  ros::init(init_argc, init_argv, "projector_calibration");
  if (!ros::master::check()){
    return false;
  }

  ros::start();
  ros::NodeHandle nh;


  if (calibrator.load_everything_on_startup){
    stringstream msg;
    calibrator.initFromFile(msg);
    writeToOutput(msg);
    //    if (qnode.calibrator.isKinectTrafoSet()){
    //      activateSlider();
    //    }
  }

  ant_walk_distance = 10;

  objectMask = cv::Mat(480,640,CV_8UC1);
  pixel_modeler.init(480,640,10);

  if (calibrator.mask_valid()){
    ROS_INFO("calibrator mask: %i %i",calibrator.mask.cols,calibrator.mask.rows );
    area_mask = calibrator.mask;
    detector.setDetectionArea(calibrator.mask);
    pixel_modeler.setMask(calibrator.mask);
  }

  //train_frame_cnt = 1;
  train_background = true;

  planner_max_dist = 100;
  frame_cnt = 0;

  planner_scaling_factor = 1;

  // foreGroundVisualizationActive = false;
  modeler_cell_size = 1/100.0; // given in m

  water_request_id = 0;
  mesh_visualizer = Mesh_visualizer(nh);
  restart_modeler = true;

  max_update_dist = 0.1;

  current_frame_static = false;
  time_of_last_static_frame = ros::TIME_MIN; // earlier than everything

  update_pixel_model = false;
  update_elevation_map = false;

  next_ant_id = 0;
  height_line_distance = 0.05;

  start();
  return true;
}

void QNode::writeToOutput(const std::stringstream& msg)
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

void QNode::saveParameters(){
  cv::FileStorage fs("data/settings.yml", cv::FileStorage::WRITE);

  fs << "brightness_threshold" << calibrator.eval_brightness_threshold;
  fs << "color_range" << color_range;
  fs << "min_dist" << min_dist;
  //  fs << "max_dist" << max_dist;
  // fs << "openGL_visualizationActive" << openGL_visualizationActive;
  fs << "depth_visualization_active" << depth_visualization_active;
  fs << "show_texture" << show_texture;
  fs << "water_simulation_active" << water_simulation_active;
  fs << "show_height_lines" << show_height_lines;
  fs << "calibration_active" << calibration_active;
  fs << "update_pixel_model" << update_pixel_model;
  fs << "do_gesture_recognition" << do_gesture_recognition;
  fs << "update_elevation_map" << update_elevation_map;
  fs << "height_line_distance" << height_line_distance;
}

bool QNode::loadParameters(){

  cv::FileStorage fs("data/settings.yml", cv::FileStorage::READ);

  if (!fs.isOpened()) {
    ROS_WARN("Could not read data/settings.yml");
    return false;
  }

  calibrator.eval_brightness_threshold = (int)fs["brightness_threshold"];

  color_range = (float)fs["color_range"];
  min_dist = (float)fs["min_dist"];
  //  max_dist = (float)fs["max_dist"];
  height_line_distance = (float)fs["height_line_distance"];
  // openGL_visualizationActive = (int)fs["openGL_visualizationActive"];
  depth_visualization_active = (int)fs["depth_visualization_active"];
  show_texture = (int)fs["show_texture"];
  water_simulation_active  = (int)fs["water_simulation_active"];
  show_height_lines  = (int)fs["show_height_lines"];
  calibration_active = (int) fs["calibration_active"];


  update_pixel_model = (int) fs["update_pixel_model"];
  do_gesture_recognition = (int) fs["do_gesture_recognition"];
  update_elevation_map = (int) fs["update_elevation_map"];


  return true;
}

void QNode::depthCamInfoCB(const sensor_msgs::CameraInfoConstPtr& cam_info){
  if (!calibrator.depth_cam_model_set){
    ROS_INFO("Got pinhole model for depth camera");
    calibrator.setDepthCameraModel(cam_info);
  }
}

void QNode::runDetector(){


  timing_start("runDetector");


  if (!detector.detectionAreaSet()){
    ROS_WARN("NO DETECTION AREA SET!");
    return;
  }


  // ROS_INFO("Update cnt: %i", dadate_cnt);

  if (pixel_modeler.update_cnt < 30)
    return;

  timing_start("fg_pixel");
  // 4ms
  cv::Mat dists1;
  pixel_modeler.getForeground_dist(current_cloud,min_dist,pixel_foreground,&dists1,2);
  timing_end("fg_pixel");

  //  cv::Mat fg_step2, dists2;
  //  pixel_modeler.getForeground_dist(current_cloud,min_dist,fg_step2,&dists2,2);
  //  if (pub_detection_foreground2.getNumSubscribers()){
  //    cv_bridge::CvImage out_msg;
  //    out_msg.header.stamp   = ros::Time::now();
  //    out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  //    cv::Mat n = normalize_mat(dists2);
  //    out_msg.image    = n;
  //    pub_detection_foreground2.publish(out_msg.toImageMsg());
  //  }

  timing_start("pub");

  if (pub_detection_foreground.getNumSubscribers()){
    cv_bridge::CvImage out_msg;
    out_msg.header.stamp   = ros::Time::now();
    out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    cv::Mat n = normalize_mat(dists1);
    out_msg.image    = n;
    pub_detection_foreground.publish(out_msg.toImageMsg());
  }



  //  timing_end("fg_pixel");

  if (pixel_foreground.cols == 0){
    return;
  }



  if (pub_pixel_model.getNumSubscribers() > 0){
    Cloud model;
    pixel_modeler.getCloudRepresentation(current_cloud,model,1);
    Cloud::Ptr msg = model.makeShared();
    msg->header.frame_id = "/fixed_frame";
    msg->header.stamp = ros::Time::now();
    pub_pixel_model.publish(msg);
  }

  timing_end("pub");

  // ROS_INFO("fg: %i %i  norm: %i %i", pixel_foreground.cols, pixel_foreground.cols, current_norm.cols, current_norm.rows);

  //  cv::Mat dists;
  //  pixel_modeler.getDistance(calibrator.input_cloud,dists);

  // creates depth-map for all pixels within masked region
  // (does not depend on current estimated model)
  //  pixel_modeler.getNorm(current_cloud,current_norm);

  //  cv::Mat mean;
  //  pixel_modeler.getMeans(mean);


  //  cv::Mat dists = mean-current_norm;
  //  double m,x;
  //  cv::minMaxLoc(dists,&m,&x);
  //  ROS_INFO("dists: %f %f",m,x);

  bool hand_visible_in_last_frame = detector.handVisibleInCurrentFrame();

  timing_start("newFrame");
  detector.newFrame(pixel_foreground,dists1, &calibrator.input_cloud);
  timing_end("newFrame");

  timing_start("analyse");
  detector.analyseScene();
  timing_end("analyse");

  Q_EMIT sig_handvisible(detector.handVisibleInCurrentFrame());


  bool hand_disappeared = hand_visible_in_last_frame && !detector.handVisibleInCurrentFrame();


  if (hand_disappeared)
    ROS_INFO("HXXXXXX HAND IS NO LONGER VISIBLE");


  timing_start("tracking");
  grasp_tracker.update_tracks(detector.grasp_detections,true);

  // don't start new tracks while the elevation map can change. Otherwise moved sand could be accepted as new object
  fingertip_tracker.update_tracks(detector.finger_detections, !update_elevation_map);

  // only update objects if hand was not visible (since objects could be occluded)
  if ( !detector.handVisibleInCurrentFrame()){
    piece_tracker.update_tracks(detector.object_detections, !update_elevation_map);
  }
  timing_end("tracking");

  timing_end("runDetector");


  // visualize tracks on Kinect's RGB Image
  Cloud grasps;
  for (GraspTrack_it it = grasp_tracker.tracks.begin(); it != grasp_tracker.tracks.end(); ++it){
    if (it->second.state == Track_Active){
      it->second.visualizeOnImage(current_col_img,getColor(it->first));
      pcl_Point center = it->second.last_detection()->position_world;
      grasps.push_back(center);
      //      ROS_INFO("Grasp Track (%i) at: %f %f %f", it->first, center.x,center.y, center.z);
    }
  }

  if (pub_hand.getNumSubscribers()){
    Cloud::Ptr msg = grasps.makeShared();
    msg->header.frame_id = "fixed_frame";
    msg->header.stamp = ros::Time::now();
    pub_hand.publish(msg);
  }


  for (PieceTrack_it it = piece_tracker.tracks.begin(); it != piece_tracker.tracks.end(); ++it){
    if (it->second.state == Track_Active){
      it->second.visualizeOnImage(current_col_img,getColor(it->first));
      // pcl_Point center = it->second.last_detection()->position_world;
      // ROS_INFO("Piece Track (%i) at: %f %f %f", it->first, center.x,center.y, center.z);
    }
  }


  for (FingerTrack_it it = fingertip_tracker.tracks.begin(); it != fingertip_tracker.tracks.end(); ++it){
    // if (it->second.state == Track_Active)
    it->second.visualizeOnImage(current_col_img,getColor(it->first),TT_FINGERTIP);
    //    pcl_Point center = it->second.last_detection()->position_world;
    //    ROS_INFO("Finger Track (%i) at: %f %f %f",it->first, center.x,center.y, center.z);
  }



  Q_EMIT received_col_Image(); // visualisation of Kinect-Color-Image

}


void QNode::toggle_gesture_recognition(bool do_recognition){
  do_gesture_recognition = do_recognition;

  //  if (do_recognition){
  //    ROS_INFO("resetting  tracks");
  piece_tracker.reset();
  grasp_tracker.reset();
  fingertip_tracker.reset();
  //  }

}

void QNode::visualizePlanner(QPixmap* img){

  // ROS_INFO("Showing paths");
  if (!with_path_planning)
    return;

  // simple mutex: (otherwise callback updates height map)
  with_path_planning = false;


  timing_start("planner");

  Cloud model = elevation_map.getModel();

  if (pub_ant_model.getNumSubscribers()){
    // ROS_INFO("sending ant model %zu",planner.model.size());
    Cloud::Ptr msg = planner.model.makeShared();
    msg->header.frame_id = "/fixed_frame";
    msg->header.stamp = ros::Time::now ();
    pub_ant_model.publish(msg);
  }

  //  if (planner.height_map.cols != int(model.width)){
  //    ROS_WARN("PLANNER HAS WRONG SIZE (%i %i)", planner.height_map.cols, model.width);
  //    with_path_planning = true;
  //    return;
  //  }

  QPainter painter(img);

  //  if (piece_tracker.tracks.size() < 2)
  //    continue;

  //  // use first and second piece as start and end position of path
  //  PieceTrack_it it = piece_tracker.tracks.begin();
  //  pcl_Point start = it->second.last_detection()->position_world;
  //  it++;
  //  pcl_Point end = it->second.last_detection()->position_world;

  //  cv::Scalar color =  getColor(it->first);



  //  cv::Point s = elevation_map.grid_pos(end); // walk from current position to start of grasp
  //  cv::Point goal = elevation_map.grid_pos(start);

  cv::Point s = cv::Point(0,0);
  cv::Point goal = cv::Point(elevation_map.mean.cols*0.7,elevation_map.mean.rows*0.7);

  ROS_INFO("Computing path from %i %i to %i %i", s.x,s.y,goal.x,goal.y);


  // planner.printParameters();

  // todo: don't do every time
  planner.computePolicy(goal);
  planner.computePath(s);


  std::vector<cv::Point> contour;
  painter.setPen(QPen(QColor(0,0,255),10));
  planner.getRangeOfMotion(planner_max_dist,contour);

  visualization_msgs::Marker marker;

  if (pub_ant_range.getNumSubscribers()){
    planner.createRangeMarker(marker, contour);
    pub_ant_range.publish(marker);
  }

  uint N = contour.size();
  if (N>0){
    for (uint i=0; i<N; ++i){
      pcl_Point P1 = model.at(contour[i].x,contour[i].y);
      cv::Point2f px1 = applyPerspectiveTrafo(P1, calibrator.proj_Matrix());

      pcl_Point P2 = model.at(contour[(i+1)%N].x,contour[(i+1)%N].y);
      cv::Point2f px2= applyPerspectiveTrafo(P2, calibrator.proj_Matrix());

      painter.drawLine(QPoint(px1.x,px1.y),QPoint(px2.x,px2.y));
    }
  }


  //    int w = model.width;
  //    int h = model.height;
  //    planner.computePolicy(cv::Point(w-10,h-10));
  //    planner.computePath(cv::Point(10,10));

  painter.setPen(QPen(QColor(255,0,0),10));
  std::vector<cv::Point> path = planner.getPath();
  ROS_INFO("Visualizing path with %zu entries", path.size());
  for (uint i=0; i<path.size()-1; ++i){
    cv::Point p1 = path[i];
    pcl_Point P1 = model.at(p1.x,p1.y);

    cv::Point p2 = path[i+1];
    pcl_Point P2 = model.at(p2.x,p2.y);

    cv::Point2f px1 = applyPerspectiveTrafo(P1, calibrator.proj_Matrix());
    cv::Point2f px2 = applyPerspectiveTrafo(P2, calibrator.proj_Matrix());
    //  ROS_INFO("pixel pos: %f %f", px1.x,px1.y);
    painter.drawLine(QPoint(px1.x,px1.y),QPoint(px2.x,px2.y));
  }


  timing_end("planner");


  if (pub_path.getNumSubscribers()){
    //ROS_INFO("Sending ath markar");
    // show in RVIZ
    visualization_msgs::Marker marker;
    planner.createPathMarker(marker);
    pub_path.publish(marker);
  }




  with_path_planning = true;

}

void QNode::visualizeTracks(QPixmap* img){

  for (PieceTrack_it it = piece_tracker.tracks.begin(); it != piece_tracker.tracks.end(); ++it){
    // if (it->second.state == Track_Active)
    //it->second.visualizeOnImage(current_col_img,getColor(it->first),TT_FINGERTIP);
    pcl_Point center = it->second.last_detection()->position_world;
    cv::Point2f px = applyPerspectiveTrafo(center, calibrator.proj_Matrix());

    QPainter painter(img);
    cv::Scalar color =  getColor(it->first);
    painter.setBrush(QBrush(QColor(color.val[0],color.val[1],color.val[2])));
    painter.setPen(QPen(QColor(color.val[0],color.val[1],color.val[2]),20));
    int size = 50;
    painter.drawEllipse(QPoint(px.x,px.y),size,size);

    uint n = it->second.detections.size();

    for (uint i=0; i<n-1; ++i){
      cv::Point2f px1 = applyPerspectiveTrafo(it->second.detections[i].position_world, calibrator.proj_Matrix());
      cv::Point2f px2 = applyPerspectiveTrafo(it->second.detections[i+1].position_world, calibrator.proj_Matrix());
      painter.drawLine(QPoint(px1.x,px1.y),QPoint(px2.x,px2.y));
    }


    // ROS_INFO("Visualizing Object at: %f %f %f (visualized at %f %f)", center.x,center.y, center.z, px.x,px.y);
  }


  for (FingerTrack_it it = fingertip_tracker.tracks.begin(); it != fingertip_tracker.tracks.end(); ++it){
    // if (it->second.state == Track_Active)
    //it->second.visualizeOnImage(current_col_img,getColor(it->first),TT_FINGERTIP);
    pcl_Point center = it->second.last_detection()->position_world;
    cv::Point2f px = applyPerspectiveTrafo(center, calibrator.proj_Matrix());

    QPainter painter(img);
    cv::Scalar color =  getColor(it->first);
    painter.setBrush(QBrush(QColor(color.val[0],color.val[1],color.val[2])));
    painter.setPen(QPen(QColor(color.val[0],color.val[1],color.val[2]),20));
    int size = 50;
    painter.drawEllipse(QPoint(px.x,px.y),size,size);

    uint n = it->second.detections.size();

    for (uint i=0; i<n-1; ++i){
      cv::Point2f px1 = applyPerspectiveTrafo(it->second.detections[i].position_world, calibrator.proj_Matrix());
      cv::Point2f px2 = applyPerspectiveTrafo(it->second.detections[i+1].position_world, calibrator.proj_Matrix());
      painter.drawLine(QPoint(px1.x,px1.y),QPoint(px2.x,px2.y));
    }


    //ROS_INFO("Visualizing Object at: %f %f %f (visualized at %f %f)", center.x,center.y, center.z, px.x,px.y);
  }


  for (GraspTrack_it it = grasp_tracker.tracks.begin(); it != grasp_tracker.tracks.end(); ++it){
    // if (it->second.state == Track_Active)
    //it->second.visualizeOnImage(current_col_img,getColor(it->first),TT_FINGERTIP);
    pcl_Point center = it->second.last_detection()->position_world;
    cv::Point2f px = applyPerspectiveTrafo(center, calibrator.proj_Matrix());

    QPainter painter(img);
    cv::Scalar color =  getColor(it->first);
    // painter.setBrush(QBrush(QColor(color.val[0],color.val[1],color.val[2])));
    painter.setPen(QPen(QColor(color.val[0],color.val[1],color.val[2]),30));
    int size = 100;
    painter.drawEllipse(QPoint(px.x,px.y),size,size);

    // ROS_INFO("Visualizing Object at: %f %f %f (visualized at %f %f)", center.x,center.y, center.z, px.x,px.y);
  }


}

void QNode::cloudCB(const sensor_msgs::PointCloud2ConstPtr& cloud_ptr){

  if (with_path_planning){
    move_ants();
  }


  if (calibration_active)
    return;

  timing_start("CloudCb");

  pcl::fromROSMsg(*cloud_ptr, current_cloud);
  calibrator.setInputCloud(current_cloud);


  if (calibrator.isKinectTrafoSet()){

    detector.setTransformation(calibrator.kinect_trafo);
    calibrator.publishWorldFrame("/openni_rgb_optical_frame","/fixed_frame");
    calibrator.publishProjectorFrame("/openni_rgb_optical_frame","/projector_frame");
    if(pub_cloud_worldsystem.getNumSubscribers() > 0){
      Cloud::Ptr msg = calibrator.cloud_moved.makeShared();
      msg->header.frame_id = "/fixed_frame";
      msg->header.stamp = ros::Time::now();
      pub_cloud_worldsystem.publish(msg);
    }


  }else{
    timing_end("CloudCb");
    calibration_active = true;
    return;
  }



  if (!calibrator.projMatrixSet())
    return;

  if (do_gesture_recognition){ runDetector(); }

  if (piece_tracker.tracks.size() == 0){
    elevation_map.unLockCells();
    objectMask.setTo(0);
  }
  else{
    //    ROS_INFO("Locking cells! (%zu objects)", piece_tracker.tracks.size() );
    drawObjectContours(piece_tracker,objectMask);
    //    cv::namedWindow("locked");
    //    cv::imshow("locked",objectMask);
  }

  if (update_pixel_model){
    if (detector.handVisibleInCurrentFrame()){
      ROS_INFO("Hand visible, pixel model is not updated!");
    }else{
      if (do_gesture_recognition)
        pixel_modeler.update(current_cloud,&objectMask,2);
      else
        pixel_modeler.update(current_cloud,NULL,2);
    }
  }

  if (update_elevation_map && !detector.handVisibleInCurrentFrame()){
    timing_start("up_model");

    if (piece_tracker.tracks.size() > 0){
      elevation_map.lockCells(objectMask,calibrator.cloud_moved);
    }

    elevation_map.updateHeight(calibrator.cloud_moved,0.1);

//    if (with_path_planning){
//      // ROS_INFO("QNode: updating planner with cellsize %f", elevation_map.cell_size());
//      planner.setHeightMap(elevation_map.mean, elevation_map.cell_size());

//      if (pub_ant_model.getNumSubscribers()){
//        ROS_INFO("sending ant model %zu",planner.model.size());
//        Cloud::Ptr msg = planner.model.makeShared();
//        msg->header.frame_id = "/fixed_frame";
//        msg->header.stamp = ros::Time::now ();
//        pub_ant_model.publish(msg);
//      }

//    }

    if (pub_model.getNumSubscribers()){
      Cloud model = elevation_map.getModel();
      Cloud::Ptr msg = model.makeShared();
      msg->header.frame_id = "/fixed_frame";
      msg->header.stamp = ros::Time::now();
      pub_model.publish(msg);
    }

    timing_end("up_model");

  }

  if (water_simulation_active){
    if (restart_modeler){
      init_watersimulation();
      restart_modeler = false;
    }
    else{
      timing_start("water");
      step_watersimulation();
      timing_end("water");
#ifdef PRINT_TIMING
      ROS_INFO("Water simulation: %f ms", (ros::Time::now()-start_water).toSec()*1000.0);
#endif

    }
  }




  Q_EMIT model_computed();

  timing_end("CloudCb");
}

void QNode::imgCloudCB(const sensor_msgs::ImageConstPtr& img_ptr, const sensor_msgs::PointCloud2ConstPtr& cloud_ptr)
{

  if (!calibration_active) { return; }


  timing_start("callback");
  pcl::fromROSMsg(*cloud_ptr, current_cloud);
  cv_ptr = cv_bridge::toCvCopy(img_ptr, sensor_msgs::image_encodings::BGR8);

  current_col_img = cv_ptr->image;

  Q_EMIT received_col_Image();

  // timing_start("input");
  calibrator.setInputCloud(current_cloud);
  calibrator.setInputImage(current_col_img,&area_mask);
  // timing_end("input");



  if (calibrator.isKinectTrafoSet()){
    detector.setTransformation(calibrator.kinect_trafo);
    calibrator.publishWorldFrame("/openni_rgb_optical_frame","/fixed_frame");
    if(pub_cloud_worldsystem.getNumSubscribers() > 0){
      Cloud::Ptr msg = calibrator.cloud_moved.makeShared();
      msg->header.frame_id = "/fixed_frame";
      msg->header.stamp = ros::Time::now();
      pub_cloud_worldsystem.publish(msg);
    }
  }

  return;


  //  //  timing_start("fg_model");
  //  // cv::Mat fg = elevation_map.getFGMask(calibrator.cloud_moved, min_dist);
  //  //  timing_end("fg_model");

  //  // current_col_img = visualizeMask(current_col_img,pixel_foreground);

  //  //    cv::erode(foreground,foreground,cv::Mat(),cv::Point(-1,-1),2);
  //  //    cv::dilate(foreground,foreground,cv::Mat(),cv::Point(-1,-1),2);


  //  //  cv_bridge::CvImage out_msg;
  //  //  out_msg.header   = img_ptr->header;
  //  //  out_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC1;

  //  //  out_msg.image    = pixel_foreground;
  //  //  pub_gauss_foreground.publish(out_msg.toImageMsg());


  //  //  ROS_INFO("Updating");

  //  //  cv::namedWindow("mean");
  //  //  cv::imshow("mean",modeler.mean);

  //  //  cv::namedWindow("gaussian");
  //  //  cv::Mat mean_g;
  //  //  modeler.getGaussianMean(mean_g);
  //  //  cv::imshow("gaussian",mean_g);





  //  // Q_EMIT received_col_Image();

  //  return;

  //#ifdef PRINT_TIMING
  //  ROS_INFO("getFGMask: %f ms", (ros::Time::now()-start_fg).toSec()*1000.0);
  //#endif



  //  ros::Time start_object_detection = ros::Time::now();

  //#ifdef PRINT_TIMING
  //  ROS_INFO("detectPlayingPieces: %f ms", (ros::Time::now()-start_object_detection).toSec()*1000.0);
  //#endif



  //  if (current_cloud.size() == 0)
  //    return;

  //  calibrator.setInputCloud(current_cloud);

  //  if (calibrator.isKinectTrafoSet() && pub_cloud_worldsystem.getNumSubscribers() > 0)
  //    {
  //      Cloud::Ptr msg = calibrator.cloud_moved.makeShared();
  //      msg->header.frame_id = "/openni_rgb_optical_frame";
  //      msg->header.stamp = ros::Time::now();
  //      pub_cloud_worldsystem.publish(msg);
  //    }

  //  if (train_background && !calibrator.isKinectTrafoSet())
  //    {
  //      ROS_WARN("Kinect Transformation has to be set before background can be computed");
  //    }

  //  if (calibrator.isKinectTrafoSet() && train_background)
  //    {

  //      if (!elevation_map.is_initialized){
  //        float lx = 0.15;
  //        float ly = 0.18;
  //        //    modeler_cell_size = 0.0033;
  //        // modeler_cell_size = 0.01;
  //        elevation_map.init(modeler_cell_size, -lx,lx,-ly,ly);
  //      }

  //    }



  //  if (foreGroundVisualizationActive && detector.isInitiated())
  //    {
  //      // ROS_INFO("foreground visulization active");

  //      //   ROS_INFO("RANGE from %f to %f", min_dist, max_dist);

  //      // Cloud changed =  detector.removeBackground(calibrator.input_cloud, min_dist, max_dist);

  //      Cloud trafoed;
  //      pcl::getTransformedPointCloud(changed,calibrator.getCameraTransformation(),trafoed);


  //      Cloud::Ptr msg = trafoed.makeShared();
  //      msg->header.frame_id = "/openni_rgb_optical_frame";
  //      msg->header.stamp = ros::Time::now();
  //      pub_foreground.publish(msg);

  //      projectCloudIntoImage(trafoed, calibrator.proj_Matrix(),
  //                            calibrator.projector_image, -1,-100,  color_range); // -1,-100: all values are accepted
  //      Q_EMIT update_projector_image();
  //    }


  //  if (depth_visualization_active && calibrator.projMatrixSet())
  //    {

  //      ROS_INFO("color_range: %f", color_range);
  //      projectCloudIntoImage(calibrator.cloud_moved, calibrator.proj_Matrix(), calibrator.projector_image, max_dist, 0, color_range);


  //      Cloud colored = colorizeCloud(calibrator.cloud_moved,0, max_dist,color_range);
  //      Cloud::Ptr msg = colored.makeShared();
  //      msg->header.frame_id = "/openni_rgb_optical_frame";
  //      msg->header.stamp = ros::Time::now();
  //      pub_colored_cloud.publish(msg);

  //      Q_EMIT update_projector_image();
  //    }


  //  Q_EMIT received_col_Image();

}

/**
  *
  * @param msg rgb image from Kinect
  */
void QNode::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

  if (calibration_active) { return; }

  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  current_col_img = cv_ptr->image;

  // otherwise it is emmited at the end of run_detector!
  if (!do_gesture_recognition){
    Q_EMIT received_col_Image();
  }

  Q_EMIT process_events();


  return;





  //  ros::Time cb_start = ros::Time::now();


  //  // < 1 ms
  //  //  cv_bridge::CvImagePtr depth_ptr = cv_bridge::toCvCopy(msg);
  //  //  cv::Mat * depth = &depth_ptr->image;



  //  //  bool emit_new_image = false;


  //  //  if (water_simulation_active)
  //  //   ROS_INFO("water_simulation_active");
  //  //
  //  //  if (openGL_visualizationActive)
  //  //     ROS_INFO("openGL_visualizationActive");
  //  //
  //  //  if (show_height_lines)
  //  //     ROS_INFO("show_height_lines");


  //  if (calibrator.projMatrixSet() && (water_simulation_active  || show_height_lines)){
  //    //  if (calibrator.isKinectTrafoSet()){

  //    ros::Time start_train = ros::Time::now();
  //    bool current_frame_dynamic = elevation_map.updateHeight(calibrator.cloud_moved,max_update_dist);
  //#ifdef PRINT_TIMING
  //    ROS_INFO("Frame Update: %f ms", (ros::Time::now()-start_train).toSec()*1000.0);
  //#endif

  //    //   ros::Time start_obj = ros::Time::now();
  //    //   modeler.saveAsObj("foo.obj");
  //    //#ifdef PRINT_TIMING
  //    //   ROS_INFO("Creating Object file: %f ms", (ros::Time::now()-start_obj).toSec()*1000.0);
  //    //#endif

  //    //   if (current_frame_dynamic)
  //    //    ROS_INFO("dynamic scene");


  //    if (current_frame_dynamic){
  //      duration_since_last_static_frame = msg->header.stamp - time_of_last_static_frame;
  //    }else{
  //      time_of_last_static_frame = msg->header.stamp;

  //      if (duration_since_last_static_frame.toSec() > 0.5)
  //        ROS_INFO("Scene static again!");

  //      duration_since_last_static_frame = ros::Duration(0);
  //    }

  //    Q_EMIT scene_static(duration_since_last_static_frame.toSec());


  //    // emit_new_image = true;
  //  }


  //  if (water_simulation_active && calibrator.projMatrixSet()){
  //    if (restart_modeler){
  //      init_watersimulation();
  //      restart_modeler = false;
  //    }
  //    else{
  //      ros::Time start_water = ros::Time::now();
  //      step_watersimulation();
  //#ifdef PRINT_TIMING
  //      ROS_INFO("Water simulation: %f ms", (ros::Time::now()-start_water).toSec()*1000.0);
  //#endif
  //    }
  //  }


  //  //  if (emit_new_image)
  //  //    Q_EMIT model_computed();

  //#ifdef PRINT_TIMING
  //  ROS_INFO("Callback time: %f ms", (ros::Time::now()-cb_start).toSec()*1000.0);
  //#endif
}

/**
  * @return
  */
bool QNode::init_watersimulation(){


  //  ROS_INFO("Height: %i %i, viscosity: %f", land.cols, land.rows,sim_viscosity);

  cv_bridge::CvImage out_msg;
  out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  out_msg.image    = elevation_map.getHeightImage();


  water_simulation::simulator_init srv_msg;

  water_request_id++;
  srv_msg.request.id = water_request_id;
  srv_msg.request.land_img = *out_msg.toImageMsg();
  srv_msg.request.viscosity = sim_viscosity;
  srv_msg.request.add_sink_border = true;


  ros::service::waitForService("srv_simulator_init",ros::DURATION_MAX);
  ros::service::waitForService("srv_simulator_step",ros::DURATION_MAX);


  if (ros::service::call("srv_simulator_init", srv_msg)){
    ROS_INFO("simulation was initialized");
  }else{
    ROS_WARN("init failed (Simulation Service not available");
  }


  return true;

}


void QNode::update_ant(cv::Point goal){
  // getting and setting height map takes 0.01 ms


  //  cv::Mat height = elevation_map.getHeightImage();
  //  planner.setHeightMap(height);

  //  // only add water if simulation is running
  //  //  if (water.size() == height.size())
  //  //   planner.setWaterDepthMap(water);

  //  // todo: add water



  //  cv::Point start;
  //  if (ants.size() == 0){
  //    start = cv::Point(height.cols/2,height.rows/2); // ant starts at center of grid
  //    ant.setId(0);
  //    ROS_INFO("ANt is not initrialized");
  //  }else{
  //    ant = ants[ant.getId()];
  //    start = ant.getPosition();
  //    ROS_INFO("ant at pos %i ( %i %i)", ant.getPosInPath(), start.x, start.y );
  //  }

  //  ROS_INFO("UPDate: goal: %i %i", goal.x, goal.y);
  //  //planner.computePolicy(goal,modeler.getCellSize());
  //  ROS_INFO("starting at %i %i", start.x, start.y);
  //  //planner.computePath(start);

  //  //ant.setPath(planner.getPath(),planner.getPathEdges());

  //  // replaces old ant (since same ID)
  //  //  Q_EMIT newAnt(ant);

  //  // currently, only one ant is used
  //  ants[ant.getId()] = ant;


  //  Q_EMIT sl_update_ant();

}


void QNode::move_ants(){
  ROS_INFO("Moving %zu ant with dist: %f", ants.size(),ant_walk_distance);

  for (std::map<int,Ant>::iterator it = ants.begin(); it != ants.end(); ++it){
    it->second.walk(ant_walk_distance);
  }

}



void QNode::run_ant_demo(){

  Path_planner& planner_ = planner;

  ROS_INFO("New ant");

 ROS_INFO("planner, cellsize: %f",elevation_map.cell_size());

  // getting and setting height map takes 0.01 ms
  cv::Mat height = elevation_map.getHeightImage();
  planner_.setHeightMap(height,elevation_map.cell_size());

  if (pub_ant_model.getNumSubscribers()){
    ROS_INFO("sending ant model %zu",planner_.model.size());
    Cloud::Ptr msg = planner_.model.makeShared();
    msg->header.frame_id = "/fixed_frame";
    msg->header.stamp = ros::Time::now ();
    pub_ant_model.publish(msg);
  }

  // only add water if simulation is running
  if (water.size() == height.size())
    planner_.setWaterDepthMap(water);


  int w = height.cols;
  int h = height.rows;

  cv::Point goal = cv::Point(w-2,h-2);
  cv::Point start = cv::Point(1,1);


  planner_.computePolicy(goal);
  planner_.computePath(start);
  //  planner.saveHillSideCostImage();

  Ant ant;


  ant.setPath(planner_.getPath(),planner_.getPathEdges());

  ROS_INFO("new path has %zu steps",planner_.getPath().size());

  ant.setId(next_ant_id++);

  ants[ant.getId()] = ant;

//  timing_end("planner");

  if (pub_path.getNumSubscribers()){
    //ROS_INFO("Sending ath markar");
    // show in RVIZ
    visualization_msgs::Marker marker;
    planner_.createPathMarker(marker);
    pub_path.publish(marker);
  }



  // Q_EMIT newAnt(ant);
}


bool QNode::step_watersimulation(){

  water_simulation::simulator_step msg_step;
  msg_step.request.iteration_cnt = iterations_per_frame;
  msg_step.request.id = water_request_id;

  // Updating the land height
  cv_bridge::CvImage out_msg;
  out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  out_msg.image    = elevation_map.getHeightImage();
  msg_step.request.land_img = *out_msg.toImageMsg();
  msg_step.request.update_land_img = true;

  // update viscosity
  msg_step.request.update_viscosity = true;
  msg_step.request.viscosity = sim_viscosity;


  water_simulation::msg_source_sink source;
  source.height = 0.05; // 5cm of water at this position
  //  cv::Point pos = elevation_map.grid_pos(0,0); // source at origin
  //  source.x = pos.x;
  //  source.y = pos.y;
  source.radius = 3;
  source.additive = false;



  for (PieceTrack_it it = piece_tracker.tracks.begin(); it != piece_tracker.tracks.end(); ++it){
    if (it->second.state == Track_Active){
      it->second.visualizeOnImage(current_col_img,getColor(it->first));
      pcl_Point center = it->second.last_detection()->position_world;
      cv::Point pos = elevation_map.grid_pos(center);
      source.x = pos.x;
      source.y = pos.y;
      msg_step.request.sources_sinks.push_back(source);
      ROS_INFO("Source (%i) at: %f %f %f (%i %i)", it->first, center.x,center.y, center.z,pos.x,pos.y);
    }
  }

  ROS_INFO("New water iteration with %zu sources", msg_step.request.sources_sinks.size());


  if (ros::service::call("srv_simulator_step", msg_step)){

    if (!msg_step.response.valid_id){
      ROS_WARN("WATER SIMULATION: sent request with wrong id!");
      return false;
    }else{
      cv_bridge::toCvCopy(msg_step.response.water_img, sensor_msgs::image_encodings::TYPE_32FC1)->image.copyTo(water);
    }

  }

  return true;
}

/**
  *
  * @param config
  * @param level
  */
void QNode::paramCallback(const projector_calibration::visualization_paramsConfig& config, uint32_t level){
  //  ROS_INFO("new config: %f cm, hist: %i", config.cell_length_cm, config.hist_length);

  // train_frame_cnt = config.hist_length;
  elevation_map.weight = config.update_weight;

  //  ROS_INFO("cell size: %f, new size: %f", modeler_cell_size,config.cell_length_cm/100.0);

  iterations_per_frame = config.sim_iter_cnt;
  sim_viscosity = config.sim_viscosity;


  //   ROS_INFO("CONFIG>LIGHT_POS: %f",config.light_pos);
  // Q_EMIT new_light(config.light_pos);

  max_update_dist = config.max_update_dist;

  planner.setMaxSteepness(config.ant_steepness);
  planner.setHeightCostFactor(config.ant_heightcost_factor);
  planner.setUphillfactor(config.ant_uphill_factor);
  planner.setFourNeighbours(config.ant_use_four);
  planner.setSmoothing(config.ant_use_smoothing);
  planner.setHillSideCostFactor(config.ant_hillside_factor);
  planner.setPathLengthFactor(config.ant_path_length_factor);
  planner.enemy_factor = config.ant_enemy_factor;
  planner_max_dist = config.ant_dist_threshold;
  planner.setScale(config.ant_scale_factor);

  ant_walk_distance = config.ant_walk_dist;


  // ROS_INFO("Paramconfig: cell_length: %f", config.cell_length_cm);

  if (abs(modeler_cell_size - config.cell_length_cm/100.0) > 0.00001){

    modeler_cell_size = config.cell_length_cm/100.0;
    // Initializing the modeler:
    float lx = 0.15;
    float ly = 0.25;

    // ros::Time start_init = ros::Time::now();
    elevation_map.init(modeler_cell_size, -lx,lx,-ly,ly);

    restart_modeler = true;
  }

}


void
QNode::run()
{
  //  ros::Rate loop_rate(40);

  // Initializing the modeler:
  float lx = 0.20;
  float ly = 0.30;
  // ros::Time start_init = ros::Time::now();
  elevation_map.init(modeler_cell_size, -lx,lx,-ly,ly);
  //     ROS_INFO("init modeller: %f ms", (ros::Time::now()-start_init).toSec()*1000.0);


  ros::NodeHandle nh;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
      sensor_msgs::PointCloud2> policy;
  message_filters::Subscriber<sensor_msgs::Image> image_sub(nh,
                                                            "/camera/rgb/image_color", 2);
  //                                                      "/camera/rgb/image_rect_color",1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh,
                                                                  "/camera/rgb/points", 2);
  //                                                                "/camera/depth_registered/points",1);
  message_filters::Synchronizer<policy> sync(policy(5), image_sub, cloud_sub);
  sync.registerCallback(boost::bind(&QNode::imgCloudCB, this, _1, _2));


  sub_cam_info = nh.subscribe("/camera/depth/camera_info", 1, &QNode::depthCamInfoCB,this);
  sub_cloud = nh.subscribe("/camera/rgb/points",1,&QNode::cloudCB,this);
  sub_col_image = nh.subscribe("/camera/rgb/image_color", 1, &QNode::imageCallback,this);

  dynamic_reconfigure::Server<projector_calibration::visualization_paramsConfig> srv;
  dynamic_reconfigure::Server<projector_calibration::visualization_paramsConfig>::CallbackType f;
  f = boost::bind(&QNode::paramCallback,this, _1, _2);
  srv.setCallback(f);




  image_transport::ImageTransport it(nh);
  // image_transport::Subscriber sub = it.subscribe("camera/depth/image", 10, boost::bind(&QNode::imageCallback, this, _1));

  pub_cloud_worldsystem = nh.advertise<Cloud> ("cloud_world", 1);
  pub_3d_calib_points = nh.advertise<Cloud> ("calibration_points_3d", 1);
  pub_colored_cloud = nh.advertise<Cloud> ("colored", 1);
  pub_eval_marker = nh.advertise<Cloud> ("disc_center", 1);
  pub_background = nh.advertise<Cloud> ("background", 1);
  pub_model = nh.advertise<Cloud> ("surface_model", 1);
  pub_foreground  = nh.advertise<Cloud> ("foreground", 1);
  pub_hand = nh.advertise<Cloud>("grasp_detections",1);


  pub_pixel_model = nh.advertise<Cloud>("detection/model",1);


  pub_gauss_foreground = nh.advertise<sensor_msgs::Image>("gauss_foreground",1);
  // pub_surface_foreground = nh.advertise<sensor_msgs::Image>("surface_foreground",1);

  pub_projector_marker = nh.advertise<visualization_msgs::Marker>("projector_pose",2);
  pub_detection_foreground  = nh.advertise<sensor_msgs::Image>("detection/foreground",1);
  pub_detection_foreground2  = nh.advertise<sensor_msgs::Image>("detection/foreground2",1);


  pub_path = nh.advertise<visualization_msgs::Marker>("/ant/path", 1);
  pub_ant_model = nh.advertise<Cloud>("/ant/model",1);
  pub_ant_range = nh.advertise<visualization_msgs::Marker>("/ant/range", 1);

  first_depth_callback = true;

  ros::spin();

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
