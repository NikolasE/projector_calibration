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

  calibrator.eval_projection_matrix_Checkerboard(corners, ss, &area_mask);


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
  ros::NodeHandle n;

  user_input = new User_Input();
  user_input->init();

  train_frame_cnt = 1;
  train_background = true;


  foreGroundVisualizationActive = false;
  modeler_cell_size = 1/100.0; // given in m

  water_request_id = 0;
  mesh_visualizer = Mesh_visualizer(n);
  restart_modeler = true;

  max_update_dist = 0.1;

  current_frame_static = false;
  time_of_last_static_frame = ros::TIME_MIN; // earlier than everything


  next_ant_id = 0;

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
  fs << "max_dist" << max_dist;
  fs << "openGL_visualizationActive" << openGL_visualizationActive;
  fs << "depth_visualization_active" << depth_visualization_active;
  fs << "show_texture" << show_texture;
  fs << "water_simulation_active" << water_simulation_active;
  fs << "show_height_lines" << show_height_lines;
  fs << "calibration_active" << calibration_active;
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
  max_dist = (float)fs["max_dist"];
  openGL_visualizationActive = (int)fs["openGL_visualizationActive"];
  depth_visualization_active = (int)fs["depth_visualization_active"];
  show_texture = (int)fs["show_texture"];
  water_simulation_active  = (int)fs["water_simulation_active"];
  show_height_lines  = (int)fs["show_height_lines"];
  calibration_active = (int) fs["calibration_active"];

  return true;
}

void QNode::imgCloudCB(const sensor_msgs::ImageConstPtr& img_ptr, const sensor_msgs::PointCloud2ConstPtr& cloud_ptr)
{

  // ROS_INFO("new msgs");

  ros::Time now_callback = ros::Time::now();

  pcl::fromROSMsg(*cloud_ptr, current_cloud);
  cv_ptr = cv_bridge::toCvCopy(img_ptr, sensor_msgs::image_encodings::BGR8);

  current_col_img = cv_ptr->image;

  calibrator.setInputCloud(current_cloud);
  calibrator.setInputImage(current_col_img);


  if (calibrator.isKinectTrafoSet()){

    calibrator.publishWorldFrame("/openni_rgb_optical_frame","/fixed_frame");

    if(pub_cloud_worldsystem.getNumSubscribers() > 0){
      Cloud::Ptr msg = calibrator.cloud_moved.makeShared();
      msg->header.frame_id = "/fixed_frame";
      msg->header.stamp = ros::Time::now();
      pub_cloud_worldsystem.publish(msg);
    }
  }




  cv::Mat areaMask = cv::Mat(current_col_img.size(), CV_8UC1);
  areaMask.setTo(0);


  if (calibration_active){
    Q_EMIT received_col_Image();
    return;
  }




  cv::Mat fg = modeler.getFGMask(calibrator.cloud_moved, min_dist, &areaMask);

  std::vector<Playing_Piece> object_detections;
  std::vector<Grasp> grasp_detections;
  bool border_crossing;

#ifdef PRINT_TIMING
  ROS_INFO("getFGMask: %f ms", (ros::Time::now()-start_fg).toSec()*1000.0);
#endif



  ros::Time start_object_detection = ros::Time::now();

  // ~ 5ms (without visualization)
  detectGraspAndObjects(fg,areaMask, calibrator.cloud_moved, border_crossing, &grasp_detections, &object_detections, &current_col_img);

  // ROS_INFO("objects: %zu, grasps: %zu", object_detections.size(), grasp_detections.size());

#ifdef PRINT_TIMING
  ROS_INFO("detectPlayingPieces: %f ms", (ros::Time::now()-start_object_detection).toSec()*1000.0);
#endif


  if (border_crossing){
    hand_visible_in_last_frame = true;
  }else{

    if (hand_visible_in_last_frame)
      piece_Tracker.update_tracks(object_detections,1e6); // no maximal distance between corresponding object and detection
    else
      piece_Tracker.update_tracks(object_detections);

    hand_visible_in_last_frame = false;
  }

  Q_EMIT sig_handvisible(border_crossing);

  for (Piece_it it = piece_Tracker.tracks.begin(); it!=piece_Tracker.tracks.end(); ++it){
    Playing_Piece *p = &it->second;

    char text[20];
    sprintf(text,"%i", p->id);

    // draw number on image:
    cv::putText(current_col_img,text, p->last_detection,0,2,CV_RGB(255,0,0),2);
  }


  grasp_Tracker.update_tracks(grasp_detections);

  //for (uint i=0; i<grasp_debouncer.grasps.size(); ++i){
  for (Grasp_it it = grasp_Tracker.tracks.begin(); it!=grasp_Tracker.tracks.end(); ++it){
    Grasp* g = &it->second;


    /// emit signals so that other classes can react to the user input
    if (g->state == Track_Confirmed){ Q_EMIT sig_grasp_started(g->last_detection,g->id); }
    if (g->state == Track_Active){    Q_EMIT sig_grasp_moved(g->last_detection,g->id); }
    if (g->state == Track_Finished){  Q_EMIT sig_grasp_ended(g->last_detection,g->id); }

    cv::Scalar color;

    /// state before the grasp was detected three times
    if (g->state == Track_Initialized || g->state == Track_Confirmed){
      //    ROS_INFO("Initializing grasp %i at %f %f", g->id, g->last_detection.x, g->last_detection.y);
      color = CV_RGB(255,255,0);
    }

    /// normal state
    if (g->state == Track_Active){
      //    ROS_INFO("Active grasp (%i) at %f %f",i, g->last_detection.x, g->last_detection.y);
      color = CV_RGB(0,255,0);
    }

    /// end of grasp (only valid for one frame)
    if (g->state == Track_Finished){
      // ROS_INFO("finished grasp (%i) at %f %f",g->id, g->last_detection.x, g->last_detection.y);
      color = CV_RGB(255,0,0);
    }

    cv::circle(  current_col_img,g->last_detection,20, color,-1);
  }



  //  for (uint i=0; i<grasp_detections.size(); ++i){
  //   pcl_Point p3 = calibrator.cloud_moved.at(grasp_detections[i].x,grasp_detections[i].y);
  //   cv::Point grid = modeler.grid_pos(p3);
  //
  //   ROS_INFO("grasp %i 3d: %f %f %f", i, p3.x,p3.y,p3.z);
  //   ROS_INFO("grasp %i: %f %f", i, grasp_detections[i].x,grasp_detections[i].y);
  //   ROS_INFO("Grid pos: %i %i", grid.x, grid.y);
  //
  //   if (p3.x == p3.x && grid.x >= 0){
  //    ROS_INFO("Updating ant!");
  //   // update_ant(cv::Point(grid.x, grid.y));
  //   }
  //
  //  }

  Q_EMIT received_col_Image();

  return;



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
        float lx = 0.18;
        float ly = 0.15;

        //    modeler_cell_size = 0.0033;
        // modeler_cell_size = 0.01;
        modeler.init(modeler_cell_size, -lx,lx,-ly,ly);
      }


      //   ros::Time start_train = ros::Time::now();
      //   uint cnt = modeler.addTrainingFrame(calibrator.cloud_moved);
      //   ROS_INFO("Adding Frame: %f ms", (ros::Time::now()-start_train).toSec()*1000.0);
      //
      //   // uint cnt = detector.addTrainingFrame(calibrator.input_cloud);
      //   // ROS_INFO("Background calibration: added traingframe %i of %i", cnt, train_frame_cnt);
      //   if (cnt == train_frame_cnt)
      //    {
      //    //detector.computeBackground(0.007);
      //    //    train_background = false;
      //    //    ROS_INFO("computed Background");
      //    //
      //    //    cv::imwrite("area.jpg",area_mask);
      //    //    cv::imwrite("m1.jpg",detector.mask);
      //    //
      //    //    if (area_mask.cols == current_col_img.cols){
      //    //     detector.applyMask(area_mask);
      //    //    }
      //    //
      //    //    cv::imwrite("m2.jpg",detector.mask);
      //    //
      //    //
      //    //    Cloud foo = detector.showBackground(calibrator.input_cloud);
      //    //    Cloud::Ptr msg = foo.makeShared();
      //    //    msg->header.frame_id = "/openni_rgb_optical_frame";
      //    //    msg->header.stamp = ros::Time::now();
      //    //    pub_background.publish(msg);
      //
      //    // compute background method 2:
      //    ros::Time now = ros::Time::now();
      //    modeler.computeModel();
      //    ROS_INFO("Compute Model: %f ms", (ros::Time::now()-now).toSec()*1000.0);
      //
      //
      //
      //    //    Cloud model = modeler.getModel();
      //    //    Cloud::Ptr msg = model.makeShared();
      //    //    msg->header.frame_id = "/openni_rgb_optical_frame";
      //    //    msg->header.stamp = ros::Time::now();
      //    //    pub_model.publish(msg);
      //
      //    Q_EMIT model_computed();
      //
      //    }
    }



  if (foreGroundVisualizationActive && detector.isInitiated())
    {
      // ROS_INFO("foreground visulization active");

      //   ROS_INFO("RANGE from %f to %f", min_dist, max_dist);

      Cloud changed =  detector.removeBackground(calibrator.input_cloud, min_dist, max_dist);
      //   ROS_INFO("background: min %f, max: %f",  min_dist, max_dist);

      Cloud trafoed;
      pcl::getTransformedPointCloud(changed,calibrator.getCameraTransformation(),trafoed);


      Cloud::Ptr msg = trafoed.makeShared();
      msg->header.frame_id = "/openni_rgb_optical_frame";
      msg->header.stamp = ros::Time::now();
      pub_foreground.publish(msg);

      //   ROS_INFO("color_range: %f", color_range);

      projectCloudIntoImage(trafoed, calibrator.proj_Matrix,
                            calibrator.projector_image, -1,-100,  color_range); // -1,-100: all values are accepted

      //   cv::dilate(calibrator.projector_image,calibrator.projector_image,cv::Mat(),cv::Point(-1,-1),2);


      //   imwrite("fg.jpg",*detector.getForeground());

      //   ROS_INFO("Publishing foreground with %zu points", changed.size());

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
      projectCloudIntoImage(calibrator.cloud_moved, calibrator.proj_Matrix, calibrator.projector_image, max_dist, 0, color_range);


      Cloud colored = colorizeCloud(calibrator.cloud_moved,0, max_dist,color_range);
      Cloud::Ptr msg = colored.makeShared();
      msg->header.frame_id = "/openni_rgb_optical_frame";
      msg->header.stamp = ros::Time::now();
      pub_colored_cloud.publish(msg);

      Q_EMIT update_projector_image();
    }

  //  Q_EMIT received_col_Image();

  ROS_INFO("FULL Callback: %f ms", (ros::Time::now()-now_callback).toSec()*1000.0);
}




/**
  * @todo check if there is a good heightmodel
  * @return
  */
bool QNode::init_watersimulation(){

  cv::Mat land = modeler.getHeightImage();

  //  ROS_INFO("Height: %i %i, viscosity: %f", land.cols, land.rows,sim_viscosity);

  land.convertTo(land, CV_64FC1,1); // scaling?

  cv_bridge::CvImage out_msg;
  out_msg.encoding = sensor_msgs::image_encodings::TYPE_64FC1;
  out_msg.image    = land;


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


  cv::Mat height = modeler.getHeightImage();
  planner.setHeightMap(height);

  // only add water if simulation is running
  //  if (water.size() == height.size())
  //   planner.setWaterDepthMap(water);

  // todo: add water


  cv::Point start;
  if (ants.size() == 0){
    start = cv::Point(height.cols/2,height.rows/2); // ant starts at center of grid
    ant.setId(0);
    ROS_INFO("ANt is not initrialized");
  }else{
    ant = ants[ant.getId()];
    start = ant.getPosition();
    ROS_INFO("ant at pos %i ( %i %i)", ant.getPosInPath(), start.x, start.y );
  }

  ROS_INFO("UPDate: goal: %i %i", goal.x, goal.y);
  planner.computePolicy(goal,modeler.getCellSize());
  ROS_INFO("starting at %i %i", start.x, start.y);
  planner.computePath(start);

  ant.setPath(planner.getPath(),planner.getPathEdges());

  // replaces old ant (since same ID)
  //  Q_EMIT newAnt(ant);

  // currently, only one ant is used
  ants[ant.getId()] = ant;


  Q_EMIT sl_update_ant();

}


void QNode::run_ant_demo(){

  // getting and setting height map takes 0.01 ms
  cv::Mat height = modeler.getHeightImage();
  planner.setHeightMap(height);

  // only add water if simulation is running
  if (water.size() == height.size())
    planner.setWaterDepthMap(water);


  int w = height.cols;
  int h = height.rows;

  cv::Point goal = cv::Point(w-1,h-1);
  cv::Point start = cv::Point(15,15);


  planner.computePolicy(goal,modeler.getCellSize());



  planner.computePath(start);
  //  planner.saveHillSideCostImage();

  Ant ant;

  ant.setPath(planner.getPath(),planner.getPathEdges());

  ant.setId(next_ant_id++);
  Q_EMIT newAnt(ant);
}


bool QNode::step_watersimulation(){

  water_simulation::simulator_step msg_step;
  msg_step.request.iteration_cnt = iterations_per_frame;
  msg_step.request.id = water_request_id;

  // Updating the land height
  cv::Mat land = modeler.getHeightImage();
  land.convertTo(land, CV_64FC1,1); // scaling?
  cv_bridge::CvImage out_msg;
  out_msg.encoding = sensor_msgs::image_encodings::TYPE_64FC1;
  out_msg.image    = land;
  msg_step.request.land_img = *out_msg.toImageMsg();
  msg_step.request.update_land_img = true;

  // update viscosity
  msg_step.request.update_viscosity = true;
  msg_step.request.viscosity = sim_viscosity;



  water_simulation::msg_source_sink source;
  source.height = 0.05; // 5cm of water at this position
  cv::Point pos = modeler.grid_pos(0,0); // source at origin
  source.x = pos.x;
  source.y = pos.y;
  source.radius = 3;
  msg_step.request.sources_sinks.push_back(source);


  if (ros::service::call("srv_simulator_step", msg_step)){

    if (!msg_step.response.valid_id){
      ROS_WARN("WATER SIMULATION: sent request with wrong id!");
      return false;
    }else{
      //    ROS_INFO("WATER SIMULATION: Active");
      cv_bridge::CvImagePtr cv_ptr;
      cv_ptr = cv_bridge::toCvCopy(msg_step.response.water_img, sensor_msgs::image_encodings::TYPE_64FC1);

      cv_ptr->image.copyTo(water);
      //      cv::imshow("water", cv_ptr->image*10);
      //      cv::waitKey(1);
    }

  }



  return true;

}

/**
  *
  * //@todo apply bilinear filtering on image
  * @param msg
  */
void QNode::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

  Q_EMIT process_events();

  ros::Time cb_start = ros::Time::now();


  // < 1 ms
  cv_bridge::CvImagePtr depth_ptr = cv_bridge::toCvCopy(msg);
  cv::Mat * depth = &depth_ptr->image;


  //  int cnt = grasp_detector.updateModel(*depth);
  //  if (cnt > 3){
  //   grasp_detector.detectGrasps(grasps, 0.05,depth);
  //   for (uint i=0; i<grasps.size(); ++i){
  //    ROS_INFO("Grasp %i at %f %f", i,grasps.at(i).x,grasps.at(i).y);
  //   }
  //  }


  float fx = 525.0;  // focal length x
  float fy = 525.0;  // focal length y
  float cx = 319.5;  // optical center x
  float cy = 239.5;  // optical center y
  //  ros::Time start_creation = ros::Time::now();
  // 3 ms for creation of cloud
  Cloud cloud = createCloud(*depth, fx,fy,cx,cy);
  //  ROS_INFO("create cloud: %f ms", (ros::Time::now()-start_creation).toSec()*1000.0);

  ros::Time start_set = ros::Time::now();
  calibrator.setInputCloud(cloud);
  //  ROS_INFO("setting and transforming cloud: %f ms", (ros::Time::now()-start_set).toSec()*1000.0);



  //  if (depth_visualization_active && calibrator.projMatrixSet()){
  //   projectCloudIntoImage(calibrator.cloud_moved, calibrator.proj_Matrix, calibrator.projector_image, max_dist, 0, color_range);
  //   Q_EMIT update_projector_image();
  //   return;
  //  }


  // modeler.updateHeight(calibrator.cloud_moved);

  //  if (current_frame_static){
  //   run_ant_demo();
  ////   std::stringstream msg; msg << "";
  ////   writeToOutput(msg);
  //   writeToOutput(std::stringstream("new path was computed"));
  //   Q_EMIT model_computed();
  //   // todo: Q_EMIT for path visualization
  //  }



  //  pcl_Point center = calibrator.cloud_moved.at(320,240);
  //  ROS_INFO("center point of moved cloud %f %f %f",center.x,center.y,center.z);


  bool emit_new_image = false;


  //  if (water_simulation_active)
  //   ROS_INFO("water_simulation_active");
  //
  //  if (openGL_visualizationActive)
  //     ROS_INFO("openGL_visualizationActive");
  //
  //  if (show_height_lines)
  //     ROS_INFO("show_height_lines");


  if (calibrator.projMatrixSet() && (water_simulation_active || openGL_visualizationActive || show_height_lines)){
    //  if (calibrator.isKinectTrafoSet()){

    ros::Time start_train = ros::Time::now();
    bool current_frame_dynamic = modeler.updateHeight(calibrator.cloud_moved,max_update_dist);
#ifdef PRINT_TIMING
    ROS_INFO("Frame Update: %f ms", (ros::Time::now()-start_train).toSec()*1000.0);
#endif

    //   ros::Time start_obj = ros::Time::now();
    //   modeler.saveAsObj("foo.obj");
    //#ifdef PRINT_TIMING
    //   ROS_INFO("Creating Object file: %f ms", (ros::Time::now()-start_obj).toSec()*1000.0);
    //#endif

    //   if (current_frame_dynamic)
    //    ROS_INFO("dynamic scene");


    if (current_frame_dynamic){
      duration_since_last_static_frame = msg->header.stamp - time_of_last_static_frame;
    }else{
      time_of_last_static_frame = msg->header.stamp;

      if (duration_since_last_static_frame.toSec() > 0.5)
        ROS_INFO("Scene static again!");

      duration_since_last_static_frame = ros::Duration(0);
    }

    Q_EMIT scene_static(duration_since_last_static_frame.toSec());


    emit_new_image = true;
  }


  if (water_simulation_active && calibrator.projMatrixSet()){
    if (restart_modeler){
      init_watersimulation();
      restart_modeler = false;
    }
    else{
      ros::Time start_water = ros::Time::now();
      step_watersimulation();
#ifdef PRINT_TIMING
      ROS_INFO("Water simulation: %f ms", (ros::Time::now()-start_water).toSec()*1000.0);
#endif
    }
  }


  if (emit_new_image)
    Q_EMIT model_computed();

#ifdef PRINT_TIMING
  ROS_INFO("Callback time: %f ms", (ros::Time::now()-cb_start).toSec()*1000.0);
#endif
}

/**
  *
  * @param config
  * @param level
  */
void QNode::paramCallback(const projector_calibration::visualization_paramsConfig& config, uint32_t level){
  //  ROS_INFO("new config: %f cm, hist: %i", config.cell_length_cm, config.hist_length);

  train_frame_cnt = config.hist_length;
  modeler.weight = config.update_weight;

  //  ROS_INFO("cell size: %f, new size: %f", modeler_cell_size,config.cell_length_cm/100.0);

  iterations_per_frame = config.sim_iter_cnt;
  sim_viscosity = config.sim_viscosity;


  //   ROS_INFO("CONFIG>LIGHT_POS: %f",config.light_pos);
  Q_EMIT new_light(config.light_pos);

  max_update_dist = config.max_update_dist;

  planner.setMaxSteepness(config.ant_steepness);
  planner.setHeightCostFactor(config.ant_cost_factor);
  planner.setUphillfactor(config.ant_uphill_factor);
  planner.setFourNeighbours(config.ant_use_four == 1);
  planner.setSmoothing(config.ant_use_smoothing == 1);
  planner.setHillSideCostFactor(config.ant_hillside_factor);
  planner.setPathLengthFactor(config.ant_path_length_factor);



  if (abs(modeler_cell_size - config.cell_length_cm/100.0) > 0.00001){

    modeler_cell_size = config.cell_length_cm/100.0;
    // Initializing the modeler:
    float lx = 0.30;
    float ly = 0.25;

    // ros::Time start_init = ros::Time::now();
    modeler.init(modeler_cell_size, -lx,lx,-ly,ly);

    restart_modeler = true;
  }

}


void
QNode::run()
{
  //  ros::Rate loop_rate(40);


  // Initializing the modeler:
  float lx = 0.30;
  float ly = 0.20;
  // ros::Time start_init = ros::Time::now();
  modeler.init(modeler_cell_size, -lx,lx,-ly,ly);
  //     ROS_INFO("init modeller: %f ms", (ros::Time::now()-start_init).toSec()*1000.0);


  ros::NodeHandle nh;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
      sensor_msgs::PointCloud2> policy;
  message_filters::Subscriber<sensor_msgs::Image> image_sub(nh,
                                                            "/camera/rgb/image_color", 1);
  //                                                      "/camera/rgb/image_rect_color",1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh,
                                                                  "/camera/rgb/points", 1);
  //                                                                "/camera/depth_registered/points",1);
  message_filters::Synchronizer<policy> sync(policy(2), image_sub, cloud_sub);
  sync.registerCallback(boost::bind(&QNode::imgCloudCB, this, _1, _2));




  dynamic_reconfigure::Server<projector_calibration::visualization_paramsConfig> srv;
  dynamic_reconfigure::Server<projector_calibration::visualization_paramsConfig>::CallbackType f;
  f = boost::bind(&QNode::paramCallback,this, _1, _2);
  srv.setCallback(f);




  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("camera/depth/image", 10, boost::bind(&QNode::imageCallback, this, _1));
  //  image_transport::Subscriber sub_col = it.subscribe("/camera/rgb/image_color", 10, boost::bind(&QNode::imageColCallback, this, _1));


  pub_cloud_worldsystem = nh.advertise<Cloud> ("cloud_world", 1);
  pub_3d_calib_points = nh.advertise<Cloud> ("calibration_points_3d", 1);
  pub_colored_cloud = nh.advertise<Cloud> ("colored", 1);
  pub_eval_marker = nh.advertise<Cloud> ("disc_center", 1);
  pub_background = nh.advertise<Cloud> ("background", 1);
  pub_model = nh.advertise<Cloud> ("surface_model", 1);
  pub_foreground  = nh.advertise<Cloud> ("foreground", 1);

  pub_projector_marker = nh.advertise<visualization_msgs::Marker>("projector_pose",2);

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
