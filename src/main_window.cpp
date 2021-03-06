/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/projector_calibration/main_window.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

using namespace std;

namespace projector_calibration {

using namespace Qt;

/*****************************************************************************
  ** Implementation [MainWindow]
  *****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
  : QMainWindow(parent)
  , qnode(argc,argv)
{

  QRect screenres = QApplication::desktop()->screenGeometry(1);

  gl_viewer = new GL_Mesh_Viewer(parent);
  gl_viewer->resize(500,750);
  gl_viewer->show();

  ROS_INFO("argc: %i",argc);

  if (argc < 2){
    gl_viewer->move(QPoint(screenres.x(), screenres.y()));
    gl_viewer->showFullScreen();
  }

  darker = cv::Mat(480,640,CV_8UC3);
  inv = cv::Mat(480,640,CV_8UC1);

  pixmap_col = QPixmap(640,480);

  manual_z_change = 0;

  rgb_image_scale = 1;
  projector_image_scale = 0.5;


  ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
  QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

  ReadSettings();
  setWindowIcon(QIcon(":/images/icon.png"));
  ui.tab_manager->setCurrentIndex(4);

  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
  QObject::connect(&qnode, SIGNAL(received_col_Image()), this, SLOT(sl_received_image()));
  QObject::connect(&qnode, SIGNAL(update_projector_image()), this, SLOT(update_proj_image()));
  QObject::connect(&qnode, SIGNAL(model_computed()), this, SLOT(show_model_openGL()));
  QObject::connect(&qnode, SIGNAL(scene_static(double)), this, SLOT(scene_static(double)));
  QObject::connect(&qnode, SIGNAL(new_light(float)), this, SLOT(setLightPos(float)));
  QObject::connect(&qnode, SIGNAL(newAnt(Ant)), this, SLOT(got_new_ant(Ant)));
  QObject::connect(&qnode, SIGNAL(sl_update_ant()), this, SLOT(update_ant()));


  //  QObject::connect(&gl_viewer,SIGNAL())

  qRegisterMetaType<cv::Point2f>("cv::Point2f");

  QObject::connect(&qnode, SIGNAL(sig_grasp_started(cv::Point2f, int)), this, SLOT(sl_grasp_started(cv::Point2f, int)));
  QObject::connect(&qnode, SIGNAL(sig_grasp_moved(cv::Point2f, int)), this, SLOT(sl_grasp_moved(cv::Point2f, int)));
  QObject::connect(&qnode, SIGNAL(sig_grasp_ended(cv::Point2f, int)), this, SLOT(sl_grasp_ended(cv::Point2f, int)));
  QObject::connect(&qnode, SIGNAL(sig_handvisible(bool)), this, SLOT(  sl_handvisible(bool)));
  //  QObject::connect(&qnode, SIGNAL(visualize_Detections()), this, SLOT(  visualizeTracksAndPlanner()));
  QObject::connect(&qnode, SIGNAL(copy_projector_image()), this, SLOT(  updateProjectorImage()));


  secs_since_last_static_image = 0;

  QObject::connect(&mousehandler_projector, SIGNAL(redraw_image()), this, SLOT(update_proj_image()));

  show_fullscreen_pattern();
  update_proj_image();

  // Logging
  ui.view_logging->setModel(qnode.loggingModel());
  QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));
  QObject::connect(&qnode, SIGNAL(process_events()), this, SLOT(process_events()));

  QObject::connect(&mousehandler_projector, SIGNAL(marker_area_changed()), this, SLOT(select_marker_area()));
  QObject::connect(&mousehandler_points, SIGNAL(new_point()), this, SLOT(mouse_new_points()));
  QObject::connect(&mousehandler_points, SIGNAL(new_point()), this, SLOT( sl_received_image()));



  ui.cb_map->setChecked(false);
  show_map_image = false;


  ui.lb_img_2->installEventFilter(&mousehandler_projector);
  ui.lb_kinect_image->installEventFilter(&mousehandler_points);


  ui.ed_proj_size_x->setText(QString::number(qnode.calibrator.proj_size.width));
  ui.ed_proj_size_y->setText(QString::number(qnode.calibrator.proj_size.height));

  ui.ed_corners_x->setText(QString::number(qnode.calibrator.checkboard_size.width));
  ui.ed_corners_y->setText(QString::number(qnode.calibrator.checkboard_size.height));
  ui.ed_markersize->setText(QString::number(qnode.calibrator.printed_marker_size_mm));

  // qnode.user_interaction_active = ui.cb_user->isChecked();
  //  qnode.depth_visualization_active = ui.cb_depth_visualization->isChecked();
  //  qnode.foreGroundVisualizationActive = ui.cb_foreground->isChecked();
  pattern_size_auto = ui.cb_autosize->isChecked();




  gl_viewer->grasp_tracker = &qnode.grasp_tracker;
  gl_viewer->piece_tracker = &qnode.piece_tracker;
  gl_viewer->fingertip_tracker = &qnode.fingertip_tracker;


  cout << qnode.calibrator.cal_cv_with_dist.distCoeffs << endl;

  gl_viewer->setProjectorCalibration(qnode.calibrator.cal_cv_with_dist);

  gl_viewer->ants = &qnode.ants;

  loadParameters();
}

MainWindow::~MainWindow() {

}


void MainWindow::sl_grasp_started(cv::Point2f pos, int id){
  stringstream ss; ss << "New: " << id << " at: " << pos.x << " " << pos.y;
  qnode.writeToOutput(ss);
}
void MainWindow::sl_grasp_moved(cv::Point2f pos, int id){
  stringstream ss; ss << "moved: " << id << " at: " << pos.x << " " << pos.y;
  qnode.writeToOutput(ss);
}
void MainWindow::sl_grasp_ended(cv::Point2f pos, int id){
  stringstream ss; ss << "ended: " << id << " at: " << pos.x << " " << pos.y;
  qnode.writeToOutput(ss);
}




cv::Mat qimage2mat(const QImage& qimage) {
  cv::Mat mat = cv::Mat(qimage.height(), qimage.width(), CV_8UC4, (uchar*)qimage.bits(), qimage.bytesPerLine());
  cv::Mat mat2 = cv::Mat(mat.rows, mat.cols, CV_8UC3 );
  int from_to[] = { 0,0,  1,1,  2,2 };
  cv::mixChannels( &mat, 1, &mat2, 1, from_to, 3 );
  return mat2;
}

QImage mat2qimage(const cv::Mat& mat) {
  cv::Mat rgb;
  cv::cvtColor(mat, rgb, CV_BGR2RGB);
  return QImage((const unsigned char*)(rgb.data), rgb.cols, rgb.rows, QImage::Format_RGB888);
}

QImage Mat2QImage(const cv::Mat3b &src) {
  //return mat2qimage(src);

  QImage dest(src.cols, src.rows, QImage::Format_ARGB32);
  for (int y = 0; y < src.rows; ++y) {
    const cv::Vec3b *srcrow = src[y];
    QRgb *destrow = (QRgb*)dest.scanLine(y);
    for (int x = 0; x < src.cols; ++x) {
      destrow[x] = qRgba(srcrow[x][2], srcrow[x][1], srcrow[x][0], 255);
    }
  }
  return dest;
}



void MainWindow::new_expmap(int ignored){


#ifdef WITH_LIBGEOMETRY
  if (!qnode.modeler.isExpMapInitialized()){
    ROS_WARN("map is getting initialzed!");
    qnode.modeler.initExpMapGenerator();
  }

  int x_pos = ui.sl_expmap_x->value();
  int y_pos = ui.sl_expmap_y->value();
  float patch_radius = ui.sl_expmap_radius->value()/100.0;

  ROS_INFO("expmap: x: %i, y: %i, r: %f", x_pos, y_pos, patch_radius);

  //  timing_start("getPatch");
  UV_Patch patch =  qnode.modeler.getPatchAround(cv::Point(y_pos,x_pos),patch_radius);
  //  timing_end("getPatch");


  cv::Mat uv_mask, uv_values;
  //timing_start("visu");
  qnode.modeler.visualizePatch(patch,uv_mask, uv_values);
  //timing_end("visu");


  std::vector<cv::Vec3i> triangles;
  // timing_start("triangle");
  getTriangles(uv_mask, triangles); // ~ 1ms
  //timing_end("triangle");
  ROS_INFO("Found %zu triangles with uv-coordinates", triangles.size());

  gl_viewer->storeExpMapInfo(triangles, uv_values, 2/patch_radius, patch.center_id);

#else
  ROS_WARN("Support for ExpMap not active!");
#endif

}

void MainWindow::test_expmap(){

#ifdef WITH_LIBGEOMETRY

  if (!qnode.modeler.isExpMapInitialized()){
    ROS_WARN("map is getting initialzed!");
    qnode.modeler.initExpMapGenerator();
  }

  UV_Patch patch;

  int x_pos = ui.sl_expmap_x->value();
  int y_pos = ui.sl_expmap_y->value();
  float patch_radius = ui.sl_expmap_radius->value()/100.0;

  ROS_INFO("expmap: x: %i, y: %i, r: %f", x_pos, y_pos, patch_radius);

  // timing_start("patch 0.05 a");
  patch =  qnode.modeler.getPatchAround(cv::Point(y_pos,x_pos),patch_radius);
  //  timing_end("patch 0.05 a");

  cv::Mat uv_mask, uv_values;
  //timing_start("visu");
  qnode.modeler.visualizePatch(patch,uv_mask, uv_values);
  //timing_end("visu");


  std::vector<cv::Vec3i> triangles;
  // timing_start("triangle");
  getTriangles(uv_mask, triangles); // ~ 1ms
  //timing_end("triangle");
  ROS_INFO("Found %zu triangles with uv-coordinates", triangles.size());


  //  cv::imwrite("patch.png",uv_mask);

  gl_viewer->storeExpMapInfo(triangles, uv_values, 2/patch_radius, patch.center_id);


#else
  ROS_WARN("Support for ExpMap not active!");
#endif
}

void MainWindow::save_model_obj(){
  // bool ignore_triangles_without_normal = false;
  //qnode.modeler.saveAsObj("/usr/gast/engelhan/ros/master_thesis/surface.obj",ignore_triangles_without_normal);

  //  ros::Time start = ros::Time::now();
#ifdef WITH_LIBGEOMETRY
  qnode.modeler.initExpMapGenerator();
  test_expmap();
#endif


  //
  //  qnode.modeler.expMapTestRun();
  //
  //  //  for (int x= qnode.modeler.getModel().width-1; x>=0; --x)
  //  //   for (int y =0 ; y<qnode.modeler.getModel().height; ++y){
  //  //     UV_Patch patch =  qnode.modeler.getPatchAround(cv::Point(x,y));
  //  //   }
  //  // UV_Patch patch =  qnode.modeler.getPatchAround(cv::Point(60,60));
  //
  //  ROS_INFO("testRun: %f ms (for %zu patches)", (ros::Time::now()-start).toSec()*1000.0,qnode.modeler.getModel().size());
  //


}


void MainWindow::show_model_openGL(){

  qApp->processEvents();

  timing_start("open_gl");

  if (!qnode.elevation_map.modelComputed()){
    ROS_INFO("Model is not computed");
    return;
  }




  //  ros::Time now_getmodel = ros::Time::now();
  //  timing_start("get model");
  Cloud model;
  qnode.elevation_map.getModel(model);
  //  timing_end("get model");

  //  Cloud model = qnode.pixel_modeler.getModel(qnode.modeler.min_x(),qnode.modeler.min_y(),
  //                                             qnode.modeler.cell_size(),qnode.modeler.cell_size());



  //   Cloud model;
  //   qnode.pixel_modeler.getCloudRepresentation(qnode.current_cloud,model,-1);

  //   pcl::getTransformedPointCloud(model,qnode.calibrator.kinect_trafo,model);


  //  ROS_INFO("getModel: %f ms", (ros::Time::now()-now_getmodel).toSec()*1000);


  if (model.size() == 0){
    ROS_INFO("Model is not computed (and empty)");
    return;
  }


  //  if (!(qnode.openGL_visualizationActive || qnode.water_simulation_active)){
  //    return;
  //  }

  // TODO: create only once after model is initialized
  cv::Mat color(model.height, model.width, CV_8UC3);
  color.setTo(0);

  //  if (qnode.openGL_visualizationActive){

  // TODO: don't copy image
  cv::Mat height = qnode.elevation_map.getHeightImage();

  //    cv::Mat height;
  //    qnode.pixel_modeler.getMeans(height);
  heightVisualization(color, height, 0,500, qnode.color_range,NULL);
  //  }


  // cv::imwrite("height.png", color);

  if (qnode.water_simulation_active){

    // white surrounding if no height lines are visualized
    //    if (!qnode.openGL_visualizationActive)
    //      color.setTo(255);

    // ROS_INFO("water cols: %i, model: %i", qnode.water.cols, model.width);

    if (qnode.water.cols == int(model.width)){
      waterVisualization(color, qnode.water, 0.005,0.02,NULL);

      //      cv::Mat alpha; // todo: don't generate each time
      //      waterVisualizationAlpha(color, alpha,qnode.water, 0.005,0.02,NULL);
      // cv::imwrite("water.png", color);
      //    cv::imwrite("alpha.png", alpha);
    }else{
      ROS_ERROR("qnode.water.cols == int(model.width)");
    }
  }

  Cloud colored = transferColorToMesh(color, model, NULL);

#ifdef DO_TIMING
  ROS_INFO("colorizeCloud: %f ms", (ros::Time::now()-now_color).toSec()*1000.0);
#endif

  ros::Time now_mesh = ros::Time::now();
  float max_edge_length = 0.05; // max edge length of drawn triangle in m
  gl_viewer->cloud = colored;
  qnode.mesh_visualizer.createMesh(gl_viewer->cloud,gl_viewer->mesh, max_edge_length);
#ifdef DO_TIMING
  ROS_INFO("createMesh: %f ms", (ros::Time::now()-now_mesh).toSec()*1000.0);
#endif

  gl_viewer->proj_matrix = qnode.calibrator.proj_Matrix();

  assert(gl_viewer->proj_matrix.type() == CV_64FC1);

  // new stuff:
  gl_viewer->world2Projector = qnode.calibrator.cal_cv_with_dist.proj_trafo;
  gl_viewer->cam_internals = qnode.calibrator.cal_cv_with_dist.camera_matrix;

  //  cv::Mat P = qnode.calibrator.proj_matrix_cv;
  //  cout << "Projection Matrix openCV " << endl << P << endl;
  //  cout << "cv internal " << endl << qnode.calibrator.camera_matrix_CV << endl;
  //  cout << "cv trafo " << endl << qnode.calibrator.proj_trafo_CV << endl;
  //  cv::Mat p = qnode.calibrator.camera_matrix_CV*qnode.calibrator.proj_trafo_CV;
  //  p /= p.at<double>(3,3);
  //  cout << "product: " <<  p << endl;
  //  cv::Point2f px = applyPerspectiveTrafo(cv::Point3f(0,0,0),P);
  //  ROS_INFO("0,0,0 was projected to %f %f", px.x, px.y);

  if (qnode.show_height_lines){
    timing_start("height_lines");
    std::vector<Line_collection> height_lines;

    float min_height =  qnode.elevation_map.getMinHeight();
    float max_height =  qnode.elevation_map.getMaxHeight();
    float dist = qnode.height_line_distance;

    // get first and last visible height line:
    float min_ = ceil(min_height/dist)*dist;
    float max_ = floor(max_height/dist)*dist;


    qnode.mesh_visualizer.findHeightLines(gl_viewer->mesh, gl_viewer->cloud,height_lines, min_,max_,dist);

    gl_viewer->set_height_lines(height_lines);
    timing_end("height_lines");
  }else{
    std::vector<Line_collection> height_lines;
    gl_viewer->set_height_lines(height_lines);
  }






#ifdef DO_TIMING
  ROS_INFO("Showing Model with openGL (total): %f ms", (ros::Time::now()-start_show_openGL).toSec()*1000);
#endif


  gl_viewer->initMapSize(qnode.elevation_map.min_x(),qnode.elevation_map.min_y(),qnode.elevation_map.getWidth(), qnode.elevation_map.getHeight());


  //  // draws mesh and heigt_lines (if set)
  timing_start("Rendering");
  //  gl_viewer->withDistortion(true); // render with distortion

  gl_viewer->show_fullscreenimage = false;

  gl_viewer->updateGL();

  timing_end("Rendering");


  // Optional: Also render ortographic projection (geographical Map)
  if (show_map_image){
    QPixmap map_image = gl_viewer->getMapImage(ui.lb_img_2->width(),ui.lb_img_2->height());
    ui.lb_img_2->setPixmap(map_image);
    ui.lb_img_2->repaint();
  }

  //#ifdef DO_TIMING
  timing_end("open_gl");
  //#endif


}


/*****************************************************************************
  ** Implementation [Slots]
  *****************************************************************************/

void MainWindow::updateHeightLines(){


  
}

void MainWindow::updateProjectorImage(){

}



//void MainWindow::visualizeTracksAndPlanner(){

//}

void MainWindow::showNoMasterMessage() {
  QMessageBox msgBox;
  msgBox.setText("Couldn't find the ros master.");
  msgBox.exec();
  close();
}


void MainWindow::update_ant(){
  //  ROS_INFO("Updating Ant");

  //  if (qnode.ant.getState() == ANT_MOVING)
  //   gl_viewer->ants[qnode.ant.getId()] = qnode.ant;

//  gl_viewer->ants = &qnode.ants;

}


void MainWindow::got_new_ant(Ant ant){

//  ROS_INFO("Got new ant %i",ant.getId());

//  if (ant.getState() == ANT_MOVING){
//    ROS_INFO("Added ant");
//    gl_viewer->ants[ant.getId()] = ant;
//  }

}

void MainWindow::ant_demo(){
  qnode.run_ant_demo();
}

void MainWindow::select_marker_area(){

  if (!mousehandler_projector.area_marked()){
    stringstream ss; ss << "Select area on the projector image!";
    qnode.writeToOutput(ss);
    return;
  }



  // undo shrinking!
  cv::Point l1(mousehandler_projector.down.x/projector_image_scale,mousehandler_projector.down.y/projector_image_scale);
  cv::Point l2(mousehandler_projector.up.x/projector_image_scale,mousehandler_projector.up.y/projector_image_scale);

  if (pattern_size_auto){
    // set new pattern_size
    float dx = abs(l1.x-l2.x);
    float dy = abs(l1.y-l2.y);


    qnode.calibrator.checkboard_size.width = int(fmax(dx/100,3.0));
    qnode.calibrator.checkboard_size.height = int(fmax(dy/100,3.0));

    if (qnode.calibrator.checkboard_size.width == qnode.calibrator.checkboard_size.height)
      qnode.calibrator.checkboard_size.width++;

    ui.ed_corners_x->setText(QString::number(qnode.calibrator.checkboard_size.width));
    ui.ed_corners_y->setText(QString::number(qnode.calibrator.checkboard_size.height));

    //   ROS_INFO("new size: %i %i", qnode.calibrator.checkboard_size.width, qnode.calibrator.checkboard_size.height);


  }


  qnode.calibrator.projectSmallCheckerboard(l1,l2);
  update_proj_image(); // update image on gui

}



void MainWindow::save_kinect_trafo(){
  stringstream ss;

  if (!qnode.calibrator.isKinectTrafoSet()){
    ss << "Kinect Trafo not yet computed!";
    qnode.writeToOutput(ss);
    return;
  }

  qnode.calibrator.saveKinectTrafo(ss);
  qnode.calibrator.publishWorldFrame("/openni_rgb_optical_frame","/fixed_frame");
  qnode.writeToOutput(ss);
}


void MainWindow::activateSlider(){
  ui.slider_z->setEnabled(true);
  ui.slider_z->setValue(0);
  manual_z_change = 0;

  ui.slider_yaw->setEnabled(true);
  ui.slider_yaw->setValue(0);
  manual_yaw_change = 0;
}


void MainWindow::compute_trafo(){
  stringstream ss;

  qnode.calibrator.setInputImage(qnode.current_col_img);
  qnode.calibrator.setInputCloud(qnode.current_cloud);

  if(!qnode.calibrator.findCheckerboardCorners()){
    ss << "Could not detect Marker";
    qnode.writeToOutput(ss);
    return;
  }

  if (qnode.calibrator.computeKinectTransformation(ss)){


    Cloud::Ptr cloud_msg = qnode.calibrator.kinect_frame_points.makeShared();
    cloud_msg->header.frame_id = "/openni_rgb_optical_frame";
    cloud_msg->header.stamp = ros::Time::now ();
    qnode.pub_3d_calib_points.publish(cloud_msg);
    ROS_INFO("Sending %zu point", qnode.calibrator.kinect_frame_points.size());

    ss << "Computation of Kinect Trafo successful";
    activateSlider();
    qnode.calibrator.publishWorldFrame("/openni_rgb_optical_frame","/fixed_frame");
  }else{
    ss << "Computation of Kinect Trafo failed";
  }

  qnode.writeToOutput(ss);
}



void MainWindow::projection_opencv(){

  float mean_error, max_error;
  qnode.calibrator.computeProjectionMatrix_OPENCV(mean_error, max_error,true);
  stringstream ss; ss << "Opencv WITH distortion coeffs: " << mean_error << " max: " << max_error;
  qnode.writeToOutput(ss);

  qnode.calibrator.computeProjectionMatrix_OPENCV(mean_error,max_error,false);
  stringstream ss2; ss2 << "Opencv WITHOUT distortion coeffs: " << mean_error << " max: " << max_error;
  qnode.writeToOutput(ss2);

  if (qnode.pub_projector_marker.getNumSubscribers()>0){
    visualization_msgs::Marker marker;
    qnode.calibrator.createProjektorMarker_CV(marker);
    qnode.pub_projector_marker.publish(marker);
  }

}


void MainWindow::marker_size_changed(){
  // check if input is valid (positive) double
  // if not reset edit to old value
  double val;
  bool ok;
  val = ui.ed_markersize->text().toDouble(&ok);

  std::stringstream ss;

  if (ok && val > 0){
    ss << "New length of printed marker square: " << val << "mm";
    qnode.calibrator.printed_marker_size_mm = val;
  }else{
    // should not happen due to input mask
    ss << val << "  is no valid double value, resetting to old value";
    ui.ed_markersize->setText(QString::number(qnode.calibrator.printed_marker_size_mm));
  }

  qnode.writeToOutput(ss);
}


void MainWindow::load_kinect_trafo_from_file(){
  stringstream msg;

  qnode.calibrator.loadKinectTrafo(msg);
  qnode.writeToOutput(msg);

  if (qnode.calibrator.isKinectTrafoSet()){
    activateSlider();
  }

}


void MainWindow::project_black_background(){
  // projects black background and removes position of last checkerboard detections
  qnode.calibrator.projectUniformBackground(false);
  update_proj_image(); // update image on gui
}

void MainWindow::project_white_background(){
  // projects white background and removes position of last checkerboard detections
  qnode.calibrator.projectUniformBackground(true);
  update_proj_image(); // update image on gui
}



void MainWindow::pattern_size_changed(){
  ROS_INFO("Changing checkerboard size to %i %i",ui.ed_corners_x->text().toInt(), ui.ed_corners_y->text().toInt());
  qnode.calibrator.checkboard_size = cv::Size(ui.ed_corners_x->text().toInt(), ui.ed_corners_y->text().toInt());
}

void MainWindow::update_proj_image(){


  // timing_start("copy_projector");

  gl_viewer->showImage(qnode.calibrator.projector_image);
  gl_viewer->updateGL();

  // timing_end("copy_projector");


  // draw projector image on right label
  if (qnode.calibrator.projector_image.cols > 0){
    cv::Mat small2;
    cv::resize(qnode.calibrator.projector_image, small2, cv::Size(),projector_image_scale,projector_image_scale, CV_INTER_CUBIC);

    if (mousehandler_projector.area_marked())
      cv::rectangle(small2,  mousehandler_projector.down, mousehandler_projector.move, mousehandler_projector.area_marked()?CV_RGB(0,255,0):CV_RGB(255,0,0),2);


    qimg_p = Mat2QImage(small2);
    ui.lb_img_2->setPixmap(pixmap_p.fromImage(qimg_p, 0));
    ui.lb_img_2->repaint();
  }

}

void MainWindow::evaluate_pattern(){ qnode.eval_projection(); }

void MainWindow::detect_disc(){   qnode.disc_evaluation();  }

void MainWindow::scene_static(double secs_since_last_static_image){

  //  if (this->secs_since_last_static_image > secs_since_last_static_image){
  //   if (qnode.modeler.modelComputed()){
  //    qnode.modeler.initExpMapGenerator();
  //    test_expmap();
  //   }
  //  }

  this->secs_since_last_static_image = secs_since_last_static_image;
  ui.lb_static->setText(QString::number(int(secs_since_last_static_image*100)/100.0));
  ui.lb_static->repaint();
}

void MainWindow::setLightPos(float z){
  //  ROS_INFO("UPDATING LIGHT POSITION");
  // gl_viewer->setLightPos(z);
  //  std::stringstream ss;
  //  ss << "light pos " << z;
  //  qnode.writeToOutput(ss);
}

void MainWindow::mouse_new_points(){

  // ROS_INFO("mouse_new_points> %zu", mousehandler_points.pts.size());

  if (mousehandler_points.pts.size() < 4) { return; }

  vector<cv::Point> normal_sized;

  for (uint i=0; i<mousehandler_points.pts.size(); ++i){
    cv::Point pi = mousehandler_points.pts[i];
    normal_sized.push_back( cv::Point(pi.x/rgb_image_scale, pi.y/rgb_image_scale));
  }


  if (qnode.current_col_img.rows == 0) return;

  qnode.area_mask = cv::Mat(qnode.current_col_img.size(), CV_8UC1);
  qnode.area_mask.setTo(0);
  cv::fillConvexPoly(qnode.area_mask, normal_sized, cv::Scalar::all(255));


  qnode.detector.setDetectionArea(qnode.area_mask);
  qnode.pixel_modeler.setMask(qnode.area_mask);


  // mousehandler_points.pts.clear();

  ROS_INFO("saving area mask to data/kinect_mask.png");
  cv::imwrite("data/kinect_mask.png", qnode.area_mask);
  //  cv::namedWindow("bar");
  //  cv::imshow("bar", qnode.area_mask);
  //  cv::waitKey(10);

}


void MainWindow::sl_received_image(){


  // ROS_INFO("Updating image on GUI");

  frame_update_times.push_back(ros::Time::now());
  if (frame_update_times.size() > hist_length){
    ros::Time earlier = frame_update_times[frame_update_times.size()-1-hist_length];
    double fr = hist_length/(frame_update_times[frame_update_times.size()-1]-earlier).toSec();
    ui.lb_framerate->setText(QString::number(int(fr*100)/100.0));
  }

  timing_start("draw image");

  if (qnode.current_col_img.cols == 0){
    return;
  }

  // draw kinect image on left label
  cpy = qnode.current_col_img.clone();

  if (qnode.calibrator.detected_corners.size() > 0){
    cv::drawChessboardCorners(cpy, qnode.calibrator.checkboard_size, qnode.calibrator.detected_corners, true);
  }

  /*
   * Draw region not withing the marked region darker
   */
  if (draw_mask){
    timing_start("dark");
    if (qnode.area_mask.cols == cpy.cols){
      darker = cpy*0.5;
      inv = 255- qnode.area_mask;
      darker.copyTo(cpy,inv);
    }
    // timing_end("dark");
  }

  bool resize = abs(rgb_image_scale-1.0) > 0.01;
  if (resize){
    ROS_INFO("resizing image");
    cv::resize(cpy, small, cv::Size(),rgb_image_scale,rgb_image_scale, CV_INTER_CUBIC);
  }

  cv::Mat& img = resize?small:cpy;

  // 0ms
  qimg_col = QImage((const uchar*)(img.data), img.cols, img.rows, img.step1(), QImage::Format_RGB888).rgbSwapped();


  // 2-3 ms
  //timing_start("cp_painter");
  QPainter painter(&pixmap_col);
  painter.drawImage(pixmap_col.rect(),qimg_col,qimg_col.rect());
  //timing_end("cp_painter");

  ui.lb_kinect_image->setPixmap(pixmap_col); // 0 ms
  ui.lb_kinect_image->repaint();


  //  timing_end("draw image"); // 3-4 ms

  //qApp->processEvents();
}

void MainWindow::loadParameters(){
  if (!qnode.loadParameters()) return;

  sl_threshold_changed(qnode.calibrator.eval_brightness_threshold);
  color_slider_moved(qnode.color_range*100);
  min_dist_changed(qnode.min_dist*1000);
  //  z_max_changed(qnode.max_dist*100);
  heightline_dist_changed(qnode.height_line_distance*10*100);

  draw_mask = true;
  ui.cb_dark->setChecked(draw_mask);

  //  ui.cb_
  ui.cb_showHeightLines->setChecked(qnode.show_height_lines);

  // ui.cb_depth_visualization_GL->setChecked(qnode.openGL_visualizationActive);
  // ui.cb_depth_visualization->setChecked(qnode.depth_visualization_active);

  ui.cb_distortion->setChecked(gl_viewer->withDistortion());


  qnode.calibration_active = false;

  ui.cb_calibration_active->setChecked(qnode.calibration_active);
  gl_viewer->show_texture = qnode.show_texture;
  ui.cb_texture->setChecked(gl_viewer->show_texture);
  ui.cb_water->setChecked(qnode.water_simulation_active);


  ui.fg_update_model->setChecked(qnode.update_pixel_model);
  ui.cb_gesture->setChecked(false);  // qnode.do_gesture_recognition);
  qnode.toggle_gesture_recognition(false);
  ui.fg_update_map->setChecked(true);// qnode.update_elevation_map);
  qnode.update_elevation_map = true;


  ui.cb_water->setChecked(qnode.water_simulation_active);

}

void MainWindow::toggle_update_elevationmap(bool status){
  qnode.update_elevation_map = status;
  qnode.saveParameters();
}


void MainWindow::sl_handvisible(bool visible){
  ui.lb_handvisible->setText(visible?"visible":"invisible");
  ui.lb_handvisible->repaint();
}

void MainWindow::color_slider_moved(int col_cm){
  // set sliderposition so that function can also be called
  // when loading parameters from file
  ui.slider_color->setSliderPosition(col_cm);
  qnode.color_range = col_cm/100.0; // now in meter

  // ROS_INFO("color slider: %i cm", col_cm);
  ui.lb_color->setText(QString::number(col_cm));

  qnode.saveParameters();
}


// Calibration Evaluation
void MainWindow::sl_threshold_changed(int threshold){
  ui.sl_brightness->setSliderPosition(threshold);
  qnode.calibrator.eval_brightness_threshold = threshold;
  ui.lb_brightness->setText( QString::number(threshold));
  ui.lb_brightness->repaint();

  qnode.saveParameters();
}

void MainWindow::min_dist_changed(int min_dist){

  ui.slider_mindist->setSliderPosition(min_dist);
  qnode.min_dist = min_dist/1000.0;
  ui.lb_mindist->setText(QString::number(min_dist));

  ROS_INFO("Detection Min dist: %f", qnode.min_dist);

  qnode.saveParameters();
}


void MainWindow::heightline_dist_changed(int pos){
  ui.sl_height_dist->setSliderPosition(pos);
  qnode.height_line_distance = pos/1000.0;
  ui.lb_height_dist->setText(QString::number(pos/10.0));
  qnode.saveParameters();
}

void MainWindow::z_max_changed(int z_max){
  //  ui.slider_z_max->setSliderPosition(z_max);
  //  qnode.max_dist = z_max/100.0; // given in cm
  //  ui.z_max_label->setText(QString::number(z_max));
  //  ui.z_max_label->repaint();

  //  qnode.saveParameters();
}



void MainWindow::drawDetections(QImage* img){






}

void MainWindow::visualizeDetectionsOnSurface(){

  // ROS_INFO("visualizeDetectionsOnSurface");


  //  for (GraspTrack_it it = qnode.grasp_tracker.tracks.begin(); it != grasp_tracker.tracks.end(); ++it){
  //    if (it->second.state == Track_Active){
  //      it->second.visualizeOnImage(current_col_img,getColor(it->first));
  //      pcl_Point center = it->second.last_detection().position_world;
  //      grasps.push_back(center);
  //      ROS_INFO("Found grasp at: %f %f %f", center.x,center.y, center.z);
  //    }
  //  }



  //  for (PieceTrack_it it = qnode.piece_tracker.tracks.begin(); it != piece_tracker.tracks.end(); ++it){
  //    if (it->second.state == Track_Active){
  //      it->second.visualizeOnImage(current_col_img,getColor(it->first));
  //      pcl_Point center = it->second.last_detection().position_world;
  //      ROS_INFO("Found Piece (%i) at: %f %f %f", it->first, center.x,center.y, center.z);
  //    }
  //  }


  //  for (FingerTrack_it it = qnode.fingertip_tracker.tracks.begin(); it != fingertip_tracker.tracks.end(); ++it){
  //    // if (it->second.state == Track_Active)
  //    //it->second.visualizeOnImage(current_col_img,getColor(it->first),TT_FINGERTIP);
  //    pcl_Point center = it->second.last_detection().position_kinect;

  //    cv::Point2f c = applyPerspectiveTrafo(center, qnode.calibrator.proj_Matrix);


  //    ROS_INFO("Found finger at: %f %f %f", center.x,center.y, center.z);
  //  }


}



void MainWindow::start_water_node(){
  int foo = system("rosrun water_simulation simulation &");
  cout << "started water_simulation with return value " << foo << endl;
}

void MainWindow::load_observations(){
  qnode.calibrator.loadObservations();
}

void MainWindow::manual_yaw_changed(int yaw){
  float y = yaw/2.0;
  ROS_INFO("y: %f", y);
  assert(qnode.calibrator.isKinectTrafoSet());
  float diff = y-manual_yaw_change;
  qnode.calibrator.rotateKinectTrafo(diff/180.0*M_PI);
  manual_yaw_change = y;
  ROS_INFO("Manual yaw: %f", manual_yaw_change);
  ui.lb_yaw->setText(QString::number(manual_yaw_change));
}


void MainWindow::save_elevation_map(){
  cv::Mat height = qnode.elevation_map.getHeightImage();
  cv::imwrite("img/height.png",height*255);
  ROS_INFO("wrote height image to img/height.png");
  double min_val,max_val;
  cv::minMaxLoc(height, &min_val, &max_val);
  cv::Mat scaled = (height-min_val)/(max_val-min_val);

  ROS_INFO("Height: %f %f", min_val, max_val);

  cv::imwrite("img/height_scaled.png",scaled*255);

  cv::minMaxLoc(scaled, &min_val, &max_val);
  ROS_INFO("Height2: %f %f", min_val, max_val);
}



void MainWindow::manual_z_changed(int z){
  assert(qnode.calibrator.isKinectTrafoSet());
  float diff = z-manual_z_change;
  qnode.calibrator.translateKinectTrafo(diff/100.0);

  qnode.calibrator.publishWorldFrame("/openni_rgb_optical_frame","/fixed_frame");


  if (qnode.calibrator.projMatrixSet()){
    // TODO: just adapt matrices to new coordinate system if already defined
    // qnode.calibrator.proj_Matrix = cv::Mat(0,0,CV_32FC1);
    // qnode.calibrator.hom_CV = cv::Mat(0,0,CV_32FC1);

    stringstream ss; ss << "Matrices are now invalid! (TODO!)";
    qnode.writeToOutput(ss);
  }



  manual_z_change = z;
  ui.lb_z->setText(QString::number(manual_z_change));
}


void MainWindow::show_fullscreen_pattern(){

  qnode.calibrator.projectFullscreenCheckerboard();

  ui.ed_corners_x->setText(QString::number(qnode.calibrator.checkboard_size.width));
  ui.ed_corners_y->setText(QString::number(qnode.calibrator.checkboard_size.height));

  update_proj_image(); // update image on gui
}



void MainWindow::find_projection_area(){
  sstream msg;
  qnode.calibrator.findOptimalProjectionArea2(qnode.calibrator.test_img.size, msg);

  if (qnode.calibrator.warpMatrixSet()){
    qnode.calibrator.showUnWarpedImage(*qnode.calibrator.getTestImg());
    update_proj_image();
  }
  qnode.writeToOutput(msg);

}


/*****************************************************************************
  ** Calibration Functions
  *****************************************************************************/

void MainWindow::add_new_observation(){
  sstream msg;

  if (!qnode.calibrator.isKinectTrafoSet()){
    qnode.writeToOutput(sstream("Can't add observation if coordinate system is not defined!"));
    return;
  }

  if (qnode.calibrator.getCurrentProjectorCornerCnt() == 0){
    qnode.writeToOutput(sstream("Can't add observation if no pattern is projected!"));
    return;
  }


  // pass current images to the calibrator
  //  qnode.calibrator.setInputImage(qnode.current_col_img);
  //  qnode.calibrator.setInputCloud(qnode.current_cloud);

  if(!qnode.calibrator.findCheckerboardCorners()){
    qnode.writeToOutput(sstream("Could not detect Marker!"));
    return;
  }

  qnode.calibrator.storeCurrentObservationPairs();
  qnode.calibrator.saveObservations();

  if (qnode.pub_3d_calib_points.getNumSubscribers() > 0){
    ROS_WARN("SENDING %zu 3d observations", qnode.calibrator.observations_3d.size());
    // show observations in rviz

    pcl_Point *p = &qnode.calibrator.observations_3d[0];
    p->r = 0;
    p->g = 1;

    p = &qnode.calibrator.observations_3d[1];
    p->r = 0;
    p->b = 1;

    Cloud::Ptr cloud_msg = qnode.calibrator.observations_3d.makeShared();
    cloud_msg->header.frame_id = "/fixed_frame";
    cloud_msg->header.stamp = ros::Time::now ();
    qnode.pub_3d_calib_points.publish(cloud_msg);
  }

  msg << "Now " << qnode.calibrator.getNumPairs() << " pairs for optimization";
  qnode.writeToOutput(msg);

}

void MainWindow::compute_homography(){
  sstream msg;
  float mean_error;
  if (qnode.calibrator.computeHomography_OPENCV(mean_error)){
    msg << "Homography computed with mean reprojection error of " << setprecision(3) << mean_error << " pixels ";
  }else{
    msg << "Could not compute Homography (not enough points)";
  }
  qnode.writeToOutput(msg);

}

void MainWindow::compute_projection_matrix(){
  sstream msg;

  float mean_error;
  if (qnode.calibrator.computeProjectionMatrix(mean_error,false)){
    msg << "Projection Matrix WITHOUT Normalization " << mean_error << "pixels ";
    // qnode.calibrator.computeProjectionMatrix_OPENCV(mean_error);
    // cout << "opencv calib: mean of " << mean_error << endl;

    //    cv::Mat K,R,t;

    //    cv::Mat rx,ry,rz;
    //    vector<double> angles;

    //    // cv::Point3d angles;

    //    cv::decomposeProjectionMatrix(qnode.calibrator.proj_Matrix,K,R,t,rx,ry,rz,angles);

    //    t /= t.at<double>(3);
    //    K /= K.at<double>(2,2);

    //    msg << endl << "translation: " << t.at<double>(0) << " " << t.at<double>(1) << " " << t.at<double>(2) << endl;
    //    msg << "angles: " << angles[0] << " " << angles[1] << " " << angles[2];

    //    ROS_INFO("sending projector Pose as Marker");
    //    visualization_msgs::Marker marker;
    //    qnode.calibrator.createProjektorMarker(marker);
    //    qnode.pub_projector_marker.publish(marker);


    qnode.calibrator.computeProjectionMatrix(mean_error,true);
    msg << endl <<  "Projection Matrix WITH Normalization " << mean_error << "pixels ";
    visualization_msgs::Marker marker;
    qnode.calibrator.createProjektorMarker(marker);
    qnode.pub_projector_marker.publish(marker);

  }else{
    msg << "Could not compute Projection Matrix (not enough points or to little variation in z-direction)";
  }

  qnode.writeToOutput(msg);
}

void MainWindow::save_projection_matrix(){
  sstream msg;
  qnode.calibrator.saveCalibrations(msg);
  qnode.writeToOutput(msg);
}

void MainWindow::save_homography(){
  sstream msg;
  qnode.calibrator.saveHomographyCV(msg);
  qnode.writeToOutput(msg);
}

void MainWindow::clear_projection_matrix(){
  // qnode.calibrator.proj_Matrix = cv::Mat(0,0,CV_64FC1);
}

void MainWindow::show_height_lines(bool status){
  qnode.show_height_lines = status;
  qnode.saveParameters();
}

void MainWindow::dark_toggled(bool status){
  draw_mask = status;
}


void MainWindow::show_map_toggled(bool status){
  show_map_image = status;
}

void MainWindow::wireframe_toggled(bool status){
  gl_viewer->toggleWireframe(status);
}

void MainWindow::path_toggled(bool status){
  qnode.with_path_planning = status;

  if (status){
    qnode.planner.setHeightMap(qnode.elevation_map.mean,qnode.elevation_map.cell_size());
  }

  //  if (status){
  //    ui.cb_gesture->setChecked(false);
  //    ui.fg_update_model->setChecked(false);
  //  }
}


void MainWindow::with_distortion_toggled(bool status){
  gl_viewer->toggleDistortion(status);
  //  gl_viewer->resizeGL(gl_viewer->w_,gl_viewer->h_);
}


void MainWindow::gesture_toggled(bool status){
  qnode.toggle_gesture_recognition(status);
  ui.lb_handvisible->setText("??");

  qnode.saveParameters();
}


void MainWindow::toggle_update_model(bool status){
  qnode.update_pixel_model = status;
  //  if (status){
  //    ui.cb_gesture->setChecked(false);
  //  }
  qnode.saveParameters();
}

void MainWindow::foreground_visualization_toggled(bool status){
  qnode.calibration_active = status;

  // gl_viewer->resizeGL(gl_viewer->width(),gl_viewer->height());

  //  if (!status){
  //    gl_viewer->withDistortion(false);
  //    ui.cb_distortion->setChecked(false);
  //  }

  qnode.saveParameters();
}

void MainWindow::water_simulation_toggled(bool status){

  if (status && !qnode.water_simulation_active){
    //bool success = qnode.init_watersimulation();
    //   if (success)
    //    ROS_INFO("Started water simulation");
    //   else
    //    ROS_INFO("Could not start water simulation");
  }

  qnode.water_simulation_active = status;
  qnode.saveParameters();
}

void MainWindow::gl_visualization_toggled(bool status){
  ROS_INFO("deprecated!");
  //  qnode.openGL_visualizationActive = status;
  //  qnode.saveParameters();
}

//void MainWindow::user_interaction_toggled(bool status){
//  qnode.user_interaction_active = status;
//}

//void MainWindow::foreGroundVisualizationToggled(bool status){

//  //  min_dist_changed(ui.slider_mindist->value());
//  //  z_max_changed(ui.slider_z_max->value());
//  //  color_slider_moved(ui.slider_color->value());
//  qnode.calibration_active = status;
//  qnode.foreGroundVisualizationActive = status;
//}

void MainWindow::process_events(){
  qApp->processEvents();
}

void MainWindow::pattern_auto_size_toggled(bool status){
  pattern_size_auto = status;

  if (pattern_size_auto)
    qnode.writeToOutput(sstream("Pattern Size: auto"));
  else
    qnode.writeToOutput(sstream("Pattern Size: user defined"));

}

void MainWindow::depth_visualzation_toggled(bool status){

  //  min_dist_changed(ui.slider_mindist->value());
  //  min_dist_changed(ui.slider_z_max->value());
  //  color_slider_moved(ui.slider_color->value());

  qnode.depth_visualization_active = status;
}


void MainWindow::show_texture(bool status){
  gl_viewer->show_texture = status;

  qnode.show_texture = status;
  qnode.saveParameters();
}

void MainWindow::delete_last_img(){
  qnode.calibrator.restartCalibration();
  //  if (!qnode.calibrator.removeLastObservations())
  //    qnode.writeToOutput(sstream("No image to remove"));
}



void MainWindow::restart_water_simulation(){
  qnode.restart_modeler = true;
}


/*****************************************************************************
  ** Implemenation [Slots][manually (connect)ed]
  *****************************************************************************/

/**
  * This function is signalled by the underlying model. When the model changes,
  * this will drop the cursor down to the last line in the QListview to ensure
  * the user can always see the latest log message.
  */
void MainWindow::updateLoggingView() {
  ui.view_logging->scrollToBottom();
}

/*****************************************************************************
  ** Implementation [Menu]
  *****************************************************************************/


/*****************************************************************************
  ** Implementation [Configuration]
  *****************************************************************************/

void MainWindow::ReadSettings() {
  QSettings settings("Qt-Ros Package", "projector_calibration");
  restoreGeometry(settings.value("geometry").toByteArray());
  restoreState(settings.value("windowState").toByteArray());
}

void MainWindow::WriteSettings() {
}

void MainWindow::closeEvent(QCloseEvent *event)
{
  WriteSettings();
  QMainWindow::closeEvent(event);
}

}  // namespace projector_calibration

