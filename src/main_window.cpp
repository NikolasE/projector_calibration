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

  gl_viewer = new GL_Mesh_Viewer(parent);

  // open label fullscreen on secondary monitor
  lb_img.setParent(NULL);
  QRect screenres = QApplication::desktop()->screenGeometry(1);
  //  ROS_INFO("screenres 1: %i %i", screenres.x(), screenres.y());
  lb_img.move(QPoint(screenres.x(), screenres.y()));
  lb_img.showFullScreen();


  manual_z_change = 0;

  image_scale = 0.5;

  ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
  QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

  ReadSettings();
  setWindowIcon(QIcon(":/images/icon.png"));
  ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).

  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
  QObject::connect(&qnode, SIGNAL(received_col_Image()), this, SLOT(sl_received_image()));
  QObject::connect(&qnode, SIGNAL(update_projector_image()), this, SLOT(update_proj_image()));
  QObject::connect(&qnode, SIGNAL(model_computed()), this, SLOT(show_model_openGL()));
  QObject::connect(&qnode, SIGNAL(scene_static(double)), this, SLOT(scene_static(double)));
  QObject::connect(&qnode, SIGNAL(new_light(float)), this, SLOT(setLightPos(float)));
  QObject::connect(&qnode, SIGNAL(newAnt(Ant)), this, SLOT(got_new_ant(Ant)));
  QObject::connect(&qnode, SIGNAL(sl_update_ant()), this, SLOT(update_ant()));



  QObject::connect(&mousehandler_projector, SIGNAL(redraw_image()), this, SLOT(update_proj_image()));



  if (qnode.calibrator.load_everything_on_startup){
   sstream msg;
   qnode.calibrator.initFromFile(msg);
   qnode.writeToOutput(msg);
   if (qnode.calibrator.isKinectTrafoSet()){
    activateSlider();
   }
  }


  show_fullscreen_pattern();
  update_proj_image();

  // Logging
  ui.view_logging->setModel(qnode.loggingModel());
  QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));
  QObject::connect(&qnode, SIGNAL(process_events()), this, SLOT(process_events()));

  QObject::connect(&mousehandler_projector, SIGNAL(marker_area_changed()), this, SLOT(select_marker_area()));
  QObject::connect(&mousehandler_points, SIGNAL(new_point()), this, SLOT(mouse_new_points()));
  QObject::connect(&mousehandler_points, SIGNAL(new_point()), this, SLOT( sl_received_image()));




  ui.lb_img_2->installEventFilter(&mousehandler_projector);
  ui.lb_kinect_image->installEventFilter(&mousehandler_points);


  ui.ed_proj_size_x->setText(QString::number(qnode.calibrator.proj_size.width));
  ui.ed_proj_size_y->setText(QString::number(qnode.calibrator.proj_size.height));

  ui.ed_corners_x->setText(QString::number(qnode.calibrator.checkboard_size.width));
  ui.ed_corners_y->setText(QString::number(qnode.calibrator.checkboard_size.height));
  ui.ed_markersize->setText(QString::number(qnode.calibrator.printed_marker_size_mm));

  qnode.user_interaction_active = ui.cb_user->isChecked();
  qnode.depth_visualization_active = ui.cb_depth_visualization->isChecked();
  qnode.foreGroundVisualizationActive = ui.cb_foreground->isChecked();
  pattern_size_auto = ui.cb_autosize->isChecked();

  loadParameters();

 }

 MainWindow::~MainWindow() {

 }


 cv::Mat qimage2mat(const QImage& qimage) {
     cv::Mat mat = cv::Mat(qimage.height(), qimage.width(), CV_8UC4, (uchar*)qimage.bits(), qimage.bytesPerLine());
     cv::Mat mat2 = cv::Mat(mat.rows, mat.cols, CV_8UC3 );
     int from_to[] = { 0,0,  1,1,  2,2 };
     cv::mixChannels( &mat, 1, &mat2, 1, from_to, 3 );
     return mat2;
 };

 QImage mat2qimage(const cv::Mat& mat) {
     cv::Mat rgb;
     cv::cvtColor(mat, rgb, CV_BGR2RGB);
     return QImage((const unsigned char*)(rgb.data), rgb.cols, rgb.rows, QImage::Format_RGB888);
 };


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



 void MainWindow::show_model_openGL(){


  qApp->processEvents();

  ros::Time start_show_openGL = ros::Time::now();


  if (!qnode.modeler.modelComputed()){
   ROS_INFO("Model is not computed");
   return;
  }


  //  ros::Time now_getmodel = ros::Time::now();
  Cloud model = qnode.modeler.getModel();
  //  ROS_INFO("getModel: %f ms", (ros::Time::now()-now_getmodel).toSec()*1000);


  if (model.size() == 0){
   ROS_INFO("Model is not computed (and empty)");
   return;
  }

  ros::Time now_color = ros::Time::now();

  // TODO: create only once after model is initialized
  cv::Mat color(model.height, model.width, CV_8UC3);
  color.setTo(0);


  if (qnode.openGL_visualizationActive){
   // TODO: don't copy image
   cv::Mat height = qnode.modeler.getHeightImage();
   heightVisualization(color, height, 0,100, qnode.color_range,NULL);
  }


  if (qnode.water_simulation_active){

   // white surrounding if no height lines are visualized
   if (!qnode.openGL_visualizationActive)
    color.setTo(255);

   if (qnode.water.cols == int(model.width)){
    //waterVisualization(color, qnode.water, 0.005,0.03,NULL);

    cv::Mat alpha; // todo: don't generate each time
    waterVisualizationAlpha(color, alpha,qnode.water, 0.0,0.015,NULL);

    cv::imwrite("water.png", color);
    //    cv::imwrite("alpha.png", alpha);
   }
  }

  Cloud colored = transferColorToMesh(color, model,NULL);

#ifdef DO_TIMING
  ROS_INFO("colorizeCloud: %f ms", (ros::Time::now()-now_color).toSec()*1000.0);
#endif



  ros::Time now_mesh = ros::Time::now();
  float max_length = 0.05; // max edge length of drawn triangle in m
  pcl::PolygonMesh mesh = qnode.mesh_visualizer.createMesh(colored, max_length);
#ifdef DO_TIMING
  ROS_INFO("createMesh: %f ms", (ros::Time::now()-now_mesh).toSec()*1000.0);
#endif

  //  qnode.mesh_visualizer.visualizeMesh(mesh);

  gl_viewer->mesh = &mesh;
  gl_viewer->proj_matrix = qnode.calibrator.proj_Matrix;
//  gl_viewer->setPathWithColors(qnode.planner.getPath(),qnode.planner.getPathColor());

  //  Cloud_n with_normals;
  //  computeNormals(model, with_normals);
  //  gl_viewer->setNormals(with_normals);



  for (std::map<int,Ant>::iterator it = qnode.ants.begin(); it != qnode.ants.end(); ++it){
   it->second.walk(5);
   // todo funding as parameter
   // todo zeitabhaengig
  }


  // show height lines
  if (qnode.show_height_lines){
   ros::Time start_height = ros::Time::now();
   std::vector<Line_collection> height_lines;
   qnode.mesh_visualizer.findHeightLines(mesh, height_lines, qnode.modeler.getMinHeight(), qnode.modeler.getMaxHeight(), 0.02);
   gl_viewer->set_height_lines(height_lines);
   ROS_INFO("Height lines: %f ms", (ros::Time::now()-start_height).toSec()*1000.0);
  }else{
   std::vector<Line_collection> height_lines;
   gl_viewer->set_height_lines(height_lines);
  }


  //  gl_viewer->resize(lb_img.width(),lb_img.height());
  gl_viewer->drawMapImage(false);
  ros::Time now_render = ros::Time::now();
  QPixmap pix2 = gl_viewer->renderPixmap(lb_img.width(),lb_img.height(),true);


#ifdef DO_TIMING
  ROS_INFO("Rendering: %f ms", (ros::Time::now()-now_render).toSec()*1000.0);
#endif



//  pix2.save(QString("fooo.png"),0,100);
//
////  cv::Mat cv_img = qimage2mat(QImage(pix2));
//
//  cv::Mat cv_img = cv::imread("fooo.png");
//
//  cv::circle(cv_img, cv::Point(100,200),30,CV_RGB(255,0,0),-1);
//
//  QImage foo = mat2qimage(cv_img);
//
//  lb_img.setPixmap(pixmap_col.fromImage(foo, 0));
//

  lb_img.setPixmap(pix2);
  lb_img.repaint();

#ifdef DO_TIMING
  ROS_INFO("Showing Model with openGL (total): %f ms", (ros::Time::now()-start_show_openGL).toSec()*1000);
#endif

  frame_update_times.push_back(ros::Time::now());
  if (frame_update_times.size() > hist_length){
   ros::Time earlier = frame_update_times[frame_update_times.size()-1-hist_length];
   double fr = hist_length/(frame_update_times[frame_update_times.size()-1]-earlier).toSec();
   ui.lb_framerate->setText(QString::number(int(fr*100)/100.0));
  }


  // TODO: adapt size of image
  if (qnode.show_height_lines){

   gl_viewer->drawMapImage(true);
   // show image on right image-label
   gl_viewer->initMapSize(ui.lb_img_2->width(),ui.lb_img_2->height(), qnode.modeler.min_x(),qnode.modeler.min_y(),qnode.modeler.getWidth(), qnode.modeler.getHeight());
   //  gl_viewer->resize(ui.lb_img_2->width(),ui.lb_img_2->height());
   QPixmap pix = gl_viewer->renderPixmap(ui.lb_img_2->width(),ui.lb_img_2->height(),true);
   ui.lb_img_2->setPixmap(pix);
   ui.lb_img_2->repaint();
  }



 }

 /*****************************************************************************
 ** Implementation [Slots]
 *****************************************************************************/

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

  gl_viewer->ants = &qnode.ants;

 }


 void MainWindow::got_new_ant(Ant ant){

  ROS_INFO("Got new ant %i",ant.getId());

//  if (ant.getState() == ANT_MOVING){
//   ROS_INFO("Added ant");
//   gl_viewer->ants[ant.getId()] = ant;
//  }

 }

 void MainWindow::ant_demo(){
  // qnode.run_ant_demo();
 }

 void MainWindow::select_marker_area(){

  if (!mousehandler_projector.area_marked()){
   stringstream ss; ss << "Select area on the projector image!";
   qnode.writeToOutput(ss);
   return;
  }



  // undo shrinking!
  cv::Point l1(mousehandler_projector.down.x/image_scale,mousehandler_projector.down.y/image_scale);
  cv::Point l2(mousehandler_projector.up.x/image_scale,mousehandler_projector.up.y/image_scale);

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

 void MainWindow::learn_environment(){
  qnode.detector.reset();
  qnode.modeler.reset();
  qnode.train_background = true;
 }


 void MainWindow::save_kinect_trafo(){
  stringstream ss;

  if (!qnode.calibrator.isKinectTrafoSet()){
   ss << "Kinect Trafo not yet computed!";
   qnode.writeToOutput(ss);
   return;
  }

  qnode.calibrator.saveKinectTrafo(ss);
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
   ss << "Computation of Kinect Trafo successful";
   activateSlider();

  }else{
   ss << "Computation of Kinect Trafo failed";
  }


  qnode.writeToOutput(ss);
 }



 void MainWindow::projection_opencv(){
  float mean_error;
  qnode.calibrator.computeProjectionMatrix_OPENCV(mean_error);
  stringstream ss; ss << "Opencv: " << mean_error;
  qnode.writeToOutput(ss);
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
  // ROS_INFO("Changing checkerboard size");
  qnode.calibrator.checkboard_size = cv::Size(ui.ed_corners_x->text().toInt(), ui.ed_corners_y->text().toInt());
 }





 // void MainWindow::setProjectorPixmap(const QPixmap& pixmap){
 //  ROS_INFO("PROJECTOR PIXMAP");
 //
 //  lb_img.setPixmap(pixmap);
 //  lb_img.repaint();
 // }



 void MainWindow::update_proj_image(){


  assert(qnode.calibrator.projector_image.cols > 0);
  //qimg_proj = Mat2QImage(qnode.calibrator.projector_image);
  lb_img.setPixmap(pixmap_proj.fromImage(qimg_proj, 0));
  lb_img.repaint();


  // draw projector image on right label
  if (qnode.calibrator.projector_image.cols > 0){
   cv::Mat small2;
   cv::resize(qnode.calibrator.projector_image, small2, cv::Size(),image_scale,image_scale, CV_INTER_CUBIC);

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
  ui.lb_static->setText(QString::number(int(secs_since_last_static_image*100)/100.0));
  ui.lb_static->repaint();
 }


 void MainWindow::setLightPos(float z){
  //  ROS_INFO("UPDATING LIGHT POSITION");
  gl_viewer->setLightPos(z);
  std::stringstream ss;
  ss << "light pos " << z;
  qnode.writeToOutput(ss);
 }

 void MainWindow::mouse_new_points(){

  if (mousehandler_points.pts.size() < 4) return;

  vector<cv::Point> normal_sized;

  for (uint i=0; i<mousehandler_points.pts.size(); ++i){
   cv::Point pi = mousehandler_points.pts[i];
   normal_sized.push_back( cv::Point(pi.x/image_scale, pi.y/image_scale));
  }

  if (qnode.current_col_img.rows == 0) return;

  qnode.area_mask = cv::Mat(qnode.current_col_img.rows, qnode.current_col_img.cols, CV_8UC1);
  ROS_INFO("mask: %i %i", qnode.area_mask.rows, qnode.area_mask.cols);
  qnode.area_mask.setTo(0);
  cv::fillConvexPoly(qnode.area_mask, normal_sized, cv::Scalar::all(255));

  mousehandler_points.pts.clear();

  //  cv::imwrite("mask.jpg", qnode.area_mask);
  //  cv::namedWindow("bar");
  //  cv::imshow("bar", qnode.area_mask);
  //  cv::waitKey(10);

 }


 void MainWindow::sl_received_image(){






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
  if (qnode.area_mask.cols == cpy.cols){
   cv::Mat darker = cpy*0.5;
   cv::Mat inv = 255- qnode.area_mask;
   darker.copyTo(cpy,inv);
  }



  cv::resize(cpy, small, cv::Size(),image_scale,image_scale, CV_INTER_CUBIC);

  for (uint i=0; i< mousehandler_points.pts.size(); ++i){
   cv::Point2f p = cv::Point2f(mousehandler_points.pts[i].x,mousehandler_points.pts[i].y);
   cv::circle(small, p, 10, CV_RGB(255,0,0),3);
  }


  qimg_col = Mat2QImage(small);
  ui.lb_kinect_image->setPixmap(pixmap_col.fromImage(qimg_col, 0));
  ui.lb_kinect_image->repaint();

  //qApp->processEvents();
 }

 void MainWindow::loadParameters(){
  if (!qnode.loadParameters()) return;

  sl_threshold_changed(qnode.calibrator.eval_brightness_threshold);
  color_slider_moved(qnode.color_range*100);
  min_dist_changed(qnode.min_dist*1000);
  z_max_changed(qnode.max_dist*100);

  ui.cb_depth_visualization_GL->setChecked(qnode.openGL_visualizationActive);

  ui.cb_depth_visualization->setChecked(qnode.depth_visualization_active);

  gl_viewer->show_texture = qnode.show_texture;
  ui.cb_texture->setChecked(gl_viewer->show_texture);
  ui.cb_water->setChecked(qnode.water_simulation_active);



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

  qnode.saveParameters();
 }

 void MainWindow::z_max_changed(int z_max){
  ui.slider_z_max->setSliderPosition(z_max);
  qnode.max_dist = z_max/100.0; // given in cm
  ui.z_max_label->setText(QString::number(z_max));
  ui.z_max_label->repaint();

  qnode.saveParameters();
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


 void MainWindow::manual_z_changed(int z){
  assert(qnode.calibrator.isKinectTrafoSet());
  float diff = z-manual_z_change;
  qnode.calibrator.translateKinectTrafo(diff/100.0);


  if (qnode.calibrator.projMatrixSet()){
   // TODO: just adapt matrices to new coordinate system if already defined
   qnode.calibrator.proj_Matrix = cv::Mat(0,0,CV_32FC1);
   qnode.calibrator.hom_CV = cv::Mat(0,0,CV_32FC1);

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
  qnode.calibrator.setInputImage(qnode.current_col_img);
  qnode.calibrator.setInputCloud(qnode.current_cloud);

  if(!qnode.calibrator.findCheckerboardCorners()){
   qnode.writeToOutput(sstream("Could not detect Marker!"));
   return;
  }



  qnode.calibrator.storeCurrentObservationPairs();


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
   cloud_msg->header.frame_id = "/openni_rgb_optical_frame";
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

  // qnode.calibrator.saveObservations();
  //  qnode.calibrator.loadObservations();

  float mean_error;
  if (qnode.calibrator.computeProjectionMatrix(mean_error)){
   msg << "Projection Matrix computed with mean reprojection error of " << mean_error << "pixels ";
   // qnode.calibrator.computeProjectionMatrix_OPENCV(mean_error);
   // cout << "opencv calib: mean of " << mean_error << endl;

   cv::Mat K,R,t;

   cv::Mat rx,ry,rz;
   vector<double> angles;

   // cv::Point3d angles;

   cv::decomposeProjectionMatrix(qnode.calibrator.proj_Matrix,K,R,t,rx,ry,rz,angles);

   t /= t.at<double>(3);
   K /= K.at<double>(2,2);

   msg << endl << "translation: " << t.at<double>(0) << " " << t.at<double>(1) << " " << t.at<double>(2) << endl;
   msg << "angles: " << angles[0] << " " << angles[1] << " " << angles[2];



  }else{
   msg << "Could not compute Projection Matrix (not enough points or to little variation in z-direction)";
  }

  qnode.writeToOutput(msg);
 }

 void MainWindow::save_projection_matrix(){
  sstream msg;
  qnode.calibrator.saveProjectionMatrix(msg);
  qnode.writeToOutput(msg);
 }

 void MainWindow::save_homography(){
  sstream msg;
  qnode.calibrator.saveHomographyCV(msg);
  qnode.writeToOutput(msg);
 }


 void MainWindow::show_height_lines(bool status){
  qnode.show_height_lines = status;
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
 }

 void MainWindow::gl_visualization_toggled(bool status){
  qnode.openGL_visualizationActive = status;
  qnode.saveParameters();
 }

 void MainWindow::user_interaction_toggled(bool status){
  qnode.user_interaction_active = status;
 }

 void MainWindow::foreGroundVisualizationToggled(bool status){

  //  min_dist_changed(ui.slider_mindist->value());
  //  z_max_changed(ui.slider_z_max->value());
  //  color_slider_moved(ui.slider_color->value());

  qnode.foreGroundVisualizationActive = status;
 }

 void MainWindow::process_events(){
  //  ROS_INFO("XXXXXXXXXXXXXX processing");
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
  if (qnode.calibrator.removeLastObservations())
   qnode.writeToOutput(sstream("Removed last image"));
  else
   qnode.writeToOutput(sstream("No image to remove"));
 }



 void MainWindow::restart_water_simulation(){
  qnode.restart_modeler = true;
 }


 /*****************************************************************************
 ** Implemenation [Slots][manually connected]
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

