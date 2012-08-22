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



  // open label fullscreen on secondary monitor
  lb_img.setParent(NULL);
  QRect screenres = QApplication::desktop()->screenGeometry(2);

  //  cout << "foo" << endl;
  //  ROS_INFO("screenres 1: %i %i", screenres.x(), screenres.y());


  // z_max_changed(ui.slider_z_max->value());

  // min_dist_changed(ui.slider_mindist->value());


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

 }

 MainWindow::~MainWindow() {

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


 QImage Mat2QImage(const cv::Mat3b &src) {
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


 void MainWindow::update_proj_image(){


  assert(qnode.calibrator.projector_image.cols > 0);
  qimg_proj = Mat2QImage(qnode.calibrator.projector_image);
  lb_img.setPixmap(pixmap_proj.fromImage(qimg_proj, 0));


  // draw projector image on right label
  if (qnode.calibrator.projector_image.cols > 0){
   cv::Mat small2;
   cv::resize(qnode.calibrator.projector_image, small2, cv::Size(),image_scale,image_scale, CV_INTER_CUBIC);

   if (mousehandler_projector.area_marked())
    cv::rectangle(small2,  mousehandler_projector.down, mousehandler_projector.move, mousehandler_projector.area_marked()?CV_RGB(0,255,0):CV_RGB(255,0,0),2);


   qimg_p = Mat2QImage(small2);
   ui.lb_img_2->setPixmap(pixmap_p.fromImage(qimg_p, 0));
  }



 }


 // Calibration Evaluation
 void MainWindow::sl_threshold_changed(int threshold){
  qnode.calibrator.eval_brightness_threshold = threshold;
  ui.lb_brightness->setText( QString::number(threshold));
  ui.lb_brightness->repaint();
 }

 void MainWindow::detect_disc(){  qnode.eval_projection();  }



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

  //    cv::namedWindow("foo");
  //    cv::imshow("foo",cpy);
  //    cv::waitKey(100);

 }


 void MainWindow::min_dist_changed(int min_dist){
  qnode.min_dist = min_dist/1000.0;
  ui.lb_mindist->setText(QString::number(min_dist));
 }

 void MainWindow::z_max_changed(int z_max){
  qnode.visual_z_max = z_max/100.0; // given in cm
  ui.lb_z->setText(QString::number(z_max));
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


 void MainWindow::user_interaction_toggled(bool status){
  qnode.user_interaction_active = status;
 }

 void MainWindow::foreGroundVisualizationToggled(bool status){
  qnode.foreGroundVisualizationActive = status;
 }

 void MainWindow::pattern_auto_size_toggled(bool status){
  pattern_size_auto = status;

  if (pattern_size_auto)
   qnode.writeToOutput(sstream("Pattern Size: auto"));
  else
   qnode.writeToOutput(sstream("Pattern Size: user defined"));

 }

 void MainWindow::depth_visualzation_toggled(bool status){
  qnode.depth_visualization_active = status;

 }


 void MainWindow::delete_last_img(){
  if (qnode.calibrator.removeLastObservations())
   qnode.writeToOutput(sstream("Removed last image"));
  else
   qnode.writeToOutput(sstream("No image to remove"));
 }

 // void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
 //  //  bool enabled;
 //  //  enabled = ( state == 0 );
 //  //  ui.line_edit_master->setEnabled(enabled);
 //  //  ui.line_edit_host->setEnabled(enabled);
 //  //ui.line_edit_topic->setEnabled(enabled);
 // }

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

 void MainWindow::on_actionAbout_triggered() {
  QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
 }

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

