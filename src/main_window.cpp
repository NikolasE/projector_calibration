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
  image_scale = 0.5;

  ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
  QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

  ReadSettings();
  setWindowIcon(QIcon(":/images/icon.png"));
  ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));


  QObject::connect(&qnode, SIGNAL(received_col_Image()), this, SLOT(sl_received_image()));


//  QObject::connect(&ui.ed_corners_x, SINAL())



  /*********************
   ** Logging
   **********************/
  ui.view_logging->setModel(qnode.loggingModel());
  QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

  /*********************
   ** Auto Start
   **********************/


  //  int a = 123465;//qnode.calibrator.checkboard_size.height;
  //
  //  ROS_INFO("Height: %i", a);
  //
  //
  //  QString s = ui.ed_proj_size_y->text();
  //
  //  std::cout << s.toFloat() << std::endl;


  ui.lb_img_2->installEventFilter(&mouse_handler);


  ui.ed_proj_size_x->setText(QString::number(qnode.calibrator.proj_size.width));
  ui.ed_proj_size_y->setText(QString::number(qnode.calibrator.proj_size.height));

  ui.ed_corners_x->setText(QString::number(qnode.calibrator.checkboard_size.width));
  ui.ed_corners_y->setText(QString::number(qnode.calibrator.checkboard_size.height));

 }

 MainWindow::~MainWindow() {}

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

  if (!mouse_handler.area_marked()){
   ROS_INFO("Select area on the projector image!");
   return;
  }

  // undo shrinking!
  cv::Point l1(mouse_handler.down.x/image_scale,mouse_handler.down.y/image_scale);
  cv::Point l2(mouse_handler.up.x/image_scale,mouse_handler.up.y/image_scale);

  qnode.calibrator.projectSmallCheckerboard(l1,l2);

 }




 void MainWindow::pattern_size_changed(){

  ROS_INFO("Changing checkerboard size");
  qnode.calibrator.checkboard_size = cv::Size(ui.ed_corners_x->text().toInt(), ui.ed_corners_y->text().toInt());
 }


 void MainWindow::foobar(){
  qnode.foo++;
  qnode.writeFooToList();
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


 void MainWindow::sl_received_image(){
  if (qnode.current_col_img.cols == 0)
   return;

  // draw kinect image on left label
  cv::Mat small;
  cv::resize(qnode.current_col_img, small, cv::Size(),image_scale,image_scale, CV_INTER_CUBIC);
  QImage qimg = Mat2QImage(small);
  QPixmap pixmap;
  ui.lb_kinect_image->setPixmap(pixmap.fromImage(qimg, 0));


  // draw projector image on right label
  if (qnode.calibrator.projector_image.cols > 0){
   cv::Mat small2;
   cv::resize(qnode.calibrator.projector_image, small2, cv::Size(),image_scale,image_scale, CV_INTER_CUBIC);

   //if (mouse_selection_active){
    //ROS_INFO("From %i %i", mouse_handler.down.x, mouse_handler.down.y);
     cv::rectangle(small2,  mouse_handler.down, mouse_handler.move, mouse_handler.area_marked()?CV_RGB(0,255,0):CV_RGB(255,0,0),2);
    //}

   qimg = Mat2QImage(small2);
   ui.lb_img_2->setPixmap(pixmap.fromImage(qimg, 0));
  }
 }


 void MainWindow::show_fullscreen_pattern(){

  qnode.calibrator.projectFullscreenCheckerboard();
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
  //  QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
  //  QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
  //  //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
  //  ui.line_edit_master->setText(master_url);
  //  ui.line_edit_host->setText(host_url);
  //  //ui.line_edit_topic->setText(topic_name);
  //  bool remember = settings.value("remember_settings", false).toBool();
  //  ui.checkbox_remember_settings->setChecked(remember);
  //  bool checked = settings.value("use_environment_variables", false).toBool();
  //  ui.checkbox_use_environment->setChecked(checked);
  //  if ( checked ) {
  //   ui.line_edit_master->setEnabled(false);
  //   ui.line_edit_host->setEnabled(false);
  //   //ui.line_edit_topic->setEnabled(false);
  //  }
 }

 void MainWindow::WriteSettings() {
  QSettings settings("Qt-Ros Package", "projector_calibration");
  //  settings.setValue("master_url",ui.line_edit_master->text());
  //  settings.setValue("host_url",ui.line_edit_host->text());
  //settings.setValue("topic_name",ui.line_edit_topic->text());
  //  settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
  settings.setValue("geometry", saveGeometry());
  settings.setValue("windowState", saveState());
  //  settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

 }

 void MainWindow::closeEvent(QCloseEvent *event)
 {
  WriteSettings();
  QMainWindow::closeEvent(event);
 }

}  // namespace projector_calibration

