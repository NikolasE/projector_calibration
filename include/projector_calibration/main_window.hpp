/**
 * @file /include/projector_calibration/main_window.hpp
 *
 * @brief Qt based gui for projector_calibration.
 *
 * @date November 2010
 **/
#ifndef projector_calibration_MAIN_WINDOW_H
#define projector_calibration_MAIN_WINDOW_H

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <qevent.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

/*****************************************************************************
 ** Namespace
 *****************************************************************************/

namespace projector_calibration {


 class MouseHandler : public QObject
 {
 public:
  MouseHandler( QObject *parent = 0 ) : QObject( parent ) {
//   active = true;
   up = cv::Point2i(-1,-1);
  }

  cv::Point2i down;
  cv::Point2i move;
  cv::Point2i up;

  bool area_marked(){return up.x > 0;}

  void reset(){
   up = cv::Point2i(-1,-1);
  }

//  bool active;

 protected:
  bool eventFilter( QObject *dist, QEvent *event )
  {
//   if (!active) return false;

   if( event->type() == QMouseEvent::MouseButtonPress)
    {
    QMouseEvent *mouseEvent = static_cast<QMouseEvent*>( event );
//    ROS_INFO("down at %i %i", mouseEvent->x(), mouseEvent->y());

    down = cv::Point2i(mouseEvent->x(), mouseEvent->y());
    move = down;
    up = cv::Point2i(-1,-1);

    }

   if( event->type() == QMouseEvent::MouseButtonRelease)
    {
//    ROS_INFO("release");

    QMouseEvent *mouseEvent = static_cast<QMouseEvent*>( event );
    up = cv::Point2i(mouseEvent->x(), mouseEvent->y());
    move = up;
    }

   if( event->type() == QMouseEvent::MouseMove){
//    ROS_INFO("move");
    QMouseEvent *mouseEvent = static_cast<QMouseEvent*>( event );
    move = cv::Point2i(mouseEvent->x(), mouseEvent->y());
   }


   return false;
  }
 };




 /*****************************************************************************
  ** Interface [MainWindow]
  *****************************************************************************/
 /**
  * @brief Qt central, all operations relating to the view part here.
  */
 class MainWindow : public QMainWindow {
  Q_OBJECT

 public:
  MainWindow(int argc, char** argv, QWidget *parent = 0);
  ~MainWindow();

  void ReadSettings(); // Load up qt program settings at startup
  void WriteSettings(); // Save qt program settings when closing

  void closeEvent(QCloseEvent *event); // Overloaded function
  void showNoMasterMessage();

  void activateSlider();

  float manual_z_change;
  float manual_yaw_change;

 public Q_SLOTS:
 /******************************************
  ** Auto-connections (connectQ_SLOTSByName())
  *******************************************/
 void on_actionAbout_triggered();
 //void on_checkbox_use_environment_stateChanged(int state);
 void show_fullscreen_pattern();
 void select_marker_area();
 void project_black_background();
 void project_white_background();
 void marker_size_changed();
 void load_kinect_trafo_from_file();
 void compute_trafo();
 void save_kinect_trafo();
 void manual_z_changed(int z);
 void manual_yaw_changed(int yaw);

 /******************************************
  ** Manual connections
  *******************************************/
 void updateLoggingView(); // no idea why this can't connect automatically
 void sl_received_image();
 void pattern_size_changed();




 private:
 Ui::MainWindowDesign ui;
 QNode qnode;

 MouseHandler mouse_handler;
// bool mouse_selection_active;

 // only show smaller images on the gui:
 float image_scale;

 };

}  // namespace projector_calibration







#endif // projector_calibration_MAIN_WINDOW_H
