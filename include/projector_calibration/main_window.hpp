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
#include "ui_img_window.h"
#include "qnode.hpp"
#include <qevent.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>


#include "projector_calibration/calib_eval.h"

/*****************************************************************************
 ** Namespace
 *****************************************************************************/

typedef std::stringstream sstream;


namespace projector_calibration {


 class MouseHandler_points : public QObject{
  Q_OBJECT
 public:

  std::vector<cv::Point2i> pts;

  MouseHandler_points( QObject *parent = 0 ) : QObject( parent ) {}

  Q_SIGNALS:
  void new_point();

 protected:
  bool eventFilter( QObject *dist, QEvent *event )
  {
   if( event->type() == QMouseEvent::MouseButtonRelease)
    {
    QMouseEvent *mouseEvent = static_cast<QMouseEvent*>( event );
    pts.push_back(cv::Point2i(mouseEvent->x(), mouseEvent->y()));
    ROS_INFO("new point: %i %i", pts[pts.size()-1].x,pts[pts.size()-1].y);

    Q_EMIT new_point();
    }
   return false;
  }
 };


 class MouseHandler : public QObject{
  Q_OBJECT
 public:



  MouseHandler( QObject *parent = 0 ) : QObject( parent ) {
   //   active = true;
   up = cv::Point2i(-1,-1);
  }

  cv::Point2i down;
  cv::Point2i move;
  cv::Point2i up;

  bool area_marked(){return move.x > 0;}

  void reset(){
   up = move = cv::Point2i(-1,-1);
  }

  //  bool active;
  Q_SIGNALS:
  void redraw_image();
  void marker_area_changed();


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

    Q_EMIT redraw_image();

    }

   if( event->type() == QMouseEvent::MouseButtonRelease)
    {
    //    ROS_INFO("release");

    QMouseEvent *mouseEvent = static_cast<QMouseEvent*>( event );
    up = cv::Point2i(mouseEvent->x(), mouseEvent->y());
    move = up;
    Q_EMIT redraw_image();


    if (abs(down.x-up.x) > 10 && abs(down.y-up.y) > 10)
     Q_EMIT marker_area_changed();

    }

   if( event->type() == QMouseEvent::MouseMove){
    //    ROS_INFO("move");
    QMouseEvent *mouseEvent = static_cast<QMouseEvent*>( event );
    move = cv::Point2i(mouseEvent->x(), mouseEvent->y());
    Q_EMIT redraw_image();
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

  QPixmap pixmap_proj, pixmap_col, pixmap_p;
  QImage qimg_proj, qimg_col, qimg_p;


  QLabel lb_img;
  bool pattern_size_auto;

 public Q_SLOTS:
 /******************************************
  ** Auto-connections (connectQ_SLOTSByName())
  *******************************************/
 void mouse_new_points();
 void on_actionAbout_triggered();
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
 void z_max_changed(int z_max);
 void min_dist_changed(int min_dist);
 void sl_threshold_changed(int threshold);
 void find_projection_area();
 void update_proj_image();
 void learn_environment();

 void user_interaction_toggled(bool);
 void depth_visualzation_toggled(bool);
 void pattern_auto_size_toggled(bool);
 void foreGroundVisualizationToggled(bool);

 // calibration
 void compute_homography();
 void add_new_observation();
 void compute_projection_matrix();
 void save_projection_matrix();
 void save_homography();
 void delete_last_img();

 void projection_opencv();


 void detect_disc();
 /******************************************
  ** Manual connections
  *******************************************/
 void updateLoggingView(); // no idea why this can't connect automatically
 void sl_received_image();
 void pattern_size_changed();



 private:

 cv::Mat small, cpy;

 Ui::MainWindowDesign ui;
 QNode qnode;

 MouseHandler mousehandler_projector;
 MouseHandler_points mousehandler_points;
 // bool mouse_selection_active;

 // only show smaller images on the gui:
 float image_scale;

 };

}  // namespace projector_calibration







#endif // projector_calibration_MAIN_WINDOW_H
