/**
 * @file /include/projector_calibration/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef projector_calibration_QNODE_HPP_
#define projector_calibration_QNODE_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>

#include "rgbd_utils/type_definitions.h"


#include "projector_calibration/projector_calibrator.h"
#include "projector_calibration/user_input.h"

#include "projector_calibration/calib_eval.h"

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv/cv.h>

#include "pinch_recognition/pinch_detection.h"
#include "rgbd_utils/surface_modeler.h"


/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace projector_calibration {

 /*****************************************************************************
  ** Class
  *****************************************************************************/

 class QNode : public QThread {
  Q_OBJECT
 public:

  cv_bridge::CvImagePtr cv_ptr;

  // the actual calibration object
  Projector_Calibrator calibrator;

  Pinch_detector detector;
  const static uint train_frame_cnt = 20;
  bool train_background;
  bool foreGroundVisualizationActive;


  Surface_Modeler modeler;

  Mesh_visualizer mesh_visualizer;

  User_Input* user_input;
  float visual_z_max;

  ros::Publisher pub_cloud_worldsystem; // kinect cloud in world frame
  ros::Publisher pub_3d_calib_points; // detected corners in 3d
  ros::Publisher pub_colored_cloud; // detected corners in 3d
  ros::Publisher pub_eval_marker; // center of evaluation disc
  ros::Publisher pub_background; // center of evaluation disc

  ros::Publisher pub_model;


  cv::Mat current_col_img;
  Cloud current_cloud;
  void imgCloudCB(const sensor_msgs::ImageConstPtr& img_ptr, const sensor_msgs::PointCloud2ConstPtr& cloud_ptr);
  void writeToOutput(const std::stringstream& msg);
  bool user_interaction_active;
  bool depth_visualization_active;
  float min_dist;
  float color_range;

  cv::Mat area_mask;

  float modeler_cell_size;

  QNode(int argc, char** argv );
  virtual ~QNode();
  bool init();
  bool init(const std::string &master_url, const std::string &host_url);
  void run();


  void disc_evaluation();
  void eval_projection();

  /*********************
   ** Logging
   **********************/
  enum LogLevel {
   Debug,
   Info,
   Warn,
   Error,
   Fatal
  };

  QStringListModel* loggingModel() { return &logging_model; }
  void log( const LogLevel &level, const std::string &msg);

  Q_SIGNALS:
  void loggingUpdated();
  void rosShutdown();
  void received_col_Image();
  void update_projector_image();


 private:
  int init_argc;
  char** init_argv;
  QStringListModel logging_model;





 };

}  // namespace projector_calibration

#endif /* projector_calibration_QNODE_HPP_ */
