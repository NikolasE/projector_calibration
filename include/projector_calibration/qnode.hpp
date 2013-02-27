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

#include "projector_calibration/visualization_paramsConfig.h"

#include "rgbd_utils/projector_calibrator.h"

#include "rgbd_utils/calibration_utils.h"
#include "rgbd_utils/surface_modeler.h"
#include "rgbd_utils/type_definitions.h"
#include "rgbd_utils/ants.h"
// #include "/usr/gast/engelhan/ros/rgbd_utils/cfg/cpp/rgbd_utils/path_paramsConfig.h"


#include "rgbd_utils/pinch_detection.h"

#include "water_simulation/simulator_init.h"
#include "water_simulation/simulator_step.h"
#include "water_simulation/msg_source_sink.h"
#include "water_simulation/water_simulation.h"

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv/cv.h>
#include <image_geometry/pinhole_camera_model.h>

#include <QtOpenGL/qgl.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>


namespace projector_calibration {


//#define PRINT_TIMING

class QNode : public QThread {
  Q_OBJECT
public:
  Ant ant;
  std::map<int,Ant> ants;



  //  /// Duration since last static frame
  //  ros::Duration durationSinceLastStaticFrame(){ return (ros::Time::now()-time_of_last_static_frame); }
  //
  //  /// Duration since last static frame in s
  // double secondsSinceLastStaticFrame(){ return durationSinceLastStaticFrame().toSec();}


  ros::Subscriber sub_cam_info, sub_cloud, sub_col_image;

  cv_bridge::CvImagePtr cv_ptr;

  Projector_Calibrator calibrator;

  Detector detector;

  Object_tracker<Grasp,Track<Grasp> > grasp_tracker;
  Object_tracker<Playing_Piece,Track<Playing_Piece> > piece_tracker;
  Object_tracker<Fingertip,Track<Fingertip> > fingertip_tracker;

  PixelEnvironmentModel pixel_modeler;
  Elevation_map elevation_map;
  Mesh_visualizer mesh_visualizer;
  Path_planner planner;

  void run_ant_demo();
  void update_ant(cv::Point goal);

  // uint train_frame_cnt;
  bool train_background;
  bool openGL_visualizationActive;
  bool foreGroundVisualizationActive;
  bool show_texture;
  bool water_simulation_active;
  bool with_path_planning;
  //  bool simulator_initialized;


  void runDetector();
  cv::Mat pixel_foreground;
  cv::Mat current_norm; // depth map in Kinect Frame

  cv::Mat water;

  bool init_watersimulation();
  bool step_watersimulation();
  int water_request_id;

  image_geometry::PinholeCameraModel depth_cam_model;

  bool loadParameters();
  void saveParameters();


  ros::Publisher pub_cloud_worldsystem; // kinect cloud in world frame
  ros::Publisher pub_3d_calib_points; // detected corners in 3d
  ros::Publisher pub_colored_cloud; // detected corners in 3d
  ros::Publisher pub_eval_marker; // center of evaluation disc
  ros::Publisher pub_background; // center of evaluation disc
  ros::Publisher pub_model;
  ros::Publisher pub_foreground;
  ros::Publisher pub_projector_marker;
  ros::Publisher pub_gauss_foreground;
  ros::Publisher pub_detection_foreground,pub_detection_foreground2;
  ros::Publisher pub_hand;
  ros::Publisher pub_path, pub_path_model;
  ros::Publisher pub_pixel_model;
  ros::Publisher pub_ant_model, pub_ant_range;

  cv::Mat current_col_img;
  Cloud current_cloud;

  void cloudCB(const sensor_msgs::PointCloud2ConstPtr& cloud_ptr);
  void depthCamInfoCB(const sensor_msgs::CameraInfoConstPtr& cam_info);
  void imgCloudCB(const sensor_msgs::ImageConstPtr& img_ptr, const sensor_msgs::PointCloud2ConstPtr& cloud_ptr);
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  void paramCallback(const projector_calibration::visualization_paramsConfig& config, uint32_t level);


  float planner_max_dist;
  float planner_scaling_factor;

  bool do_gesture_recognition;

  void writeToOutput(const std::stringstream& msg);
  bool depth_visualization_active;

  bool restart_modeler;

  float min_dist, max_dist;
  float color_range;
  bool show_height_lines;
  /// if distance between current and new height of a cell is larger than this value, its height will not be updated
  float max_update_dist;


  float height_line_distance;

  cv::Mat area_mask;

  cv::Mat objectMask; /// shows last observation of tracks

  float modeler_cell_size;

  bool calibration_active;

  bool update_pixel_model;
  bool update_elevation_map;

  QNode(int argc, char** argv );
  virtual ~QNode();
  bool init();
  bool init(const std::string &master_url, const std::string &host_url);
  void run();


  void disc_evaluation();
  void eval_projection();


  void visualizeTracks(QPixmap* img);
  void visualizePlanner(QPixmap* img);

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
  void model_computed();
  void scene_static(double secs_since_last_static_image);
  //  void newProjectorPixmap(const QPixmap& pixmap);
  void process_events();
  void new_light(float);
  void newAnt(Ant ant);
  void sl_update_ant();

  void sig_grasp_started(cv::Point2f, int);
  void sig_grasp_moved(cv::Point2f, int);
  void sig_grasp_ended(cv::Point2f, int);

  void sig_handvisible(bool visible);

  void visualize_Detections();
  void copy_projector_image();

private:

  int frame_cnt;

  int init_argc;
  char** init_argv;
  QStringListModel logging_model;

  int iterations_per_frame;
  float sim_viscosity;



  bool first_depth_callback;
  cv::Mat last_static_depth_image;

  ros::Time time_of_last_static_frame;
  ros::Duration duration_since_last_static_frame;
  bool current_frame_static;

  int next_ant_id;




};

}  // namespace projector_calibration

#endif /* projector_calibration_QNODE_HPP_ */
