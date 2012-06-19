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

#include "projector_calibration/projector_calibrator.h"
#include "projector_calibration/type_definitions.h"

 #include <message_filters/sync_policies/approximate_time.h>

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


  // the actual calibration object
  Projector_Calibrator calibrator;

  int foo;

  cv::Mat current_col_img;
  Cloud current_cloud;
  void writeFooToList();
  void writeToOutput(const std::stringstream& msg);


  QNode(int argc, char** argv );
  virtual ~QNode();
  bool init();
  bool init(const std::string &master_url, const std::string &host_url);
  void run();

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

  signals:
  void loggingUpdated();
  void rosShutdown();



 private:
  int init_argc;
  char** init_argv;
  ros::Publisher chatter_publisher;
  QStringListModel logging_model;





 };

}  // namespace projector_calibration

#endif /* projector_calibration_QNODE_HPP_ */
