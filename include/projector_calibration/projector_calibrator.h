/*
 * projector_calibrator.h
 *
 *  Created on: May 9, 2012
 *      Author: Nikolas Engelhard
 */

#ifndef PROJECTOR_CALIBRATOR_H_
#define PROJECTOR_CALIBRATOR_H_

#include "rgbd_utils/type_definitions.h"
#include <tf/transform_broadcaster.h>

#include <visualization_msgs/Marker.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <pcl/common/transform.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <stack>
#include "fstream"
#include <image_geometry/pinhole_camera_model.h>
#include "stat_eval.h"

typedef cv::Rect_<float> cv_RectF;



enum INTERPOLATION_TYPE {INTER_NON, INTER_SIMPLE,INTER_BI};




class Projector_Calibrator {


  // trafo cloud s.t. checkerboard is z=0,  middle of board at x=y=0
  // the first trafo is stored and used for all following frames
  Eigen::Affine3f kinect_trafo;
  bool kinect_trafo_valid;

  cv::Mat input_image; // rgb image of kinect
  cv::Mat gray; // gray version of kinect image

  /// tilt of kinect (rotation around optical axis)
  float kinect_tilt_angle_deg;
  bool kinect_orientation_valid;

  // fit a plane into the pointcloud
  float fitPlaneToCloud(const Cloud& cloud, Eigen::Vector4f& model);


  // draw a checkerboard with given number of internal corners on the image and store the corners
  // void drawCheckerboard(cv::Mat& img, const cv::Size size, std::vector<cv::Point2f>& corners_2d);
  void drawCheckerboard(cv::Mat& img,cv::Point l1, const cv::Point l2, const cv::Size size, std::vector<cv::Point2f>& corners_2d);

  // Position of internal checkerboard corners
  std::vector<cv::Point2f> current_projector_corners;


  std::string hom_cv_filename, hom_svd_filename, proj_matrix_filename, kinect_trafo_filename;

  // bool saveMat(const std::string name, const std::string filename, const cv::Mat& mat);
  // bool loadMat(const std::string name, const std::string filename, cv::Mat& mat);

  bool setupImageProjection(const cv_RectF& wall_area, const cv::Size& img_size);

  bool setupImageProjection(float width_m, float height_m, float off_x_m, float off_y_m, const cv::Size& img_size);
  bool setupImageProjection(float width_m, float off_x_m, float off_y_m, const cv::Size& img_size);

  const static int unused_pixels_rows = 25;

  int max_checkerboard_width,max_checkerboard_height;

  image_geometry::PinholeCameraModel depth_cam_model; /// model for depth camera

public:
  bool depth_cam_model_set;

  void setDepthCameraModel(const sensor_msgs::CameraInfoConstPtr& cam_info){
    depth_cam_model.fromCameraInfo(cam_info);
    depth_cam_model_set = true;
  }


  /// returns current Kinect Transformation
  Eigen::Affine3f getCameraTransformation(){
    assert(kinect_trafo_valid);
    return kinect_trafo;
  }

  /// point cloud from kinect
  Cloud input_cloud;

  /// point cloud in world-frame
  Cloud cloud_moved;

  /// Projection matrix of projector computed with DLT-Approach
  cv::Mat proj_Matrix;

  /// Projection matrix of projector computed with cv::calibrateCamera
  cv::Mat proj_matric_cv;



  /// size of squares of printed checkerboard (deprecated)
  float printed_marker_size_mm;

  /// number of inner corners of the board
  cv::Size checkboard_size;

  /// size of the projector image in pixels
  cv::Size proj_size;

  /// image on kinect image showing the area where the first checkerboard was found in white 8UC1, with board: 255, rest: 0
  cv::Mat mask;

  /// Homography computed via OpenCV
  cv::Mat  hom_CV;

  /// Homography computed via DLT
  cv::Mat hom_SVD;



  /// brightness threshold using detection of calibration disc
  int eval_brightness_threshold;


  cv::Mat projector_position; /// position of projector in the world system
  cv::Mat rotMatrix; /// rotation projector in the world system
  cv::Mat camera_matrix; /// internal camera parameters of projector

  cv::Mat rotMatrix_CV; /// rotation projector in the world system
  cv::Mat projector_position_CV; /// position of projector in the world system




  float eval_projection_matrix_Checkerboard(Cloud& corners, std::stringstream& ss);

  bool eval_projection_matrix_disc(Cloud& mean_c, std::stringstream& ss,  const cv::Mat* mask = NULL);

  // Cloud projectionAreaCorners;
  bool getProjectionAreain3D(std::vector<cv::Mat>& corners);
  bool getProjectionAreain3D(Cloud& corners);

  bool saveObservations();
  bool loadObservations();



  /// true if all stored matrices should be loaded during startup
  bool load_everything_on_startup;


  void restartCalibration();
  bool removeLastObservations();

  int getCurrentProjectorCornerCnt(){ return int(current_projector_corners.size());}

  /// get number of 3d/2d-pairs
  int getNumPairs(){return int(observations_3d.size());}

  void translateKinectTrafo(float dz);
  void rotateKinectTrafo(float dyaw);


  bool saveKinectTrafo(std::stringstream& msg);

  /// pixel coordinates of the detected checkerboard corners
  std::vector<cv::Point2f> detected_corners;

  bool loadKinectTrafo(std::stringstream& msg);
  bool saveHomographyCV(std::stringstream& msg);
  bool saveProjectionMatrix(std::stringstream& msg);


  /// remove all point from the input cloud where mask!=255
  void applyMaskOnInputCloud(Cloud& out);


  /// list of 3d-observations (in the world-frame) of the checkerboard corners
  Cloud observations_3d;

  /// list of all chessboard corners from the projector image (corresponding to the observations_3d)
  std::vector<cv::Point2f> corners_2d;


  void getCheckerboardArea(std::vector<cv::Point2i>& pts);






  /// image to be shown by the projector
  cv::Mat projector_image;



  /// a simple image for debugging
  cv::Mat test_img;

  /// apply on image
  cv::Mat warp_matrix;

  /// largest rectangular projection area that can be shown
  cv_RectF optimal_projection_area;

  bool findOptimalProjectionArea(float ratio, cv_RectF& rect, std::stringstream& msg);
  bool findOptimalProjectionArea2(cv::Mat::MSize img_px_size, std::stringstream& msg);


  /// true if projection Matrix or Homography is set
  bool projMatorHomSet(){return projMatrixSet() ||   homOpenCVSet() || homSVDSet();}


  /// returns point to Cloud in world-frame
  Cloud* getTransformedCloud(){return &cloud_moved;}

  void initFromFile(std::stringstream& msg);

  /// true if projection matrix was set
  bool projMatrixSet(){ return proj_Matrix.cols > 0;}
  /// true if Homography (using OpenCV) was computed
  bool homOpenCVSet(){ return hom_CV.cols > 0;}

  /// true if Homography (computed using DLT) was computed
  bool homSVDSet(){ return hom_SVD.cols > 0;}

  /// true if warping matrix was set
  bool warpMatrixSet(){ return warp_matrix.cols > 0;}

  //   if (calibrator.imageProjectionSet()){
  //#ifdef SHOW_TEST_IMAGE
  //    calibrator.showUnWarpedImage(calibrator.test_img);
  //#else
  //    system("xwd -root | convert - /tmp/screenshot.jpg");
  //    cv::Mat screen = cv::imread("/tmp/screenshot.jpg");
  //    cv::Mat primary_screen = screen(cv::Range(0,mainScreenSize.height), cv::Range(0,mainScreenSize.width));
  //    calibrator.showUnWarpedImage(primary_screen);
  //
  //#endif
  //   }


  // bool imageProjectionSet() { return warp_matrix.cols > 0; }

  // save 3d positions (in wall-frame) of the last checkerboard detection
  bool storeCurrentObservationPairs(INTERPOLATION_TYPE inter_type = INTER_BI, int interpolation_size = 3);


  pcl_Point getInterpolatedPoint(cv::Point2f pos);

  // stores the number of detected corners in each image to be able to remove images
  std::vector<int> number_of_features_in_images;

  /// true if orientation of Kinect is valid
  bool isKinectOrientationSet(){return kinect_orientation_valid;}

  /**
   * @brief setKinectOrientation new orientation angle of Kinect
   * @param angle_deg new tilt angle of Kinect
   */
  void setKinectOrientation(float angle_deg){kinect_tilt_angle_deg = angle_deg;kinect_orientation_valid = true;}


  bool computeKinectTransformation(std::stringstream& msg);

  /// true if transformation from Kinect to World-Frame is valid
  bool isKinectTrafoSet(){return kinect_trafo_valid;}

  /// true if mask is set
  bool mask_valid() {return mask.cols > 0;}

  bool findCheckerboardCorners();

  void createMaskFromDetections();

  /**
   * @brief sets new input image
   * @param image new image
   * @see input_image
   */
  void setInputImage(cv::Mat& image){input_image = image;}
  void setInputCloud(const Cloud& cloud);

  bool publishWorldFrame(const std::string& kinect_frame,const std::string& world_frame);

  void showUnWarpedImage(const cv::Mat& img);


//  void showUnWarpedImage(){
//    assert(test_img.data);
//    showUnWarpedImage(test_img);
//  }


  cv::Mat* getTestImg(){return &test_img;}

  bool computeProjectionMatrix(float& mean_error, bool do_scaling);
  bool computeProjectionMatrix_OPENCV(float& mean_error, bool with_distorion);

  bool computeHomography_OPENCV(float& mean_error);
  bool computeHomography_SVD();

  void projectFullscreenCheckerboard();
  void projectSmallCheckerboard(cv::Point l1, cv::Point l2);
  void projectUniformBackground(bool white);

  Cloud visualizePointCloud();

  void createProjektorMarker(visualization_msgs::Marker& marker);
  void createProjektorMarker_CV(visualization_msgs::Marker& marker);

  Projector_Calibrator();



};




#endif /* PROJECTOR_CALIBRATOR_H_ */
