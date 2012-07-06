/*
 * user_input.h
 *
 *  Created on: Mar 1, 2012
 *      Author: Nikolas Engelhard
 */

#ifndef USER_INPUT_H_
#define USER_INPUT_H_


#include "projector_calibration/calibration_utils.h"
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include "projector_calibrator.h"
#include <pcl/segmentation/extract_polygonal_prism_data.h>


void projectCloudIntoProjector(const Cloud& cloud, const cv::Mat& P, cv::Mat& img);



struct User_Input {

private:
 ros::NodeHandle * nh;

 ros::Publisher pub_filtered;
 ros::Publisher pub_hand;

 // Projector_Calibrator *calibrator_;

// std::vector<cv::Point2i> checkerboard_area;
// Cloud checkerboard_area_3d;

 Cloud cloud_, prism;

public:
 void setCloud(const Cloud& cloud,const Cloud& checkerboard_area_3d);

 bool getUserPositionTrivial(cv::Point3f& position);


 // TODO: name
 // cloud is in wall-frame
 // void processPrismTRIVIAL();

 void init(){
  nh = new ros::NodeHandle();
  pub_filtered = nh->advertise<Cloud>("user_input_full", 1);
  pub_hand = nh->advertise<Cloud>("user_input_hand", 1);
 }


};




#endif /* USER_INPUT_H_ */
