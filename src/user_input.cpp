/*
 * user_input.cpp
 *
 *  Created on: Mar 1, 2012
 *      Author: Nikolas Engelhard
 */

#include "projector_calibration/user_input.h"
#include <pcl/PointIndices.h>
#include "projector_calibration/stat_eval.h"

using namespace std;


// z points into the wall!!
void User_Input::setCloud(const Cloud& cloud, const Cloud& checkerboard_area_3d){

 if (cloud.size() == 0) return;

 cloud_ = cloud;

 // if (checkerboard_area.size() != 4) return;
 //
 // checkerboard_area_3d.clear();
 // for (uint i=0; i<checkerboard_area.size(); ++i){
 //  pcl_Point p = cloud_(checkerboard_area[i].x, checkerboard_area[i].y);
 //  if (!(p.x == p.x)) return;
 //  checkerboard_area_3d.push_back(p);
 // }


 // get points in front of checkerboard
 pcl::ExtractPolygonalPrismData<pcl_Point> prism_extractor;
 prism_extractor.setInputCloud(cloud_.makeShared());
// prism_extractor.setHeightLimits(-0.03,0.3);
 prism_extractor.setHeightLimits(-0.3,0.3);
 prism_extractor.setInputPlanarHull(checkerboard_area_3d.makeShared());


 pcl::PointIndices inlier;
 prism_extractor.segment(inlier);

 // ROS_INFO("Found %zu inlier", inlier.indices.size());


 prism.clear(); prism.reserve(inlier.indices.size());
 for (uint i=0; i<inlier.indices.size(); ++i){
  pcl_Point p = cloud_[inlier.indices[i]];
  if (!(p.x == p.x)) continue;
  prism.push_back(p);
 }

 Cloud::Ptr msg = prism.makeShared();
 msg->header.frame_id = "/openni_rgb_optical_frame";
 msg->header.stamp = ros::Time::now ();
 pub_filtered.publish(msg);

 //processPrismTRIVIAL();

}



bool User_Input::getUserPositionTrivial(cv::Point3f& position){


 if (prism.size() == 0) return false;

 bool user_found = false;

 pcl_Point closest_point;

 float min_dist = -0.10; // everything closer than this is ignored
 float max_dist = -0.20; // everything further away than this is ignored

 closest_point.z = -1e5;
 for (uint i=0; i<prism.size(); ++i){
  pcl_Point p = prism[i];
  if (p.z <= min_dist && p.z > max_dist && p.z > closest_point.z) closest_point = p;
 }

 if (closest_point.z == -1e5){
  // ROS_INFO("No finger");
 }
 else{
  // ROS_INFO("Found finger at %f %f %f", closest_point.x, closest_point.y, closest_point.z);

  position.x = closest_point.x;
  position.y = closest_point.y;
  position.z = 0;// closest_point.z; projection into plane, point of interaction should not depend on projetor pose

  user_found = true;


  if (pub_hand.getNumSubscribers()>0){
   Cloud foo; foo.push_back(closest_point);
   Cloud::Ptr msg = foo.makeShared();
   msg->header.frame_id = "/openni_rgb_optical_frame";
   msg->header.stamp = ros::Time::now ();
   pub_hand.publish(msg);
  }
 }



 return user_found;



}




