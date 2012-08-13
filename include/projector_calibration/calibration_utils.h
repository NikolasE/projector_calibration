/*
 * calibration.h
 *
 *  Created on: Mar 1, 2012
 *      Author: Nikolas Engelhard
 */

#ifndef CALIBRATION_H_
#define CALIBRATION_H_

#include "type_definitions.h"
#include <opencv/cv.h>
#include <pcl/common/transform.h>


Cloud computeMean(const std::vector<Cloud>& clouds);

/*
 * returns all points the new cloud that have a smaller z-value than the corresponding pixel in the refernce cloud
 *
 */
Cloud removeMean(const Cloud& reference, const Cloud cloud, float max_dist, std::vector<cv::Point2i>* valids = NULL);


void add(pcl_Point& a,const pcl_Point& b);
void div(pcl_Point& a, float d);
float norm(const pcl_Point& p);
pcl_Point setLength(const pcl_Point& p, float s);

void computeTransformationFromYZVectorsAndOrigin(const Eigen::Vector3f& y_direction, const Eigen::Vector3f& z_axis,
  const Eigen::Vector3f& origin, Eigen::Affine3f& transformation);



void project3D(const cv::Point2f px, const cv::Mat P, float W,  cv::Point3f& out);

float dist(pcl_Point A, pcl_Point B);

void projectCloudIntoProjector(const Cloud& cloud, const cv::Mat& P, cv::Mat& img);


bool loadMat(const std::string path, const std::string filename, cv::Mat& mat);
bool saveMat(const std::string path, const std::string filename, const cv::Mat& mat);

void applyMaskOnCloud(const cv::Mat& mask, const Cloud& in, Cloud& out);
void applyMaskOnCloud(const cv::Mat& mask, const pcl::PointCloud<pcl::Normal>& in, pcl::PointCloud<pcl::Normal>& out);


void sampleCloudWithNormals(const Cloud& points, const Cloud_n& normals, Cloud& points_out,Cloud_n& normals_out, uint step, cv::Mat* mask = NULL);


// p_ = H*p
void applyHomography(const cv::Point2f& p,const cv::Mat& H, cv::Point2f& p_);

void applyPerspectiveTrafo(const cv::Point3f& p,const cv::Mat& P, cv::Point2f& p_);
cv::Point2f applyPerspectiveTrafo(const cv::Point3f& p,const cv::Mat& P);

cv::Point2f applyPerspectiveTrafo(const Eigen::Vector3f& p,const cv::Mat& P);


cv::Point2f applyPerspectiveTrafo(const pcl_Point& p, const cv::Mat& P);

void scaleCloud(const Cloud& pts, cv::Mat& U, Cloud& transformed);
void scalePixels(const std::vector<cv::Point2f>& pxs,cv::Mat& T, std::vector<cv::Point2f>& transformed);


// returns false if mean error after transformation is above max_dist
bool computeTransformationFromPointclouds(const Cloud& fixed, const Cloud& moved, Eigen::Affine3f& trafo, float max_dist = 0.05);


// TODO: use this function in scaleCloud to prevent doubled code
/*
 * Compute center of pointcloud as column-vector
 * if out!=NULL it contains the demeaned pointcloud
 */
void centerPointCloud(const Cloud& in, cv::Mat& mean, Cloud* out = NULL);


bool saveAffineTrafo(const Eigen::Affine3f& M, const char* filename);
bool loadAffineTrafo(Eigen::Affine3f& M, const char* filename);

void printTrafo(const Eigen::Affine3f& M);

void update_min_filtered_cloud(Cloud& min_cloud, const Cloud& current);

#endif /* CALIBRATION_H_ */
