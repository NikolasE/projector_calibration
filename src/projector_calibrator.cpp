/*
 * projector_calibrator.cpp
 *
 *  Created on: May 9, 2012
 *      Author: engelhan
 */


using namespace std;

#include "projector_calibration/projector_calibrator.h"
#include "rgbd_utils/calibration_utils.h"



void Projector_Calibrator::eval_projection_matrix_Checkerboard(Cloud& corners, std::stringstream& ss, const cv::Mat* area_mask){

 corners.clear();

 if (input_image.rows == 0){  ss << "can't find corners on empty image!"; return; }

 ROS_INFO("Looking for checkerboard with %i %i corners img: %i %i", checkboard_size.width, checkboard_size.height, input_image.cols, input_image.rows);

 if (!cv::findChessboardCorners(input_image, checkboard_size,detected_corners, CV_CALIB_CB_ADAPTIVE_THRESH)) {
  ss << "Could not find a checkerboard!";
  return;
 }

 cv::cvtColor(input_image, gray, CV_BGR2GRAY);
 cv::cornerSubPix(gray, detected_corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 50, 0.01));



 // interpolate depth at adjacent pixels and project into projector:

 // projector_image.setTo(0);


 cv::Mat col_cpy; input_image.copyTo(col_cpy);


 float error = 0;

 ROS_INFO("detected: %zu, corners: %zu",detected_corners.size(), corners_2d.size() );

 // assert(detected_corners.size() == current_projector_corners.size());

 for (uint i=0; i< detected_corners.size(); ++i){

  int valid = 0;
  pcl_Point mean; mean.x = mean.y = mean.z = 0;

  cv::Point2f px = detected_corners[i];

  cv::circle(col_cpy, px, 3, CV_RGB(255,0,0),1);

  int l = 0;
  for (int x = px.x-l;x <= px.x+l ; ++x )
   for (int y = px.y-l;y <= px.y+l ; ++y )
    {
    if (x < 0 || x > int(cloud_moved.width) || y < 0 || y > int(cloud_moved.height) ) continue;

    pcl_Point p = cloud_moved.at(x, y);
    if (p.x != p.x)
     continue;

    valid++;
    add(mean, p);

    }

  if (valid == 0){
   ss << "no depth value on corner " << i;
   continue;
  }

  div(mean, valid);

  cv::Point2f pr = applyPerspectiveTrafo(mean, proj_Matrix);


  cv::Point2f px_true = current_projector_corners[i];
//  ROS_INFO("true: %f %f", px_true.x, px_true.y);

  float e= sqrt(pow(px_true.x-pr.x,2)+pow(px_true.y-pr.y,2));

//  ROS_INFO("error %i: %f", i,e);

  error+=e;


  int r = 20;
  cv::line(projector_image, cv::Point(pr.x,pr.y-r),cv::Point(pr.x,pr.y+r),CV_RGB(0,0,255),3);
  cv::line(projector_image, cv::Point(pr.x-r,pr.y),cv::Point(pr.x+r,pr.y),CV_RGB(0,0,255),3);


  cv::circle(projector_image, pr, r, CV_RGB(255,0,0), 4);
  cv::circle(projector_image, pr, 5, CV_RGB(0,255,255), -1);
  cv::circle(projector_image, pr, 3, CV_RGB(255,255,0), -1);

  corners.push_back(mean);
 }

 error /= current_projector_corners.size();


 ss << "mean reprojection error: " << error;

 ROS_INFO("Mean error: %e", error);

// cv::namedWindow("Detected Corners");
// cv::imshow("Detected Corners", col_cpy);
// cv::waitKey(10);


}



void Projector_Calibrator::eval_projection_matrix_disc(Cloud& corners, std::stringstream& ss, const cv::Mat* area_mask){

 corners.clear();


 cv::Mat bw;


 cv::namedWindow("input_image");
 cv::imshow("input_image", input_image);

 cv::waitKey(1000);


 cv::cvtColor(input_image, bw, CV_BGR2GRAY);

 cv::Mat cpy2;
 bw.copyTo(cpy2);


 if (area_mask && area_mask->cols == input_image.cols)
  {
  cv::Mat foo;
  bw.copyTo(foo, *area_mask);
  bw = foo;
  }

 cv::Mat bright;

 //  ROS_INFO("threshold: %i",calibrator.eval_brightness_threshold );

 cv::threshold(bw, bright, eval_brightness_threshold, 255,  CV_THRESH_BINARY);

 cv::dilate(bright,bright, cv::Mat());

 cv::Mat can;

 cv::Canny(bright, can, 2, 10);



 cv::Mat col;
 input_image.copyTo(col);



 /*
   vector<cv::Vec3f> circles;
   cv::HoughCircles(can, circles, CV_HOUGH_GRADIENT, 1, can.rows / 4, 200, 10);
   for (size_t i = 0; i < circles.size(); i++)
    {


     cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
     int radius = cvRound(circles[i][2]);

     ROS_INFO("radius: %f", radius);
     cv::circle(col, center, 1, cv::Scalar(255, 0, 0), -1, 8, 0);
     cv::circle(col, center, radius, cv::Scalar(0, 0, 255), 1, 8, 0);

     cv::circle(can, center, 1, cv::Scalar::all(125), -1, 8, 0);
     cv::circle(can, center, radius, cv::Scalar::all(125), 1, 8, 0);

    }

   cv::namedWindow("edges");
   cv::imshow("edges", can);

   cv::namedWindow("foo");
   cv::imshow("foo", col);

   cv::namedWindow("foreground");
   cv::imshow("foreground", bright);
   cv::waitKey(10);

   return;
  */
 vector<vector<cv::Point> > contours;
 vector<cv::Vec4i> hierarchy;

 cv::Mat cpy;
 bright.copyTo(cpy);



 cv::findContours(cpy, contours, hierarchy, CV_RETR_TREE,
   CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

 cv::namedWindow("foreground");
 cv::imshow("foreground", bright);
 cv::waitKey(100);




 if (contours.size() == 0)
  {
  ss << "Found no contours, adapt threshold or adjust mask";
  return;
  }


 int outer_contours = 0;
 for (uint i=0; i<contours.size(); ++i){
  if (hierarchy[i][3] == 0) outer_contours++;
 }



 // compute area for each contour:
 if (outer_contours > 0)
  {
  ss << "Found " << outer_contours
    << " contours, adapt threshold or adjust mask";
  //    return;
  }


 if (contours[0].size() < 20){
  ss << "Found no large contour, adapt threshold or adjust mask (size of largest contour:" << contours[0].size() << ")";
  return;
 }

 //  float area = cv::contourArea(contours[0]);

 cv::drawContours(bright, contours, 0, cv::Scalar::all(125), 2, 8, hierarchy,
   0, cv::Point());

 input_image.copyTo(cpy);

 cv::Moments mom = cv::moments(contours[0]);
 cv::Point2f mom_center = cv::Point2f(mom.m10 / mom.m00, mom.m01 / mom.m00);

 cv::circle(cpy, mom_center, 10, CV_RGB(0,255,0), 2);

 cv::line(cpy, cv::Point(mom_center.x,0),cv::Point(mom_center.x,cpy.cols),CV_RGB(255,0,0),1);
 cv::line(cpy, cv::Point(0,mom_center.y),cv::Point(cpy.rows, mom_center.y),CV_RGB(255,0,0),1);


 // get mean of all points:
 pcl_Point mean;

 mean.x = mean.y = mean.z = 0;
 int valid = 0;
 //for (uint i = 0; i < contours[0].size(); ++i)
 int l = 1;
 for (int x = mom_center.x-l;x <= mom_center.x+l ; ++x )
  for (int y = mom_center.y-l;y <= mom_center.y+l ; ++y )
   {
   if (x < 0 || x > int(cloud_moved.width) || y < 0 || y > int(cloud_moved.height) ) continue;

   pcl_Point p = cloud_moved.at(x, y);
   if (p.x != p.x)
    continue;

   valid++;
   add(mean, p);

   }

 if (valid == 0){
  ss << "no depth value on marker pose";
  return;
 }

 div(mean, valid);

 ss << "Mean Point: " << mean.x << " " << mean.y << " " << mean.z;

 projector_image.setTo(0);
 cv::Point2f px = applyPerspectiveTrafo(mean, proj_Matrix);

 cv::circle(projector_image, px, 10, CV_RGB(255,0,0), -1);
 cv::circle(projector_image, px, 5, CV_RGB(0,0,255), -1);
 cv::circle(projector_image, px, 1, CV_RGB(0,255,0), -1);
 projector_image.at<cv::Vec3b>(px.y,px.x) = cv::Vec3b(255,0,0);

 corners.push_back(mean);
 // Cloud::Ptr msg = c.makeShared();
 // msg->header.frame_id = "/openni_rgb_optical_frame";
 // msg->header.stamp = ros::Time::now();
 // pub_eval_marker.publish(msg);

 cv::namedWindow("bar");
 cv::imshow("bar", cpy);
 cv::waitKey(10);


 return;


}




bool Projector_Calibrator::saveObservations(){

 ofstream off("calib/obs.txt");

 off << number_of_features_in_images.size() << endl;
 for (uint i=0; i<number_of_features_in_images.size(); ++i)
  off << number_of_features_in_images[i] << endl;

 for (uint i=0; i<observations_3d.size(); ++i){
  pcl_Point p = observations_3d[i];
  off << p.x << " " << p.y << " " << p.z << endl;
 }

 for (uint i=0; i<corners_2d.size(); ++i){
  off << corners_2d[i].x << " " << corners_2d[i].y << endl;
 }

 return true;

}

bool Projector_Calibrator::loadObservations(){

 number_of_features_in_images.clear();
 observations_3d.clear();
 corners_2d.clear();

 ifstream iff("calib/obs.txt");
 int img_cnt; iff >> img_cnt;
 int total = 0;
 int cnt;
 for (int i=0; i<img_cnt; ++i){
  iff >> cnt;
  number_of_features_in_images.push_back(cnt);
  total += cnt;
 }

 pcl_Point p;
 for (int i=0; i<total; ++i){
  iff >> p.x >> p.y >> p.z;
  observations_3d.push_back(p);
 }

 cv::Point2f px;
 for (int i=0; i<total; ++i){
  iff >> px.x >> px.y;
  corners_2d.push_back(px);
 }

 ROS_INFO("Loaded %i pairs in %i images", total, img_cnt);


 return img_cnt>0;
}




Projector_Calibrator::Projector_Calibrator(){
 kinect_orientation_valid = false;
 kinect_trafo_valid = false;

 // checkboard_size = cv::Size(10,6);
 // proj_size = cv::Size(1024,768);

 // char foo[100];
 // std::string cwd = getcwd(foo,100);
 //
 // cout << "cwd " << cwd << endl;

 ros::param::param<bool>("projector_calibration/load_everything_on_startup", load_everything_on_startup, true);

 // reading the number of corners from file
 // int check_width, check_height;
 ros::param::param<int>("projector_calibration/checkerboard_internal_corners_x", max_checkerboard_width, 8);
 ros::param::param<int>("projector_calibration/checkerboard_internal_corners_y", max_checkerboard_height, 6 );
 checkboard_size = cv::Size(max_checkerboard_width, max_checkerboard_height);

 // reading the projector's size from file
 int proj_width, proj_height;
 ros::param::param<int>("projector_calibration/projector_px_width", proj_width, 1024);
 ros::param::param<int>("projector_calibration/projector_px_height", proj_height, 768 );
 proj_size = cv::Size(proj_width, proj_height);

 double marker_size;
 ros::param::param<double>("projector_calibration/printed_marker_corners_dist_mm", marker_size, 25);
 printed_marker_size_mm = marker_size;

 setKinectOrientation(0);


 projector_image = cv::Mat(proj_size, CV_8UC3);

 hom_cv_filename = "homography_opencv";
 hom_svd_filename = "homography_svd.yml";
 proj_matrix_filename = "projection_matrix";
 kinect_trafo_filename = "kinect_trafo";

 test_img = cv::imread("/work/home/engelhan/ros/Touchscreen/imgs/Testbild.png");

 if (!test_img.data){
  ROS_ERROR("Could not open test image at /work/home/engelhan/ros/Touchscreen/imgs/Testbild.png!");
 }

 // creating fullscreen image (old syntax)
 // cvNamedWindow("fullscreen_ipl",0);
 // cvMoveWindow("fullscreen_ipl", 2000, 100);
 // cvSetWindowProperty("fullscreen_ipl", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);

 // showFullscreenCheckerboard();


}


void Projector_Calibrator::initFromFile(std::stringstream& msg){

 // mask = cv::imread("data/kinect_mask.png",0);
 // if (mask.data){
 //  ROS_INFO("Found mask (%i %i)", mask.cols, mask.rows);
 // }else {
 //  ROS_INFO("Could not find mask at data/kinect_mask.png");
 // }


 // Check for Kinect trafo:
 char fn[100]; sprintf(fn, "data/%s.txt",kinect_trafo_filename.c_str());
 kinect_trafo_valid = loadAffineTrafo(kinect_trafo,fn);

 if (kinect_trafo_valid)
  msg << "Loaded transformation from kinect to world frame";
 else
  msg << "Could not load transformation from kinect to world frame from " <<  fn;


 msg << endl;

 // load Matrices
 if (loadMat("data/", proj_matrix_filename, proj_Matrix))
  msg << "Projection matrix was loaded" << endl;

 if (loadMat("data/", hom_cv_filename, hom_CV))
  msg << "Homography was loaded" << endl;

 // loadMat("Homography (SVD)", hom_svd_filename, hom_SVD);
}

bool Projector_Calibrator::loadKinectTrafo(std::stringstream& msg){


 char fn[100]; sprintf(fn, "data/%s.txt",kinect_trafo_filename.c_str());
 kinect_trafo_valid = loadAffineTrafo(kinect_trafo,fn);

 if (kinect_trafo_valid)
  msg << "Loaded transformation from kinect to world frame";
 else
  msg << "Could not load transformation from kinect to world frame from " <<  fn;

 return kinect_trafo_valid;
}

void Projector_Calibrator::drawCheckerboard(cv::Mat& img,cv::Point l1, const cv::Point l2, const cv::Size size, vector<cv::Point2f>& corners_2d){

 corners_2d.clear();
 detected_corners.clear();

 float min_x = min(l1.x,l2.x);
 float min_y = min(l1.y,l2.y);

 float max_x = max(l1.x,l2.x);
 float max_y = max(l1.y,l2.y);


 // draw white border with this size
 // "Note: the function requires some white space (like a square-thick border,
 // the wider the better) around the board to make the detection more robust in various environment"
 float border = 40;

 img.setTo(255);


 // cv::rectangle(img, cv::Point(min_x-20,min_y-20), cv::Point(max_x+20,max_y+20), cv::Scalar::all(255),-1);


 if (min_x > border && min_y > border && max_x < img.cols - border && max_y < img.rows-border)
  border = 0;


 float width = ((max_x-min_x)-2*border)/(size.width+1);
 float height = ((max_y-min_y)-2*border)/(size.height+1);

 float minx = border+min_x;
 float miny = border+min_y;


 // ROS_INFO("GRID: W: %f, H: %f", width, height);

 // start with black square
 for (int j = 0; j<=size.height; j++)
  for (int i = (j%2); i<size.width+1; i+=2){

   cv::Point2f lu = cv::Point2f(minx+i*width,miny+j*height);
   cv::Point2f rl = cv::Point2f(minx+(i+1)*width,miny+(j+1)*height);
   cv::rectangle(img, lu, rl ,cv::Scalar::all(0), -1);

   cv::Point2f ru = cv::Point2f(rl.x,lu.y);

   if (j==0) continue;
   if (i>0){
    corners_2d.push_back(cv::Point2f(lu.x, lu.y));
    //        cvCircle(img, cvPoint(lu.x, lu.y),20, CV_RGB(255,0,0),3);
   }
   if (i<size.width){
    corners_2d.push_back(cv::Point2f(ru.x, ru.y));
    //        cvCircle(img, cvPoint(ru.x, ru.y),20, CV_RGB(255,0,0),3);
   }
  }

 assert(int(corners_2d.size()) == size.width*size.height);


// ROS_INFO("setting corners_2d");

 // improve position of corners: (improvement of reprojection error of about 0.2 px)
 cv::Mat gray;
 cv::cvtColor(img, gray, CV_BGR2GRAY);
 cv::cornerSubPix(gray, corners_2d, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 50, 0.01));
}



/*

void Projector_Calibrator::drawCheckerboard(cv::Mat& img, const cv::Size size, vector<cv::Point2f>& corners_2d){

 corners_2d.clear();

 // draw white border with this size
 // "Note: the function requires some white space (like a square-thick border,
 // the wider the better) around the board to make the detection more robust in various environment"
 float border = 40;

 float width = (img.cols-2*border)/(size.width+1);
 float height = (img.rows-2*border)/(size.height+1);

 img.setTo(255); // all white

 float minx = border;
 float miny = border;

 // ROS_INFO("GRID: W: %f, H: %f", width, height);

 // start with black square
 for (int j = 0; j<=size.height; j++)
  for (int i = (j%2); i<size.width+1; i+=2){

   cv::Point2f lu = cv::Point2f(minx+i*width,miny+j*height);
   cv::Point2f rl = cv::Point2f(minx+(i+1)*width,miny+(j+1)*height);
   cv::rectangle(img, lu, rl ,cv::Scalar::all(0), -1);

   cv::Point2f ru = cv::Point2f(rl.x,lu.y);

   if (j==0) continue;
   if (i>0){
    corners_2d.push_back(cv::Point2f(lu.x, lu.y));
    //        cvCircle(img, cvPoint(lu.x, lu.y),20, CV_RGB(255,0,0),3);
   }
   if (i<size.width){
    corners_2d.push_back(cv::Point2f(ru.x, ru.y));
    //        cvCircle(img, cvPoint(ru.x, ru.y),20, CV_RGB(255,0,0),3);
   }
  }

 assert(int(corners_2d.size()) == size.width*size.height);

 // improve position of corners: (improvement of reprojection error of about 0.2 px)
 cv::Mat gray;
 cv::cvtColor(img, gray, CV_BGR2GRAY);
 cv::cornerSubPix(gray, corners_2d, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 50, 0.01));
}

 */


// set wall_region to same ratio as image
bool Projector_Calibrator::setupImageProjection(float width_m, float off_x_m, float off_y_m, const cv::Size& img_size){
 return setupImageProjection(width_m, width_m/img_size.width*img_size.height, off_x_m, off_y_m, img_size);
}




bool Projector_Calibrator::setupImageProjection(const cv_RectF& wall_area, const cv::Size& img_size){

 if (wall_area.width == 0){
  ROS_ERROR("setupImageProjection: wall area has width of 0!");
  return false;
 }
 return setupImageProjection(wall_area.width, wall_area.height, wall_area.x, wall_area.y, img_size);
}

bool Projector_Calibrator::setupImageProjection(float width_m, float height_m, float off_x_m, float off_y_m, const cv::Size& img_size){


 if(!projMatorHomSet()){
  ROS_WARN("setupImageProjection: Neither Projection Matrix nor Homography computed!");
  return false;
 }

 int width_px  = img_size.width;
 int height_px = img_size.height;

 float height_m_2 = width_m/width_px*height_px;

 if (fabs(height_m_2-height_m) > 0.1){
  ROS_WARN("setupImageProjection: Image and wall-section have different ratios!");
 }

 if (homOpenCVSet() || homSVDSet()){

  // Compute from Homography:
  cv::Mat px_to_world(cv::Size(3,3), CV_64FC1);
  px_to_world.setTo(0);

  // ROS_INFO("Computing warp from Homography!");

  px_to_world.at<double>(2,2) = 1;
  px_to_world.at<double>(0,0) = width_m/width_px;
  px_to_world.at<double>(0,2) = off_x_m;
  px_to_world.at<double>(1,1) = height_m/height_px; // == width/width_px
  px_to_world.at<double>(1,2) = off_y_m;

  if (homOpenCVSet())
   warp_matrix = hom_CV*px_to_world;
  else
   warp_matrix = hom_SVD*px_to_world;

  warp_matrix /= warp_matrix.at<double>(2,2); // defined up to scale

  // cout << "Warp_matrix" << endl << warp_matrix << endl;

  return true;

 }

 cv::Mat px_to_world(cv::Size(3,4), CV_64FC1);
 px_to_world.setTo(0);

 //  ROS_INFO("Computing warp from Projection Matrix!");

 px_to_world.at<double>(3,2) = 1;
 px_to_world.at<double>(0,0) = width_m/width_px;
 px_to_world.at<double>(0,2) = off_x_m;
 px_to_world.at<double>(1,1) = height_m/height_px; // == width/width_px
 px_to_world.at<double>(1,2) = off_y_m;

 warp_matrix = proj_Matrix*px_to_world;
 warp_matrix /= warp_matrix.at<double>(2,2); // defined up to scale

 //  cout << "Warp_matrix: " << endl << warp_matrix << endl;

 return true;



}

void Projector_Calibrator::showUnWarpedImage(const cv::Mat& img){

 if (!warpMatrixSet()){
  ROS_INFO("showUnWarpedImage: call setupImageProjection first.."); return;
 }

 // clear projector image
 projector_image.setTo(0);

 cv::Size size(projector_image.cols, projector_image.rows);

 cv::warpPerspective(img, projector_image, warp_matrix, size, cv::INTER_LINEAR, cv::BORDER_CONSTANT);


 int l = unused_pixels_rows;
 cv::rectangle(projector_image, cv::Point(1,l),cv::Point(projector_image.cols-2,projector_image.rows-l), CV_RGB(255,255,0), 1);


 // IplImage img_ipl = projector_image;
 // cvShowImage("fullscreen_ipl", &img_ipl);

}


bool Projector_Calibrator::computeHomography_OPENCV(float& mean_error){
#define DO_SCALING
 assert(observations_3d.size() == corners_2d.size());
 ROS_WARN("COMPUTING HOMOGRAPHY WITH openCV");

 if (number_of_features_in_images.size() > 1){
  ROS_INFO("More than one image in list, but computing Homography only from first image!");
 }

 if (number_of_features_in_images.size() == 0){
  ROS_INFO("No observations to compute homography from!");
  return false;
 }


 uint N = number_of_features_in_images[0];

 // count number of 3d points which are more than 2cm away from the z=0-plane
 float z_max = 0.03; int cnt = 0;
 for (uint i=0; i<N; ++i) { if (abs(observations_3d.at(i).z) > z_max) cnt++; }
 if (cnt>N*0.1) {  ROS_WARN("found %i of %i points with dist > 3cm", cnt,N); }


 // copy the 3d observations (x,y,~0) into 2d (x,y) (z==0 by construction of coordinate system)
 vector<cv::Point2f> src; src.reserve(N);
 for (uint i=0; i<N; ++i){src.push_back(cv::Point2f(observations_3d.at(i).x,observations_3d.at(i).y));}

 // 2d = H*3d H*(x,y,1)


 vector<cv::Point2f> first_proj_corners;
 first_proj_corners.insert(first_proj_corners.begin(),corners_2d.begin(), corners_2d.begin()+N);

#ifdef DO_SCALING
 cv::Mat T,U;
 vector<cv::Point2f> src_trafoed, d2_trafoed;
 scalePixels(src,  T, src_trafoed);
 scalePixels(first_proj_corners,  U, d2_trafoed);
 hom_CV = cv::findHomography(src_trafoed,d2_trafoed);
 hom_CV = U.inv()*hom_CV*T;
#else
 hom_CV = cv::findHomography(src,first_proj_corners);
#endif


 // cout << "Homography with OpenCV: " << endl << hom_CV << endl;

 // compute error:
 mean_error = 0;

 cv::Point2f px;
 float err_x = 0;
 float err_y = 0;

 float e_x, e_y;

 for (uint i=0; i<N; ++i){
  applyHomography(cv::Point2f(observations_3d.at(i).x,observations_3d.at(i).y),hom_CV, px);

  e_x = abs((px.x-current_projector_corners.at(i).x));
  e_y = abs((px.y-current_projector_corners.at(i).y));

  err_x += e_x/N; err_y += e_y/N ;
  mean_error += sqrt(e_x*e_x+e_y*e_y)/N;

  //  ROS_INFO("Proj: %f %f, goal: %f %f (Error: %f)", px.x,px.y,corners_2d.at(i).x, corners_2d.at(i).y,err);
 }


 ROS_INFO("mean error: %f (x: %f, y: %f)", mean_error, err_x, err_y);

 saveMat("data/", hom_cv_filename, hom_CV);

 return true;
}


bool Projector_Calibrator::computeHomography_SVD(){
#define SCALE_SVD

 ROS_WARN("COMPUTING HOMOGRAPHY WITH SVD");

 assert(observations_3d.size() == corners_2d.size());

 if (number_of_features_in_images.size() == 0){
  ROS_INFO("No image with observations!");
  return false;
 }


 if (number_of_features_in_images.size() > 1){
  ROS_INFO("More than one image in list, but computing Homography only from first image!");
 }


 // uint N = current_projector_corners.size();
 // if(observations_3d.size() < N){
 //  ROS_ERROR("computeHomography_SVD: less 3d-points than 2d-points, aborting");
 //  return false;
 // }

 int N = number_of_features_in_images[0];


 // count number of 3d points which are more than 2cm away from the z=0-plane
 float z_max = 0.03; int cnt = 0;
 for (int i=0; i<N; ++i) { if (abs(observations_3d.at(i).z) > z_max) cnt++; }
 if (cnt>N*0.1) { ROS_WARN("found %i of %i points with dist > 3cm", cnt,N); }


 vector<cv::Point2f> first_proj_corners;
 first_proj_corners.insert(first_proj_corners.begin(),corners_2d.begin(), corners_2d.begin()+N);

#ifdef SCALE_SVD
 cv::Mat T;
 vector<cv::Point2f> d2_trafoed;
 scalePixels(first_proj_corners,  T, d2_trafoed);
#else
 vector<cv::Point2f> d2_trafoed = first_proj_corners;
#endif



 // Pointcloud to 2d-points (z ==0, by construction of the coordinate system)
 vector<cv::Point2f> src, src_trafoed;
 for (int i=0; i<N; ++i) src.push_back(cv::Point2f(observations_3d.at(i).x,observations_3d.at(i).y));

#ifdef SCALE_SVD
 cv::Mat U;
 scalePixels(src, U,src_trafoed);
#else
 src_trafoed = src;
#endif


 cv::Mat A = cv::Mat(2*N,9,CV_64FC1);
 A.setTo(0);

 // p_ cross H*p = 0
 for (int i=0; i<N; ++i){
  cv::Point2f P = src_trafoed.at(i);
  cv::Point2f p = d2_trafoed.at(i);

  // ROS_INFO("P: %f %f,  p: %f %f", P.x, P.y, p.x, p.y);

  float f[9] = {0,0,0,-P.x,-P.y,-1,p.y*P.x,p.y*P.y,p.y};
  for (uint j=0; j<9; ++j) A.at<double>(2*i,j) = f[j];

  float g[9] = {P.x,P.y,1,0,0,0,-p.x*P.x,-p.x*P.y,-p.x};
  for (uint j=0; j<9; ++j) A.at<double>(2*i+1,j) = g[j];
 }
 // now solve A*h == 0
 // Solution is the singular vector with smallest singular value

 cv::Mat h = cv::Mat(9,1,CV_64FC1);
 cv::SVD::solveZ(A,h);


 // h only fixed up to scale -> set h(3,3) = 1;
 h /= h.at<double>(8);

 // cout << "h: " << h << endl;

 hom_SVD = cv::Mat(3,3,CV_64FC1);

 for (uint i=0; i<3; ++i){
  for (uint j=0; j<3; ++j)
   hom_SVD.at<double>(i,j) =  h.at<double>(3*i+j);
 }

 // cout << "Tinv " << T.inv() << endl;
 // cout << "U " << U << endl;

 // undo scaling
#ifdef SCALE_SVD
 hom_SVD = T.inv()*hom_SVD*U;
#endif

 // 2d = H*3d H*(x,y,1)


 //  cout << "Homography with SVD: " << endl << hom_SVD << endl;


 // compute error:
 float error = 0;
 float err_x = 0;
 float err_y = 0;

 float a,b;




 cv::Point2f px;
 for (int i=0; i<N; ++i){
  applyHomography(src.at(i), hom_SVD, px);

  a = abs(px.x-current_projector_corners.at(i).x);
  b = abs(px.y-current_projector_corners.at(i).y);

  err_x += a/N; err_y += b/N;

  error += sqrt(a*a+b*b)/N;

  //ROS_INFO("src: %f %f, Proj: %f %f, goal: %f %f (Error: %f)", src.at(i).x, src.at(i).y, px.x,px.y,corners_2d.at(i).x, corners_2d.at(i).y,err);
 }

 ROS_INFO("mean error: %f (x: %f, y: %f)", error, err_x, err_y);

 saveMat("data/", hom_svd_filename, hom_SVD);

 return true;
}






bool Projector_Calibrator::computeProjectionMatrix_OPENCV(float& mean_error){

 ROS_WARN("COMPUTING Projection Matrix with openCV");
 assert(observations_3d.size() == corners_2d.size());


 if (number_of_features_in_images.size() == 0){
  ROS_WARN("Can't compute projection matrix without observations");
  return false;
 }

 if (number_of_features_in_images.size() == 1){
  // TODO: check range of z-values to see if points are distributed
  ROS_WARN("You should not compute projection matrix from one image, 3d points must not lie in one plane");
  //return false;
 }


 vector<vector<cv::Point3f> > world_points;
 vector<vector<cv::Point2f> > pixels;

 vector<cv::Mat> rvecs, tvecs;

 vector<cv::Point3f> pt_3;
 vector<cv::Point2f> pt_2;

 cv::Point3f d3;
 cv::Point2f d2;

 int pos = 0; // position within current image
 int img = 0; // number of image in sequence
 for (uint i=0; i<observations_3d.size(); ++i){
  pcl_Point p = observations_3d[i];
  d3.x = p.x;   d3.y = p.y;  d3.z = p.z;
  pt_3.push_back(d3);

  d2.x = corners_2d[i].x; d2.y = corners_2d[i].y;
  pt_2.push_back(d2);

  if (pos == number_of_features_in_images[img]-1){
   world_points.push_back(pt_3); pt_3.clear();
   pixels.push_back(pt_2); pt_2.clear();
   pos=0;
   img++;
  }else{
   pos++;
  }
 }

 // small check:
 for (uint i=0; i<number_of_features_in_images.size(); ++i){
  assert(int(world_points[i].size()) == number_of_features_in_images[i]);
  assert(int(pixels[i].size()) == number_of_features_in_images[i]);
 }

 cv::Mat cameraMatrix = cv::Mat(3,3,CV_32FC1);
 cv::Mat distCoeffs = cv::Mat(1,5,CV_32FC1);
 distCoeffs.setTo(0);


 cameraMatrix.setTo(0);
 cameraMatrix.at<float>(0,2) = proj_size.width/2;
 cameraMatrix.at<float>(1,2) = proj_size.height/2;
 cameraMatrix.at<float>(0,0) = 1000;
 cameraMatrix.at<float>(1,1) = 1000;
 cameraMatrix.at<float>(2,2) = 1;

 cout << "camera (init): " << cameraMatrix << endl;



 double error = cv::calibrateCamera(world_points, pixels, proj_size, cameraMatrix, distCoeffs, rvecs, tvecs,CV_CALIB_USE_INTRINSIC_GUESS || CV_CALIB_ZERO_TANGENT_DIST || CV_CALIB_FIX_K1 || CV_CALIB_FIX_K2 || CV_CALIB_FIX_K3 );

 cout << "error " << error << endl;
 cout << "camera: " << cameraMatrix << endl;
 cout << "distortion: " << distCoeffs << endl;

 //int N = observations_3d.size();

 // only eval first matrix on its points
 int N = number_of_features_in_images[0];

 for (uint i=0; i<tvecs.size() && i<1; ++i){

  mean_error = 0;
  float total_x = 0;
  float total_y = 0;


  cout << "trafo " << i << endl;
  cout << "tvec" << tvecs[i] << endl;


  //  undistortPoints

  // build projection matrix:
  cv::Mat rot_matrix;
  cv::Rodrigues(rvecs[i], rot_matrix);
  cv::Mat P;
  cv::hconcat(rot_matrix, tvecs[i], P);

  P = cameraMatrix * P;

  P /= P.at<double>(2,3);

  cout << "P: " << endl << P << endl;

  cv::Point2f px;

  for (int i=0; i<N; ++i){
   //    ROS_INFO("projection %i", i);

   pcl_Point   p = observations_3d.points.at(i);
   cv::Point2f p_ = corners_2d.at(i);

   applyPerspectiveTrafo(cv::Point3f(p.x,p.y,p.z),P,px);


   // ROS_INFO("err: %f %f", (px.x-p_.x),(px.y-p_.y));

   total_x += abs(px.x-p_.x)/N;
   total_y += abs(px.y-p_.y)/N;
   mean_error += sqrt(pow(px.x-p_.x,2)+pow(px.y-p_.y,2))/N;

   //   float err = sqrt(pow(px.x-p_.x,2)+pow(px.y-p_.y,2));
   //   cout << "err " << err << endl;
   //   ROS_INFO("dx dy: %f %f", abs(px.x-p_.x),abs(px.y-p_.y) );

  }

  ROS_INFO("Projection Matrix: mean error: %f (x: %f, y: %f)", mean_error, total_x, total_y);





 }




 return true;
}



/*
bool Projector_Calibrator::computeProjectionMatrix_OPENCV(float& mean_error){

 ROS_WARN("COMPUTING Projection Matrix with openCV");
 assert(observations_3d.size() == corners_2d.size());


 if (number_of_features_in_images.size() == 0){
  ROS_WARN("Can't compute projection matrix without observations");
  return false;
 }

 if (number_of_features_in_images.size() == 1){
  // TODO: check range of z-values to see if points are distributed
  ROS_WARN("You should not compute projection matrix from one image, 3d points must not lie in one plane");
  //return false;
 }


 vector<vector<cv::Point3f> > world_points;
 vector<vector<cv::Point2f> > pixels;

 vector<cv::Mat> rvecs, tvecs;

 vector<cv::Point3f> pt_3;
 vector<cv::Point2f> pt_2;

 cv::Point3f d3;
 cv::Point2f d2;

 int pos = 0; // position within current image
 int img = 0; // number of image in sequence
 for (uint i=0; i<observations_3d.size(); ++i){
  pcl_Point p = observations_3d[i];
  d3.x = p.x;   d3.y = p.y;  d3.z = p.z;
  pt_3.push_back(d3);

  d2.x = corners_2d[i].x; d2.y = corners_2d[i].y;
  pt_2.push_back(d2);

  if (pos == number_of_features_in_images[img]-1){
   world_points.push_back(pt_3); pt_3.clear();
   pixels.push_back(pt_2); pt_2.clear();
   pos=0;
   img++;
  }else{
   pos++;
  }
 }

 // small check:
 for (uint i=0; i<number_of_features_in_images.size(); ++i){
  assert(int(world_points[i].size()) == number_of_features_in_images[i]);
  assert(int(pixels[i].size()) == number_of_features_in_images[i]);
 }

 cv::Mat cameraMatrix = cv::Mat(3,3,CV_32FC1);
 cv::Mat distCoeffs = cv::Mat(1,5,CV_32FC1);
 distCoeffs.setTo(0);


 cameraMatrix.setTo(0);
 cameraMatrix.at<float>(0,2) = proj_size.width/2;
 cameraMatrix.at<float>(1,2) = proj_size.height/2;
 cameraMatrix.at<float>(0,0) = 1000;
 cameraMatrix.at<float>(1,1) = 1000;
 cameraMatrix.at<float>(2,2) = 1;

 cout << "camera (init): " << cameraMatrix << endl;



 double error = cv::calibrateCamera(world_points, pixels, proj_size, cameraMatrix, distCoeffs, rvecs, tvecs,CV_CALIB_USE_INTRINSIC_GUESS || CV_CALIB_ZERO_TANGENT_DIST || CV_CALIB_FIX_K1 || CV_CALIB_FIX_K2 || CV_CALIB_FIX_K3 );

 cout << "error " << error << endl;
 cout << "camera: " << cameraMatrix << endl;
 cout << "distortion: " << distCoeffs << endl;

 //int N = observations_3d.size();

 // only eval first matrix on its points
 int N = number_of_features_in_images[0];

 for (uint i=0; i<tvecs.size() && i<1; ++i){

  mean_error = 0;
  float total_x = 0;
  float total_y = 0;


  cout << "trafo " << i << endl;
  cout << "tvec" << tvecs[i] << endl;


  //  undistortPoints

  // build projection matrix:
  cv::Mat rot_matrix;
  cv::Rodrigues(rvecs[i], rot_matrix);
  cv::Mat P;
  cv::hconcat(rot_matrix, tvecs[i], P);

  P = cameraMatrix * P;

  P /= P.at<double>(2,3);

  cout << "P: " << endl << P << endl;

  cv::Point2f px;

  for (int i=0; i<N; ++i){
   //    ROS_INFO("projection %i", i);

   pcl_Point   p = observations_3d.points.at(i);
   cv::Point2f p_ = corners_2d.at(i);

   applyPerspectiveTrafo(cv::Point3f(p.x,p.y,p.z),P,px);


   // ROS_INFO("err: %f %f", (px.x-p_.x),(px.y-p_.y));

   total_x += abs(px.x-p_.x)/N;
   total_y += abs(px.y-p_.y)/N;
   mean_error += sqrt(pow(px.x-p_.x,2)+pow(px.y-p_.y,2))/N;

   //   float err = sqrt(pow(px.x-p_.x,2)+pow(px.y-p_.y,2));
   //   cout << "err " << err << endl;
   //   ROS_INFO("dx dy: %f %f", abs(px.x-p_.x),abs(px.y-p_.y) );

  }

  ROS_INFO("Projection Matrix: mean error: %f (x: %f, y: %f)", mean_error, total_x, total_y);





 }




 return true;
}
 */

bool Projector_Calibrator::computeProjectionMatrix(float& mean_error){

 ROS_WARN("COMPUTING Projection Matrix");
 assert(observations_3d.size() == corners_2d.size());


 if (number_of_features_in_images.size() == 0){
  ROS_WARN("Can't compute projection matrix without observations");
  return false;
 }

 // get range in z:
 float z_min = 1e5; float z_max = -1e5;
 for (uint i=0; i<observations_3d.size(); ++i){
  pcl_Point p = observations_3d[i]; if (p.x!=p.x) continue;
  z_min = min(z_min,p.z); z_max = max(z_max,p.z);
 }

 if (abs(z_max-z_min) < 0.1){
  ROS_WARN("computeProjectionMatrix: Observations have to little variation in z! (%f to %f)", z_min, z_max);
  return false;
 }


 uint N = observations_3d.size();
 ROS_INFO("Computing Projection matrix from %zu images and %u pairs", number_of_features_in_images.size(),N);


 Cloud trafoed_corners;
 vector<cv::Point2f> trafoed_px;
 cv::Mat U,T;

 scaleCloud(observations_3d, U, trafoed_corners);
 scalePixels(corners_2d, T, trafoed_px);


 cv::Mat A = cv::Mat(2*N,12,CV_64FC1);
 A.setTo(0);

 //ROS_ERROR("Projection:");

 // p_ cross H*p = 0
 for (uint i=0; i<N; ++i){
  pcl_Point   P = trafoed_corners.points.at(i);
  cv::Point2f p = trafoed_px.at(i);

  //  ROS_INFO("from %f %f %f to %f %f", P.x,P.y,P.z,p.x,p.y);

  float f[12] = {0,0,0,0,-P.x,-P.y,-P.z,-1,p.y*P.x,p.y*P.y,p.y*P.z,p.y};
  for (uint j=0; j<12; ++j) A.at<double>(2*i,j) = f[j];

  float g[12] = {P.x,P.y,P.z,1,0,0,0,0,-p.x*P.x,-p.x*P.y,-p.x*P.z,-p.x};
  for (uint j=0; j<12; ++j) A.at<double>(2*i+1,j) = g[j];
 }

 // now solve A*h == 0
 // Solution is the singular vector with smallest singular value

 cv::Mat h = cv::Mat(12,1,CV_64FC1);
 cv::SVD::solveZ(A,h);

 proj_Matrix = cv::Mat(3,4,CV_64FC1);

 for (uint i=0; i<3; ++i){
  for (uint j=0; j<4; ++j)
   proj_Matrix.at<double>(i,j) =  h.at<double>(4*i+j);
 }

 // cout << "Tinv " << T.inv() << endl;
 // cout << "U " << U << endl;

 // undo scaling
 proj_Matrix = T.inv()*proj_Matrix*U;


 proj_Matrix/=proj_Matrix.at<double>(2,3);

 // cout << "Projection Matrix: " << endl << proj_Matrix << endl;


 // computation of reprojection error and check if errors are zero-centered


 mean_error = 0;
 double total_x = 0;
 double total_y = 0;

 cv::Point2f px;

 Normal_dist<float> eval_x, eval_y;

 for (uint i=0; i<N; ++i){

  pcl_Point   p = observations_3d.points.at(i);
  cv::Point2f p_ = corners_2d.at(i);

  applyPerspectiveTrafo(cv::Point3f(p.x,p.y,p.z),proj_Matrix,px);

  // ROS_INFO("err: %f %f", (px.x-p_.x),(px.y-p_.y));

  eval_x.add(px.x-p_.x);
  eval_y.add(px.y-p_.y);

  total_x += abs(px.x-p_.x)/N;
  total_y += abs(px.y-p_.y)/N;
  mean_error += sqrt(pow(px.x-p_.x,2)+pow(px.y-p_.y,2))/N;

 }

 eval_x.evaluate();
 eval_y.evaluate();

 if (!eval_x.isProbablyZeroCentered() || abs(eval_x.mean) > 10){
  ROS_WARN("Projection matrix: check values of x-direction!");
  ROS_WARN("Err in x: mu = %f, var: %f", eval_x.mean, eval_x.variance);
 }


 if (!eval_y.isProbablyZeroCentered()|| abs(eval_y.mean) > 10){
  ROS_WARN("Projection matrix: check values of y-direction!");
  ROS_WARN("Err in y: mu = %f, var: %f", eval_y.mean, eval_y.variance);
 }

 ROS_INFO("Projection Matrix: mean error: %f (x: %f, y: %f)", mean_error, total_x, total_y);
 // saveMat("Projection Matrix", proj_matrix_filename, proj_Matrix);

 saveMat("data/", proj_matrix_filename, proj_Matrix);

 cout << "Projection Matrix" << endl << proj_Matrix << endl;

 cv::Mat rotMatrix, camera_matrix;

 cv::decomposeProjectionMatrix(proj_Matrix, camera_matrix,rotMatrix, projector_position);

 //camera_matrix /= camera_matrix.at<double>(2,2);

 projector_position /= projector_position.at<double>(3);

 cout << "camera_matrix" << endl << camera_matrix << endl;
 cout << "Proj pose: " << endl << projector_position << endl;
 cout << "rotMatrix: " << endl << rotMatrix << endl;

 /*
 cv::Mat P_(proj_Matrix.colRange(0,3));
 P_ = proj_Matrix.colRange(cv::Range(0,3));
// proj_Matrix.copyTo(P_);

 cout << "sub matrix: " << P_ << endl;

 cv::Mat F_(proj_Matrix.colRange(3,4));
 F_ = proj_Matrix.colRange(cv::Range(3,4));
 cout << "F_ matrix: " << F_ << endl;

cv::Mat new_pose = P_.inv()*F_;

cout << "new_pose " << new_pose << endl;
  */



 // do some testing:
 // cv::Point3f out;
 // project3D(cv::Point2f(0,0), proj_Matrix,1, out);
 // project3D(cv::Point2f(100,100), proj_Matrix,1, out);
 // project3D(cv::Point2f(400,123), proj_Matrix,1, out);




 return true;

}


bool Projector_Calibrator::findCheckerboardCorners(){
 // #define SHOW_DETECTIONS
 // detected_corners.clear();
 if (input_image.rows == 0){  ROS_WARN("can't find corners on empty image!"); return false;  }

 ROS_INFO("Looking for checkerboard with %i %i corners img: %i %i", checkboard_size.width, checkboard_size.height, input_image.cols, input_image.rows);

 if (!cv::findChessboardCorners(input_image, checkboard_size,detected_corners, CV_CALIB_CB_ADAPTIVE_THRESH)) {
  ROS_WARN("Could not find a checkerboard!");
  return false;
 }

 cv::cvtColor(input_image, gray, CV_BGR2GRAY);
 cv::cornerSubPix(gray, detected_corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 50, 0.01));

#ifdef SHOW_DETECTIONS
 cv::Mat cpy = input_image.clone();
 cv::drawChessboardCorners(cpy, checkboard_size, detected_corners, true);
 cv::namedWindow("search",1); cv::imshow("search", cpy);
 cv::waitKey(-1);
#endif



 return true;
}

bool Projector_Calibrator::saveHomographyCV(std::stringstream& msg){
 if (hom_CV.cols == 0){
  msg << "Homography is not yet computed!";
  return false;
 }

 if (saveMat("data/", hom_cv_filename, hom_CV)){
  msg << "Homography was written to " << hom_cv_filename;
  return true;
 }else{
  msg << "Problems writing Homography to " << hom_cv_filename;
  return false;
 }

}


bool Projector_Calibrator::saveProjectionMatrix(std::stringstream& msg){
 if (proj_Matrix.cols == 0){
  msg << "Projection matrix is not yet computed!";
  return false;
 }


 if (saveMat("data/", proj_matrix_filename, proj_Matrix)){
  msg << "Projection Matrix was written to " << proj_matrix_filename;
  return true;
 }else{
  msg << "Problems writing Projection Matrix to " << proj_matrix_filename;
  return false;
 }
}




bool Projector_Calibrator::storeCurrentObservationPairs(){


 // for each 3d observation, there is a projected pixel (in current image)
 assert(current_projector_corners.size() == detected_corners.size());
 assert(current_projector_corners.size() > 0);

 if (!kinect_trafo_valid){
  ROS_WARN("can't store Observations in global Frame since it is not defined!");
  return false;
 }

 if (detected_corners.size() == 0){
  ROS_WARN("Can't add Observations since Corners havn't been detected");
  return false;
 }

 if (input_cloud.size() == 0){
  ROS_WARN("storeCurrent3DObservations: input_cloud.size() == 0");
  return false;
 }

 Cloud c_3d;
 for (uint i=0; i<detected_corners.size(); ++i){
  pcl_Point p = input_cloud.at(detected_corners[i].x, detected_corners[i].y);
  if (!(p.x == p.x)){
   ROS_WARN("storeCurrent3DObservations: Found Corner without depth!");
   return false;
  }
  c_3d.points.push_back(p);
 }

 // TODO

 // transform from kinect-frame to wall-frame
 pcl::getTransformedPointCloud(c_3d, kinect_trafo, c_3d);

 assert(corners_2d.size() == observations_3d.size());

 std::vector<cv::Point2f> foo;

 foo.insert(foo.begin(),current_projector_corners.begin(), current_projector_corners.end() );

 assert(current_projector_corners.size() > 0);
 ROS_INFO("Inserting: corners_2d.size: %zu, adding %zu points", corners_2d.size(), current_projector_corners.size());
 for (uint i=0; i< foo.size(); ++i){
  cout << "size: " << i << " / " << foo.size() << " corners_2d.size: " << corners_2d.size() << endl;
  corners_2d.push_back(foo[i]);
 }

 // corners_2d.insert(corners_2d.end(), current_projector_corners.begin(), current_projector_corners.end());
 ROS_INFO("Inserting 2");

 for (uint i=0; i<c_3d.points.size(); ++i){
  observations_3d.push_back(c_3d[i]);
 }

 // observations_3d.points.insert(observations_3d.points.end(),c_3d.points.begin(), c_3d.points.end());
 ROS_INFO("Added %zu points, now %zu 3D-Observations",c_3d.size(), observations_3d.size());
 ROS_INFO("Inserting 3");


 assert(current_projector_corners.size() == c_3d.size());
 number_of_features_in_images.push_back(current_projector_corners.size());

 ROS_INFO("Image cnt: %zu", number_of_features_in_images.size());

 return true;
}


bool Projector_Calibrator::removeLastObservations(){

 ROS_INFO("Remove!");

 ROS_INFO("imgs: %zu", number_of_features_in_images.size());


 if (number_of_features_in_images.size() == 0)
  return false;


 uint obs_in_last = *number_of_features_in_images.rbegin();

 ROS_INFO("removing %i features from last img, obs_3d: %zu", obs_in_last,observations_3d.size());

 assert(observations_3d.size() >= obs_in_last);

 uint old_size = observations_3d.size();

 // end() points 1 after the last, s.t. obs.end()-1 is last valid iterator

 observations_3d.erase (observations_3d.end()-obs_in_last-1,observations_3d.end()-1);
 corners_2d.erase(corners_2d.end()-obs_in_last-1,corners_2d.end()-1);

 ROS_INFO("erased, now: 2d: %zu 3d: %zu", corners_2d.size(), observations_3d.size());

 // remove last entry in list
 number_of_features_in_images.erase(number_of_features_in_images.begin()+number_of_features_in_images.size()-1);

 assert(observations_3d.size() + obs_in_last == old_size);
 assert(observations_3d.size() == corners_2d.size());

 ROS_INFO("removal successful");

 return true;

}


bool z_comp(pcl_Point A, pcl_Point B){return A.z > B.z;}

Cloud Projector_Calibrator::visualizePointCloud(){

 Cloud coled;

 projector_image.setTo(0);


 if (!cloud_moved.size()>0){
  ROS_INFO("No cloud!"); return coled;
 }

 // min dist from wall
 float max_dist = 0.25; // needed to scale color

 // sort cloud in z-direction

 // std::vector<pcl_Point> sorted; sorted.reserve(cloud_moved.size());
 // for (uint i=0; i<cloud_moved.size(); ++i) {
 //  pcl_Point p = cloud_moved[i];
 //  if (p.x!=p.x) continue;
 //  sorted.push_back(p);
 // }

 // std::sort(sorted.begin(), sorted.end(), z_comp);


 for (uint x=0; x<cloud_moved.width; ++x)
  for (uint y=0; y<cloud_moved.height; ++y){
   if (mask.at<uchar>(y,x) == 0) continue;

   pcl_Point p = cloud_moved.at(x,y);


   float z = -p.z;// z points into wall

   // cout << z << endl;


   // dont't show groundplane
   if (z<0 || z>max_dist)
    continue;

   cv::Point2f px;
   applyPerspectiveTrafo(cv::Point3f(p.x,p.y,p.z),proj_Matrix,px);

   int x = px.x; int y = px.y;


   if (x<0 || y<0 || x>=projector_image.cols || y >= projector_image.rows) continue;

   //  cv::Vec3b col = projector_image.at<cv::Vec3b>(y,x);
   //
   //  if (col.val[0]>0) continue;

   //  p.b = p.r = p.g = 0;


   //  if (z<min_dist)
   //   p.r = 255;
   //  else
   //   if (z<0.5)
   //    p.g = 255;
   //   else
   //    p.b = 255;

   // px.y -= 5;
   //  cv::circle(projector_image, px, 10, CV_RGB(p.r,p.g,p.b),-1);


   //  projector_image.at<cv::Vec3b>(px.y,px.x) = cv::Vec3b((int((z*3/max_dist)*180))%180,255,255);

   cv::circle(projector_image, px, 3, cv::Scalar((int(((z)/max_dist*1.5)*180))%180,255,255) ,-1);

   //   cv::circle(projector_image, px, 3, CV_RGB(255,0,0),-1);


   //coled.push_back(p);


  }

 cv::cvtColor(projector_image,projector_image, CV_HSV2BGR);


 // IplImage proj_ipl = projector_image;
 // cvShowImage("fullscreen_ipl", &proj_ipl);

 return coled;

}



//void computeTransformationFromYZVectorsAndOrigin(const Eigen::Vector3f& y_direction, const Eigen::Vector3f& z_axis,
//  const Eigen::Vector3f& origin, Eigen::Affine3f& transformation){
//
// Eigen::Vector3f x = (y_direction.cross(z_axis)).normalized();
// Eigen::Vector3f y = y_direction.normalized();
// Eigen::Vector3f z = z_axis.normalized();
//
// Eigen::Affine3f sub = Eigen::Affine3f::Identity();
// sub(0,3) = -origin[0];
// sub(1,3) = -origin[1];
// sub(2,3) = -origin[2];
//
//
// transformation = Eigen::Affine3f::Identity();
// transformation(0,0)=x[0]; transformation(0,1)=x[1]; transformation(0,2)=x[2]; // x^t
// transformation(1,0)=y[0]; transformation(1,1)=y[1]; transformation(1,2)=y[2]; // y^t
// transformation(2,0)=z[0]; transformation(2,1)=z[1]; transformation(2,2)=z[2]; // z^t
//
// transformation = transformation*sub;
//}




bool  Projector_Calibrator::computeKinectTransformation(std::stringstream& msg){

 if (!kinect_orientation_valid){
  ROS_INFO("Can't compute KinectTrafo without Kinect's orientation angle");
  return false;
 }

 if (detected_corners.size() == 0){
  msg << "can't compute Kinect trafo without observed corners";
  // ROS_WARN("can't compute Kinect trafo without observed corners");
  return false;
 }


 Cloud filtered;
 applyMaskOnInputCloud(filtered);

 Eigen::Vector4f plane_model;
 fitPlaneToCloud(filtered, plane_model);

 int m = (checkboard_size.height/2*checkboard_size.width)+(checkboard_size.width-1)/2;

 pcl_Point p  = input_cloud.at(detected_corners[m].x, detected_corners[m].y);
 pcl_Point p2 = input_cloud.at(detected_corners[m].x+sin(-kinect_tilt_angle_deg/180*M_PI)*100, detected_corners[m].y-cos(-kinect_tilt_angle_deg/180*M_PI)*100);

 if ( p2.x != p2.x){
  msg << "NAN in pointcloud, no calculation of new wall-system" << endl;
  ROS_WARN("NAN in pointcloud, no calculation of new wall-system");
  return false;
 }

 Eigen::Vector3f pl_center = Eigen::Vector3f(p.x,p.y,p.z);
 Eigen::Vector3f pl_upwards = Eigen::Vector3f(p2.x-p.x,p2.y-p.y,p2.z-p.z);

 float plane_direction = plane_model.head<3>()[2]>0?1:-1;

 // compute trafo without pcl


 // z-axis towards camera
 computeTransformationFromYZVectorsAndOrigin(pl_upwards,-plane_direction*plane_model.head<3>(), pl_center,kinect_trafo);
 // PCL alternative
 //pcl::getTransformationFromTwoUnitVectorsAndOrigin(-pl_upwards,plane_direction*plane_model.head<3>(), pl_center, kinect_trafo);


 // save to file
 // char fn[100]; sprintf(fn, "data/%s.txt",kinect_trafo_filename.c_str());
 // if (saveAffineTrafo(kinect_trafo,fn))
 //  ROS_INFO("Wrote kinect_trafo to %s", fn);
 //
 // printTrafo(kinect_trafo);

 pcl::getTransformedPointCloud(input_cloud,kinect_trafo,cloud_moved);
 kinect_trafo_valid = true;

 return true;
}



void Projector_Calibrator::getCheckerboardArea(vector<cv::Point2i>& pts){

 pts.clear();
 if (detected_corners.size() == 0){
  ROS_WARN("getCheckerboardArea: no corners!"); return;
 }

 // find corners of checkerboard (OpenCV only returns inner corners..)

 int w = checkboard_size.width;
 int h = checkboard_size.height;


 cv::Point2i p = detected_corners[0]; // first corner in top row
 cv::Point2i q = detected_corners[w+1]; // second corner in second row
 pts.push_back(cv::Point2i(2*p.x-q.x,2*p.y-q.y)); // extrapolation to find left upper corner

 p = detected_corners[w-1];
 q = detected_corners[2*w-2];
 pts.push_back(cv::Point2i(2*p.x-q.x,2*p.y-q.y));


 p = detected_corners[w*h-1];
 q = detected_corners[w*h-1-w-1];
 pts.push_back(cv::Point2i(2*p.x-q.x,2*p.y-q.y));


 p = detected_corners[(h-1)*w];
 q = detected_corners[(h-2)*w+1];
 pts.push_back(cv::Point2i(2*p.x-q.x,2*p.y-q.y));

 assert(pts.size() == 4);
}


bool Projector_Calibrator::getProjectionAreain3D(Cloud& corners){

 vector<cv::Mat> Corners_3d;

 if (!getProjectionAreain3D(Corners_3d)) return false;

 pcl_Point p;

 for (uint i=0;i<Corners_3d.size(); ++i){
  p.x = Corners_3d[i].at<double>(0);
  p.y = Corners_3d[i].at<double>(1);
  p.z = Corners_3d[i].at<double>(2);

  //  ROS_INFO("corner: %f %f %f", p.x,p.y,p.z);
  corners.push_back(p);
 }

 return true;


}

void Projector_Calibrator::updateProjectorImage(){

 // ROS_INFO("Update projector image");
 // IplImage proj_ipl = projector_image;
 // cvShowImage("fullscreen_ipl", &proj_ipl);

}


bool Projector_Calibrator::getProjectionAreain3D(vector<cv::Mat>& corners){


 if (!homOpenCVSet()){ return false;}

 cv::Mat hom_inv = hom_CV.inv();

 cv::Mat p(3,1,CV_64FC1);
 cv::Mat p_3d(3,1,CV_64FC1);


 // projector does not show lowest n pixels and top n pixels are under the OS-bar


 int space_vertical = unused_pixels_rows;
 int space_horizontal = 10;


 // for each projector corner, find the corresponding 3d point on the z=0 plane
 p.at<double>(0) = space_horizontal; p.at<double>(1) = space_vertical; p.at<double>(2) = 1;
 p_3d = hom_inv*p; p_3d/=p_3d.at<double>(2); p_3d.at<double>(2) = 0; corners.push_back(p_3d.clone());

 p.at<double>(0) = proj_size.width-space_horizontal; p.at<double>(1) = space_vertical; p.at<double>(2) = 1;
 p_3d = hom_inv*p; p_3d/=p_3d.at<double>(2); p_3d.at<double>(2) = 0; corners.push_back(p_3d.clone());

 p.at<double>(0) = proj_size.width-space_horizontal; p.at<double>(1) = proj_size.height-space_vertical; p.at<double>(2) = 1;
 p_3d = hom_inv*p; p_3d/=p_3d.at<double>(2); p_3d.at<double>(2) = 0; corners.push_back(p_3d.clone());

 p.at<double>(0) = space_horizontal; p.at<double>(1) = proj_size.height-space_vertical; p.at<double>(2) = 1;
 p_3d = hom_inv*p; p_3d/=p_3d.at<double>(2); p_3d.at<double>(2) = 0; corners.push_back(p_3d.clone());

 return true;

}


bool Projector_Calibrator::findOptimalProjectionArea2(cv::Mat::MSize img_px_size, std::stringstream& msg){

 if (!homOpenCVSet()){
  msg << "Homography is not set!"; return false;
 }

 float ratio = img_px_size[1]*1.0/img_px_size[0];


 // get 3d Poses of the projector corners:
 vector<cv::Mat> Corners_3d;
 getProjectionAreain3D(Corners_3d);


 double min_x = 1e10;
 double min_y = 1e10;

 assert(Corners_3d.size() == 4);

 for (uint i=0; i<Corners_3d.size(); ++i){
  min_x = min(min_x, Corners_3d[i].at<double>(0));
  min_y = min(min_y, Corners_3d[i].at<double>(1));
 }

 vector<cv::Point2i> px_coordinates;
 int max_x, max_y;
 max_x = max_y = -1;

 for (uint i=0; i<Corners_3d.size(); ++i){
  px_coordinates.push_back(cv::Point2f((Corners_3d[i].at<double>(0)-min_x)*100,(Corners_3d[i].at<double>(1)-min_y)*100)); // in cm <=> 1px
  max_x = max(max_x, int(px_coordinates[i].x));
  max_y = max(max_y, int(px_coordinates[i].y));
 }

 // ROS_INFO("max: %i %i",max_y,max_x);
 cv::Mat search_img(max_y,max_x,CV_8UC1); search_img.setTo(0);
 cv::fillConvexPoly(search_img,px_coordinates,CV_RGB(255,255,255));


 cv::imwrite("data/search_img.jpg", search_img);

 //#ifdef SHOW_SEARCH_IMAGE
 // cv::namedWindow("search_img",1);
 // cv::imshow("search_img", search_img);
 // cv::waitKey(10);
 //#endif


 // find largest rect in white area:

 // ratio = width/height
 bool finished = false;

 float step = 0.02; // check every X m

 float width, height; int x, y;
 for (width = max_x; width > 0 && !finished; width -= step){
  height = width/ratio;

  // ROS_INFO("Width: %f, height: %f", width, height);

  // find fitting left upper corner (sliding window)
  for (x = 0; x < max_x-width && !finished; x+= step*100){
   for (y = 0; y < max_y-height; y+=step*100){
    // ROS_INFO("Checking x = %i, y = %i", x, y);

    int x_w = x+width; int y_w = y+height;
    assert(x >= 0 && y >= 0 && x_w < search_img.cols && y< search_img.rows);
    // check if all corners are withing white area:
    if (search_img.at<uchar>(y,x) == 0) continue;
    if (search_img.at<uchar>(y,x_w) == 0) continue;
    if (search_img.at<uchar>(y_w,x_w) == 0) continue;
    if (search_img.at<uchar>(y_w,x) == 0) continue;
    // ROS_INFO("Found fitting pose (w,h: %f %f)", width, height);
    //#ifdef SHOW_SEARCH_IMAGE
    cv::rectangle(search_img, cv::Point(x,y), cv::Point(x_w, y_w), CV_RGB(125,125,125));
    //#endif

    finished = true; // break outer loops
    break;
   } // for y
  } // for x
 } // for width

#ifdef SHOW_SEARCH_IMAGE
 cv::imshow("search_img", search_img);
 cv::waitKey(10);
#endif

 if (!finished)
  return false;


 cv::imwrite("data/search_img_final.jpg", search_img);

 // restore pose in wall_frame
 optimal_projection_area.width = width/100;
 optimal_projection_area.height = height/100;
 optimal_projection_area.x = x/100.0+min_x;
 optimal_projection_area.y = y/100.0+min_y;

 // show area on input image:

 ROS_INFO("Optimal rect: x,y: %f %f, w,h: %f %f", optimal_projection_area.x, optimal_projection_area.y, optimal_projection_area.width, optimal_projection_area.height);

 setupImageProjection(optimal_projection_area, cv::Size(img_px_size[1],img_px_size[0]));

 msg << "found optimal image region";

 return true;

}




bool Projector_Calibrator::findOptimalProjectionArea(float ratio, cv_RectF& rect,std::stringstream& msg){
 // #define SHOW_SEARCH_IMAGE


 if(!kinect_trafo_valid){
  ROS_WARN("findOptimalProjectionArea: no kinect trafo!"); return false;
 }

 if (detected_corners.size() == 0){
  ROS_WARN("findOptimalProjectionArea: no corners!"); return false;
 }

 if (cloud_moved.size() == 0){
  ROS_WARN("findOptimalProjectionArea: no input cloud!"); return false;
 }


 vector<cv::Point2i> c;
 getCheckerboardArea(c);

 // get 3d coordinates of corners in the wall-system
 vector<cv::Point2f> rotated;
 float min_x = 1e10;
 float min_y = 1e10;



 for (uint i=0; i<c.size(); ++i){
  // ROS_INFO("c: %i %i", c[i].x,c[i].y);
  if (!(c[i].x >= 0 && c[i].x < int(cloud_moved.width) && c[i].y>=0 && c[i].y < int(cloud_moved.height))){
   ROS_WARN("Checkerboard to close to image border! [activate #define SHOW_MASK_IMAGE]"); return false;
  }
  pcl_Point p = cloud_moved.at(c[i].x,c[i].y);

  //   pcl_Point q = input_cloud.at(c[i].x,c[i].y);
  //
  //   ROS_INFO("P: %f %f %f", p.x,p.y,p.z);
  //   ROS_INFO("Q: %f %f %f", q.x,q.y,q.z);


  if (!(p.x == p.x)) {ROS_WARN("Found NAN in input cloud, move camera a bit and rerun"); return false; }
  rotated.push_back(cv::Point2f(p.x,p.y));
  min_x = min(min_x, p.x);
  min_y = min(min_y, p.y);
  // ROS_INFO("pre: %f %f", rotated[i].x, rotated[i].y);
 }

 vector<cv::Point2i> pt_i;
 int max_x, max_y;
 max_x = max_y = -1;

 // ROS_INFO("min: %f %f", min_x, min_y);
 for (uint i=0; i<c.size(); ++i){

  rotated[i] = cv::Point2f((rotated[i].x-min_x)*100,(rotated[i].y-min_y)*100); // in cm <=> 1px
  pt_i.push_back(cv::Point2i(rotated[i].x,rotated[i].y));
  max_x = max(max_x, pt_i[i].x);
  max_y = max(max_y, pt_i[i].y);

 }


 cv::Mat search_img(max_y,max_x,CV_8UC1); search_img.setTo(0);
 cv::fillConvexPoly(search_img,pt_i,CV_RGB(255,255,255));


#ifdef SHOW_SEARCH_IMAGE
 cv::namedWindow("search_img",1);
 cv::imshow("search_img", search_img);
 cv::waitKey(10);
#endif


 // find largest rect in white area:

 // ratio = width/height
 bool finished = false;

 float step = 0.02; // check every X m

 float width, height; int x, y;
 for (width = max_x; width > 0 && !finished; width -= step){ // check every 5 cm
  height = width/ratio;

  // ROS_INFO("Width: %f, height: %f", width, height);

  // find fitting left upper corner (sliding window)
  for (x = 0; x < max_x-width && !finished; x+= step*100){
   for (y = 0; y < max_y-height; y+=step*100){
    // ROS_INFO("Checking x = %i, y = %i", x, y);

    int x_w = x+width; int y_w = y+height;
    assert(x >= 0 && y >= 0 && x_w < search_img.cols && y< search_img.rows);
    // check if all corners are withing white area:
    if (search_img.at<uchar>(y,x) == 0) continue;
    if (search_img.at<uchar>(y,x_w) == 0) continue;
    if (search_img.at<uchar>(y_w,x_w) == 0) continue;
    if (search_img.at<uchar>(y_w,x) == 0) continue;
    // ROS_INFO("Found fitting pose (w,h: %f %f)", width, height);
#ifdef SHOW_SEARCH_IMAGE
    cv::rectangle(search_img, cv::Point(x,y), cv::Point(x_w, y_w), CV_RGB(125,125,125));
#endif

    finished = true; // break outer loops
    break;
   } // for y
  } // for x
 } // for width

#ifdef SHOW_SEARCH_IMAGE
 cv::imshow("search_img", search_img);
 cv::waitKey(10);
#endif

 if (!finished) return false;

 // restore pose in wall_frame
 rect.width = width/100;
 rect.height = height/100;

 rect.x = x/100.0+min_x;
 rect.y = y/100.0+min_y;

 // show area on input image:

 //  ROS_INFO("Optimal rect: x,y: %f %f, w,h: %f %f", rect.x, rect.y, rect.width, rect.height);

 return true;

}

void Projector_Calibrator::setInputCloud(Cloud& cloud){
 // #define COMPUTE_NANS

 if (cloud.size() == 0){
  ROS_WARN("Tried to set new cloud to empty cloud!");
  return;
 }

 input_cloud = cloud;

#ifdef COMPUTE_NANS
 // count invalid points:
 int input_nan = 0;

 for (uint i=0; i<cloud.size(); ++i) {
  pcl_Point p = cloud[i];
  if (!(p.x == p.x)) input_nan++;
 }

 int output_nan = 0;
#endif

 if (kinect_trafo_valid){
  pcl::getTransformedPointCloud(input_cloud,kinect_trafo,cloud_moved);
#ifdef COMPUTE_NANS
  for (uint i=0; i<cloud_moved.size(); ++i) {
   pcl_Point p = cloud_moved[i];
   if (!(p.x == p.x)) output_nan++;
  }
#endif

 }else{
  cloud_moved.clear();
 }


#ifdef COMPUTE_NANS
 ROS_INFO("NAN: input: %i, output: %i", input_nan, output_nan);
#endif


}


void Projector_Calibrator::createMaskFromDetections(){
 // #define SHOW_MASK_IMAGE

 if (detected_corners.size() != uint(checkboard_size.width*checkboard_size.height)){
  ROS_INFO("can't create mask if the corners were not detected!"); return; }

 mask = cv::Mat(cv::Size(640,480), CV_8UC1);  mask.setTo(0);

 vector<cv::Point2i> c;
 getCheckerboardArea(c);


 cv::fillConvexPoly(mask,c,CV_RGB(255,255,255));

 ROS_INFO("Writing kinect_mask to data/kinect_mask.png");
 cv::imwrite("data/kinect_mask.png", mask);


#ifdef SHOW_MASK_IMAGE
 cv::Mat cpy = input_image.clone();

 for (uint i=0; i<c.size(); ++i){
  cv::circle(cpy, c[i] ,10,CV_RGB(255,0,0));
  ROS_INFO("%i %i", c[i].x, c[i].y);
 }

 cv::namedWindow("Mask on Kinect Image");
 cv::imshow("Mask on Kinect Image", cpy);
 cv::waitKey(-1);

#endif
}



float Projector_Calibrator::fitPlaneToCloud(const Cloud& cloud, Eigen::Vector4f& model){
 // ROS_INFO("Fitting plane to cloud with %zu points", cloud.size());

 if (cloud.size() < 1000){
  ROS_WARN("fitPlaneToCloud: cloud size very small: %zu", cloud.size());
 }


 // http://pointclouds.org/documentation/tutorials/random_sample_consensus.php#random-sample-consensus
 pcl::SampleConsensusModelPlane<pcl_Point>::Ptr
 model_p (new pcl::SampleConsensusModelPlane<pcl_Point> (cloud.makeShared()));

 pcl::RandomSampleConsensus<pcl_Point> ransac(model_p);
 ransac.setDistanceThreshold(0.005); // max dist of 5mm
 ransac.computeModel();

 Eigen::VectorXf c;
 ransac.getModelCoefficients(c);
 model = c;

 std::vector<int> inliers;
 ransac.getInliers(inliers);
 float inlier_pct = inliers.size()*100.0/cloud.size();

 if (inlier_pct<0.5){ ROS_WARN("Only %.3f %%  inlier in fitPlaneToCloud!", inlier_pct); }
 return inlier_pct;
}





void Projector_Calibrator::applyMaskOnInputCloud(Cloud& out){

 if (!mask_valid()) createMaskFromDetections();

 assert(mask_valid() && int(input_cloud.width) == mask.cols);

 applyMaskOnCloud(mask, input_cloud, out);

 /*for (int x=0; x<mask.cols; ++x)
  for (int y=0; y<mask.rows; ++y){
   if (mask.at<uchar>(y,x) > 0){
    pcl_Point p = input_cloud.at(x,y);
    if (p.x == p.x) out.points.push_back(p);
   }
  }
  */
}




void Projector_Calibrator::projectSmallCheckerboard(cv::Point l1, cv::Point l2){
 drawCheckerboard(projector_image, l1,l2,
   checkboard_size,
   current_projector_corners);

 // IplImage proj_ipl = projector_image;
 // cvShowImage("fullscreen_ipl", &proj_ipl);
}



void Projector_Calibrator::projectUniformBackground(bool white){
 projector_image.setTo(white?255:0);
 current_projector_corners.clear(); // no corners on projector
 detected_corners.clear();           // and no detected corners anymore
 // IplImage proj_ipl = projector_image;
 // cvShowImage("fullscreen_ipl", &proj_ipl);
}



void Projector_Calibrator::projectFullscreenCheckerboard(){

 checkboard_size.width = max_checkerboard_width;
 checkboard_size.height = max_checkerboard_height;


 drawCheckerboard(projector_image, cv::Point(0,0),
   cv::Point(projector_image.cols, projector_image.rows),
   checkboard_size,
   current_projector_corners);

 // IplImage proj_ipl = projector_image;
 // cvShowImage("fullscreen_ipl", &proj_ipl);
}


bool Projector_Calibrator::saveKinectTrafo(std::stringstream& msg){

 if (!isKinectTrafoSet()){
  msg << "Can't save kinect trafo since it is not computed";
  return false;
 }


 char fn[100]; sprintf(fn, "data/%s.txt",kinect_trafo_filename.c_str());
 if (saveAffineTrafo(kinect_trafo,fn))
  msg << "Wrote kinect_trafo to " <<  fn;
 else
  msg << "Problems writing kinect trafo to " <<  fn;

 return true;
}

void Projector_Calibrator::translateKinectTrafo(float dz){
 assert(isKinectTrafoSet());
 kinect_trafo(2,3) += dz;
}

//  apply yaw rotation matrix on kinect trafo
void Projector_Calibrator::rotateKinectTrafo(float dyaw){

 assert(isKinectTrafoSet());

 Eigen::Affine3f t = Eigen::Affine3f::Identity();

 t(0,0) = t(1,1) = cos(dyaw);
 t(0,1) = sin(dyaw);
 t(1,0) = -sin(dyaw);

 kinect_trafo = t*kinect_trafo;
}



/*
 * Create 3D position of checkerboardcorners in the xy-plane so that z-axis intersects in on of the center corners and corners are axis-aligned
 * PROBLEM: synthetic markers are in 2d (z=0), so that input-matrix for SVD is rankdefficientMa
 */
/*
void Projector_Calibrator::computeKinectTrafoFromMarkerobservations(std::stringstream& msg){

 if (!corners.size() == checkboard_size.width*checkboard_size.height){
  ROS_WARN("computeKinectTrafoFromMarkerobservations: number of observed corners does not correspond to corners on marker");
  return;
 }

 if (!input_cloud.size() > 0){
  ROS_WARN("computeKinectTrafoFromMarkerobservations called with empty input_cloud!");
  return;
 }

 float w = printed_marker_size_mm/1000; // clouds are in m

 // create pointcloud with positions for the printed markers:
 int off_x = checkboard_size.width/2;
 int off_y = checkboard_size.height/2;

 pcl_Point p; p.z = 0;

 Cloud marker_points_3d;
 marker_points_3d.reserve(corners.size());
 for (int j=0; j<checkboard_size.height; ++j){
  for (int i=0; i<checkboard_size.width; ++i){

   p.x = (i-off_x)*w;
   p.y = (j-off_y)*w;
   marker_points_3d.push_back(p);

  // ROS_INFO("marker: %f %f %f", p.x,p.y,p.z);
  }
 }


 // compute observations in 3d:
 Cloud observations;
 observations.reserve(corners.size());
 for (uint i=0; i<corners.size(); ++i){
  pcl_Point p3 = input_cloud.at(corners[i].x, corners[i].y);
  // ROS_INFO("obs: %f %f %f", p3.x,p3.y,p3.z);

  if (!(p3.x == p3.x)){ROS_WARN("computeKinectTrafoFromMarkerobservations: Found Corner without depth!"); return;}
  observations.points.push_back(p3);
 }

 float max_dist = 2*w;

 // returns false if mean error after trafo is larger than max_dist
 kinect_trafo_valid = computeTransformationFromPointclouds(marker_points_3d, observations, kinect_trafo, max_dist);

 if (kinect_trafo_valid)
  msg << "Computed Transformation from Kinect to Marker-Frame";
 else
  msg << "Could not compute trafo, something happened...";


}
 */



