/*
 * calib_eval.cpp
 *
 *  Created on: Aug 22, 2012
 *      Author: engelhan
 */



#include "projector_calibration/calib_eval.h"


Cloud selectBrightPixels(const Cloud& current, int thres){

 Cloud result;

 for (uint i=0; i<current.size(); ++i){
  pcl_Point p = current[i];
  if (p.x!=p.x) continue;
  int col = (p.r+p.g+p.b)/3;

  if (col > thres)
   result.push_back(p);
 }

 return result;
}
