/*
 * calib_eval.h
 *
 *  Created on: Aug 22, 2012
 *      Author: engelhan
 */

#ifndef CALIB_EVAL_H_
#define CALIB_EVAL_H_


#include "rgbd_utils/calibration_utils.h"


/*
 *
 * thres: maximal mean rgb-value
 */
Cloud selectBrightPixels(const Cloud& current, int thres = 240);


#endif /* CALIB_EVAL_H_ */
