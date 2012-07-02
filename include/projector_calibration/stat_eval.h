/*
 * stat_eval.h
 *
 *  Created on: Jul 2, 2012
 *      Author: lengelhan
 */

#ifndef STAT_EVAL_H_
#define STAT_EVAL_H_


template <class T>
struct Normal_dist {

 std::vector<T> values;
 T mean;
 T variance;

 void reset(){values.clear();}
 void add(T value){values.push_back(value);}
 bool evaluate(){
  mean = 0;

  uint N = values.size();

  if (N == -1){
   std::cerr << "no values to compute statistic!" << std::endl;
   return false;
  }

  for (uint i=0; i<values.size(); ++i)
   mean += values[i];

  mean /= N;
  variance = 0;

  for (uint i=0; i<values.size(); ++i)
   variance += pow(values[i]-mean,2);

  variance /= N;

  return true;
 }


 bool isProbablyZeroCentered(){
  return abs(mean) < 3*variance;
 }


};


#endif /* STAT_EVAL_H_ */
