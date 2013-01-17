/*
 * stat_eval.h
 *
 *  Created on: Jul 2, 2012
 *      Author: lengelhan
 */

#ifndef STAT_EVAL_H_
#define STAT_EVAL_H_

/**
 * Basic Implementation of normal distribution
 */
template <class T>
struct Normal_dist {

 /// samples from the distribution
 std::vector<T> values;

 /// mean value of distribution
 T mean;

 /// variance of distribution
 T variance;

 /// all stored values are deleted
 void reset(){values.clear();}

 /// a new measurement is added
 void add(T value){values.push_back(value);}

 /**
  * @brief Computation of first moments of normal distribution
  * @return true if data was given
  */
 bool evaluate(){
  mean = 0;

  uint N = values.size();

  if (N == 0){
   std::cerr << "no values to compute statistic!" << std::endl;
   return false;
  }

  for (uint i=0; i<values.size(); ++i)
   mean += values[i];

  mean /= N;
  variance = 0;

  for (uint i=0; i<N; ++i)
   variance += pow(values[i]-mean,2);

  variance /= N;

  return true;
 }


 /// return abs(mean) < variance;
 bool isProbablyZeroCentered(){
  return abs(mean) < variance;
 }


};


#endif /* STAT_EVAL_H_ */
