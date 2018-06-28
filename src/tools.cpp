#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0,0,0,0;
  
  if ((estimations.size() == 0) ||
      (ground_truth.size() == 0) ||
      (estimations.size() != ground_truth.size()))
  {
    std::cout << "CalculateRMSE - Error - Input vector dimensions are not as expected";
    std::cout << "(e:"<< estimations.size() << ", g:" << ground_truth.size() << ")" << std::endl;
	    
    return rmse;
  }

  VectorXd res;
  for(int i=0; i < estimations.size(); ++i){
    res = estimations[i] - ground_truth[i];
    res = res.array() * res.array();
    rmse += res;
		
   }

   rmse = rmse/estimations.size();
   rmse = rmse.array().sqrt();

   return rmse;
}
