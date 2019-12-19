//
// Created by junlinp on 2019-12-14.
//

#ifndef ALGORITHM_ALGORITHM_UNCONSTRAINED_MINIZATION_HPP_
#define ALGORITHM_ALGORITHM_UNCONSTRAINED_MINIZATION_HPP_
#include "Eigen/Dense"
class FunctionInterface {
 public:
  virtual double Inference(Eigen::Vector3d& x) = 0;
  virtual Eigen::Vector3d Gradient(Eigen::Vector3d& x) = 0;
};

class Quadratic : public FunctionInterface {
 public:
  Quadratic(Eigen::Matrix3d P, Eigen::Vector3d q, double r);
 private:
  Eigen::Matrix3d P;
  Eigen::Vector3d q;
  double r;
 public:
  double Inference(Eigen::Vector3d& x) override;

  Eigen::Vector3d Gradient(Eigen::Vector3d& x) override;
};

double BackTraing(FunctionInterface* function,Eigen::Vector3d& x, Eigen::Vector3d& decent_direct);
void GradientDescent(FunctionInterface* function, Eigen::Vector3d& x);
#endif //ALGORITHM_ALGORITHM_UNCONSTRAINED_MINIZATION_HPP_
