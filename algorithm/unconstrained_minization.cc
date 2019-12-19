//
// Created by junlinp on 2019-12-14.
//

#include "unconstrained_minization.hpp"
Quadratic::Quadratic(Eigen::Matrix3d P, Eigen::Vector3d q, double r) :
  P(P),
  q(q),
  r(r)
{

}
double Quadratic::Inference(Eigen::Vector3d &x) {
  return 0.5 * (x.transpose() * P * x)(0, 0) + (q.transpose() * x)(0, 0) + r;
}
Eigen::Vector3d Quadratic::Gradient(Eigen::Vector3d &x) {
  return P * x + q;
}

double BackTraing(FunctionInterface* function,Eigen::Vector3d& x, Eigen::Vector3d& decent_direct) {
  double alpha = 0.2;
  double beta = 0.6;
  double t = 1.0;
  Eigen::Vector3d temp = x + t * decent_direct;
  double lhs = function->Inference(temp);
  double rhs = function->Inference(x) + alpha * t * (function->Gradient(x).transpose() * decent_direct)(0, 0);
  while(lhs > rhs) {
    t = beta * t;
    temp = x + t * decent_direct;
    lhs = function->Inference(temp);
    rhs = function->Inference(x) + alpha * t * (function->Gradient(x).transpose() * decent_direct)(0, 0);
  }
  return t;
}
void GradientDescent(FunctionInterface *function, Eigen::Vector3d &x) {
    Eigen::Vector3d gradient = -function->Gradient(x);
    while( gradient.norm() > 1e-9) {
      double t = BackTraing(function, x, gradient);
      x = x + t * gradient;
      gradient = -function->Gradient(x);
    }
}
