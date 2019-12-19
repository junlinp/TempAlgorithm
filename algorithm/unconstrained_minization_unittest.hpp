//
// Created by junlinp on 2019-12-14.
//

#ifndef ALGORITHM_ALGORITHM_UNCONSTRAINED_MINIZATION_UNITTEST_HPP_
#define ALGORITHM_ALGORITHM_UNCONSTRAINED_MINIZATION_UNITTEST_HPP_
#include "gtest/gtest.h"
#include "unconstrained_minization.hpp"
TEST(Unconstrained_minization, Quadratic) {
  Eigen::Matrix3d P;
  P << 1.0, 0.0, 0.0,
       0.0, 2.0, 0.0,
       0.0, 0.0, 3.0;
  Eigen::Vector3d q;
  q << 0.0, 0.0, 0.0;
  double r = 0;
  Quadratic quadratic(P, q, r);
  Eigen::Vector3d x;
  x << 10.0, 20.0, 30.0;
  Eigen::Vector3d gradient = -quadratic.Gradient(x);
  FunctionInterface* functor = dynamic_cast<FunctionInterface*>(&quadratic);
  GradientDescent(functor, x);
  std::cout << x << std::endl;
}
#endif //ALGORITHM_ALGORITHM_UNCONSTRAINED_MINIZATION_UNITTEST_HPP_
