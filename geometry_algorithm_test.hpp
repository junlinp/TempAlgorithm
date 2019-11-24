//
// Created by junlinp on 2019-11-24.
//

#ifndef OPENCV_TEST__GEOMETRY_ALGORITHM_TEST_HPP_
#define OPENCV_TEST__GEOMETRY_ALGORITHM_TEST_HPP_
#include "geometry_algorithm.hpp"
#include "gtest/gtest.h"
#include "opencv2/core.hpp"
#include "random"
TEST(geometry_algorithm, IsPointInPolygon) {

  std::vector<cv::Point2d> polygon;
  polygon.push_back(cv::Point2d(5.0, 5.0));
  polygon.push_back(cv::Point2d(5.0, -5.0));
  polygon.push_back(cv::Point2d(-5.0, -5.0));
  polygon.push_back(cv::Point2d(-5.0, 5.0));
  std::default_random_engine engine;
  std::uniform_real_distribution<double> distribution(-10, 10);
  std::function<double()> generate = std::bind(distribution, engine);
  for(int i = 0; i < 1024; i++) {
    cv::Point2d p(generate(), generate());

    if (p.x >= -5 && p.x <= 5 && p.y >= -5 && p.y <= 5) {
      ASSERT_TRUE(IsPointInPolygon(polygon, p));
    } else {
      ASSERT_FALSE(IsPointInPolygon(polygon, p));
    }
  }
}
#endif //OPENCV_TEST__GEOMETRY_ALGORITHM_TEST_HPP_
