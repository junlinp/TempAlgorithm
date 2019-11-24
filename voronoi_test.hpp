//
// Created by junlinp on 2019-11-19.
//

#ifndef OPENCV_TEST__VORONOI_TEST_HPP_
#define OPENCV_TEST__VORONOI_TEST_HPP_
#include <gtest/gtest.h>
#include "voronoi.hpp"

TEST(Voronoi, Voronoi) {

  Boundary boundary;
  boundary.y_max = boundary.x_max = 10.0;
  boundary.y_min = boundary.x_min = -10.0;

  std::vector<Eigen::Vector2d> points;
  points.push_back(Eigen::Vector2d(0.0, 0.0));
  points.push_back(Eigen::Vector2d(2.0, 0.0));
  points.push_back(Eigen::Vector2d(-2.0, 0.0));
  points.push_back(Eigen::Vector2d(0.0, 2.0));
  points.push_back(Eigen::Vector2d(0.0, -2.0));

  std::vector<Polygon> result;
  int ret = voronoi(points, boundary, result);

  ASSERT_TRUE(ret == 1);
  ASSERT_EQ(result.size(), 5);
  int count = 0;
  for(Polygon polygon : result) {
    std::cout << "Polygon " << count++ << std::endl;
    for(int i = 0; i < polygon.vertexs.size(); ++i) {
      std::cout << polygon.vertexs[i](0, 0) << " " << polygon.vertexs[i](1, 0) << " ";
    }
    std::cout << std::endl;
  }
}

TEST(Voronoi, Voronoi3_Triangle) {
  Boundary boundary;
  boundary.y_max = boundary.x_max = 10.0;
  boundary.y_min = boundary.x_min = -10.0;

  std::vector<Eigen::Vector2d> points;
  points.push_back(Eigen::Vector2d(2.0, 0.0));
  points.push_back(Eigen::Vector2d(-2.0, 0.0));
  points.push_back(Eigen::Vector2d(0.0, 0.5));

  std::vector<Polygon> result;
  int ret = voronoi(points, boundary, result);

  ASSERT_TRUE(ret == 1);
  ASSERT_EQ(result.size(), 3);
  int count = 0;
  for(Polygon polygon : result) {
    std::cout << "3 Point Polygon " << count++ << std::endl;
    for(int i = 0; i < polygon.vertexs.size(); ++i) {
      std::cout << polygon.vertexs[i](0, 0) << " " << polygon.vertexs[i](1, 0) << " ";
    }
    std::cout << std::endl;
  }

}
#endif //OPENCV_TEST__VORONOI_TEST_HPP_
