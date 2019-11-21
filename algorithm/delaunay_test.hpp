//
// Created by junlinp on 2019-11-09.
//

#ifndef OPENCV_TEST_ALGORITHM_DELAUNAY_TEST_HPP_
#define OPENCV_TEST_ALGORITHM_DELAUNAY_TEST_HPP_
#include "delaunay.hpp"
#include "fstream"

#include "gtest/gtest.h"
#include <Eigen/Dense>
TEST(Delaunary, IsBadTriangle) {
  algorithm::TempTriangle triangle;
  algorithm::TempPoint a;
  a.coordinate = Eigen::Vector2d(0.0, 0.0);
  algorithm::TempPoint b;
  b.coordinate = Eigen::Vector2d(1.0, 0.0);
  algorithm::TempPoint c;
  c.coordinate = Eigen::Vector2d(0.0, 1.0);
  triangle.vertexs.push_back(a);
  triangle.vertexs.push_back(b);
  triangle.vertexs.push_back(c);
  algorithm::TempPoint point;
  point.coordinate = Eigen::Vector2d(0.1, 0.1);
  point.coordinate = Eigen::Vector2d(-6.32847e-6, 7.74277-06);
  IsBadTriangle(point, triangle);
  double PI = 3.1415926535;
  for(double r = 1e-5; r < sqrt(0.5); r += 1e-3) {
    for(double theta = 1e-6; theta < 2 * PI; theta += 1e-3) {
      point.coordinate = Eigen::Vector2d(r * cos(theta) + 0.5, r * sin(theta) + 0.5);
      ASSERT_TRUE( algorithm::IsBadTriangle(point, triangle));
    }
  }

}
TEST(Delaunays, Delaunarys) {

  std::vector<Eigen::Vector2d> points;
  points.push_back(Eigen::Vector2d(0.0, 0.0));
  points.push_back(Eigen::Vector2d(2.0, 0.0));
  points.push_back(Eigen::Vector2d(-2.0, 0.0));
  points.push_back(Eigen::Vector2d(0.0, 2.0));
  points.push_back(Eigen::Vector2d(0.0, -2.0));
  std::vector<algorithm::TempPoint> temp_points;
  int id = 0;
  for(auto item : points) {
    algorithm::TempPoint t;
    t.id = id++;
    t.coordinate = item;
    temp_points.emplace_back(t);
  }
  std::vector<algorithm::TempTriangle> result;

  algorithm::Delaunay(temp_points, result);

  ASSERT_EQ(result.size(), 4);
  std::ofstream fout("trinangle.txt");

  for(int i = 0; i < result.size(); ++i) {
    algorithm::TempTriangle triangle = result[i];
    fout << triangle.vertexs[0].coordinate  <<
    triangle.vertexs[1].coordinate <<
    triangle.vertexs[2].coordinate << std::endl;
  }

  fout.close();

}
TEST(Delaunays, TempDelaunarys) {
  struct PointXYZ {
   double x, y, z;
   PointXYZ(double x = 0.0,double y = 0.0,double z = 0.0) : x(x), y(y), z(z){};
  };
  std::vector<PointXYZ> points;
  points.push_back(PointXYZ(0.0, 0.0, 0.0));
  points.push_back(PointXYZ(2.0, 0.0, 0.0));
  points.push_back(PointXYZ(-2.0, 0.0, 0.0));
  points.push_back(PointXYZ(0.0, 2.0, 0.0));
  points.push_back(PointXYZ(0.0, -2.0, 0.0));


 std::vector<algorithm::WrapTriangle<PointXYZ>> result;

  algorithm::Delaunay(points, result);

  ASSERT_EQ(result.size(), 4);
}
#endif //OPENCV_TEST_ALGORITHM_DELAUNAY_TEST_HPP_
