//
// Created by junlinp on 2019-11-24.
//

#ifndef OPENCV_TEST__GEOMETRY_ALGORITHM_HPP_
#define OPENCV_TEST__GEOMETRY_ALGORITHM_HPP_
#include <Eigen/Dense>

template <class T = Eigen::Vector3d>
bool IsLineIntersect(const T& line_start, const T& line_end, const T& line_two_start, const T& line_two_end) {
  Eigen::Matrix<double, 3, 2> A;
  Eigen::Matrix3d A_b(3, 3);
  A << (line_start - line_end), (line_two_end - line_two_start);
  A_b << A, (line_two_end - line_end);
  Eigen::FullPivLU<Eigen::Matrix<double, 3, 2>> lu_A(A);
  Eigen::FullPivLU<Eigen::Matrix3d> lu_A_b(A_b);

  if (lu_A.rank() != lu_A_b.rank()) {
    return false;
  }
  Eigen::ColPivHouseholderQR<Eigen::Matrix<double, 3, 2>> des(A);
  Eigen::Vector2d x =  des.solve((line_two_end - line_end));

  return x(0, 0) >= 0 && x(0, 0) <= 1 && x(1, 0) >=0 && x(1, 0) <= 1;
}

template <class T>
bool IsPointInPolygon(std::vector<T> polygon, T point) {
  bool b_in = false;

  for(int i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
    if ( (polygon[i].y > point.y && polygon[j].y <= point.y) || (polygon[j].y > point.y && polygon[i].y <= point.y)) {
      double x = (point.y - polygon[j].y) * (polygon[i].x - polygon[j].x) / (polygon[i].y - polygon[j].y) + polygon[j].x;
      if (x <= point.x) {
        b_in = b_in ? false : true;
      }
    }
  }
  return b_in;
}
#endif //OPENCV_TEST__GEOMETRY_ALGORITHM_HPP_
