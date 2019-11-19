//
// Created by junlinp on 2019-11-09.
//
#include "delaunay.hpp"
#include "iostream"

namespace algorithm {

bool IsBadTriangle(const TempPoint &point, const TempTriangle &triangle) {
  Eigen::Vector2d p0 = triangle.vertexs[0].coordinate;
  Eigen::Vector2d p1 = triangle.vertexs[1].coordinate;
  Eigen::Vector2d p2 = triangle.vertexs[2].coordinate;
  Eigen::Matrix2d A;
  Eigen::Vector2d point_coordinate = point.coordinate;
  double xs = point_coordinate(0, 0);
  double ys = point_coordinate(1, 0);
  //std::cout << xs << " " << ys << std::endl;

  A << 2 * (p1(0, 0) - p0(0, 0)), 2 * (p1(1, 0) - p0(1, 0)),
      2 * (p2(0, 0) - p0(0, 0)), 2 * (p2(1, 0) - p0(1, 0));
  Eigen::Vector2d b;
  b << p1(0, 0) * p1(0, 0) + p1(1, 0) * p1(1, 0) - p0(0, 0) * p0(0, 0) - p0(1, 0) * p0(1, 0),
      p2(0, 0) * p2(0, 0) + p2(1, 0) * p2(1, 0) - p0(0, 0) * p0(0, 0) - p0(1, 0) * p0(1, 0);
  //std::cout << "A : " << A << std::endl;
  //std::cout << "b : " << b << std::endl;
  //std::cout << "A^-1 : " << A.inverse() << std::endl;
  Eigen::Vector2d x = A.inverse() * b;
  double lhs = (x - p0).norm();
  double rhs = (x - point.coordinate).norm();
  return lhs - rhs >= 1e-6;

}

bool IsAdjacentEdge(const Edge &edge, const TempTriangle &triangle) {

  return IsSameEdge(edge, Edge(triangle.vertexs[0], triangle.vertexs[1])) ||
      IsSameEdge(edge, Edge(triangle.vertexs[1], triangle.vertexs[2])) ||
      IsSameEdge(edge, Edge(triangle.vertexs[2], triangle.vertexs[0]));
}

TempTriangle GenerateTriangle(const Edge &edge, const TempPoint &point) {
  Eigen::Vector3d vector1;
  vector1 << edge.second.coordinate(0, 0) - edge.first.coordinate(0, 0),
      edge.second.coordinate(1, 0) - edge.first.coordinate(1, 0),
      1;
  Eigen::Vector3d vector2;
  vector2 << point.coordinate(0, 0) - edge.first.coordinate(0, 0),
      point.coordinate(1, 0) - edge.first.coordinate(1, 0),
      1;

  TempTriangle result;
  if (vector1.cross(vector2)(2, 0) < -1e6) {
    result.vertexs.push_back(point);
    result.vertexs.push_back(edge.second);
    result.vertexs.push_back(edge.first);
  } else {
    result.vertexs.push_back(edge.first);
    result.vertexs.push_back(edge.second);
    result.vertexs.push_back(point);
  }
  return result;
}

bool TriangleHasPoint(const TempTriangle &triangle, const TempPoint &point) {

  return triangle.vertexs[0].id == point.id || triangle.vertexs[1].id == point.id || triangle.vertexs[2].id == point.id;
}

bool Delaunay(const std::vector<TempPoint> &points, std::vector<TempTriangle> &result) {
  if (points.size() < 3) {
    return false;
  }
  std::vector<TempPoint> v_temp_points(points.size());
  double min_x = 1e300, min_y = 1e300;
  double max_x = -1e300, max_y = -1e300;
  for (int i = 0; i < points.size(); ++i) {
    TempPoint tmp_point;
    tmp_point.id = i;
    tmp_point.coordinate = points[i].coordinate;
    v_temp_points.emplace_back(tmp_point);
    min_x = std::min(min_x, points[i].coordinate(0, 0) - 10.0);
    min_y = std::min(min_y, points[i].coordinate(1, 0) - 10.0);
    max_x = std::max(max_x, points[i].coordinate(0, 0) + 10.0);
    max_y = std::max(max_y, points[i].coordinate(1, 0) + 10.0);
  }

  TempPoint left_top, left_bottom, right_top, right_bottom;
  left_top.id = points.size();
  left_bottom.id = points.size() + 1;
  right_top.id = points.size() + 2;
  right_bottom.id = points.size() + 3;
  left_top.coordinate = Eigen::Vector2d(min_x, max_y);
  left_bottom.coordinate = Eigen::Vector2d(min_x, min_y);
  right_top.coordinate = Eigen::Vector2d(max_x, max_y);
  right_bottom.coordinate = Eigen::Vector2d(max_x, min_y);

  TempTriangle init_triangle_1, init_triangle_2;
  init_triangle_1.vertexs.push_back(left_top);
  init_triangle_1.vertexs.push_back(left_bottom);
  init_triangle_1.vertexs.push_back(right_bottom);

  init_triangle_2.vertexs.push_back(right_bottom);
  init_triangle_2.vertexs.push_back(right_top);
  init_triangle_2.vertexs.push_back(left_top);

  std::vector<TempTriangle> triangle_set;
  triangle_set.emplace_back(init_triangle_1);
  triangle_set.emplace_back(init_triangle_2);

  for (TempPoint &insert_point : v_temp_points) {
    std::vector<TempTriangle> bad_triangle_set;
    auto it = triangle_set.begin();

    while (it != triangle_set.end() ) {
      if (IsBadTriangle(insert_point, *it)) {

        bad_triangle_set.push_back(*it);
        it = triangle_set.erase(it);
      } else {
        ++it;
      }
    }
    std::cout << bad_triangle_set.size() << std::endl;
    for (int i = 0; i < bad_triangle_set.size(); ++i) {
      // find the edge
      TempTriangle current_triangle = bad_triangle_set[i];
      Edge one, two, three;
      one.first = current_triangle.vertexs[0];
      one.second = current_triangle.vertexs[1];
      two.first = current_triangle.vertexs[1];
      two.second = current_triangle.vertexs[2];
      three.first = current_triangle.vertexs[2];
      three.second = current_triangle.vertexs[0];

      bool one_flag = false, two_flag = false, three_flag = false;
      for (int j = 0; j < bad_triangle_set.size(); ++j) {
        if (i == j) {
          continue;
        }
        one_flag |= IsAdjacentEdge(one, bad_triangle_set[j]);
        two_flag |= IsAdjacentEdge(two, bad_triangle_set[j]);
        three_flag |= IsAdjacentEdge(three, bad_triangle_set[j]);

      }

      if (!one_flag) {
        triangle_set.push_back(GenerateTriangle(one, insert_point));
      }
      if (!two_flag) {
        triangle_set.push_back(GenerateTriangle(two, insert_point));
      }
      if (!three_flag) {
        triangle_set.push_back(GenerateTriangle(three, insert_point));
      }
    }

  }

  int triangle_id = 0;
  for (TempTriangle &t : triangle_set) {
    if (TriangleHasPoint(t, left_top) || TriangleHasPoint(t, left_bottom) ||
        TriangleHasPoint(t, right_top) || TriangleHasPoint(t, right_bottom)) {

    } else {
      TempTriangle triangle;
      triangle.vertexs = t.vertexs;
      triangle.id = triangle_id++;
      result.push_back(triangle);
    }
  }
  return true;
}
} // namespace algorithm

