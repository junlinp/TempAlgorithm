//
// Created by junlinp on 2019-11-09.
//

#ifndef OPENCV_TEST_ALGORITHM_DELAUNAY_HPP_
#define OPENCV_TEST_ALGORITHM_DELAUNAY_HPP_
#include <vector>
#include <memory>
#include <Eigen/Dense>

#include <iostream>
namespace algorithm {
struct TempPoint {
  int id;
  Eigen::Vector2d coordinate;
};
typedef std::pair<TempPoint, TempPoint> Edge;
struct Triangle {
  int id;
  std::vector<TempPoint> vertexs;
};


struct TempTriangle {
  int id;
  std::vector<TempPoint> vertexs;

  Edge GetEdge(int index) {
    Edge edge;
    edge.first = vertexs[index];
    edge.second = vertexs[(index + 1) % 3];
    return edge;
  }

};

bool IsBadTriangle(const TempPoint &point, const TempTriangle &triangle);
inline bool IsSameEdge(const Edge &lhs_edge, const Edge &rhs_edge) {
  return (lhs_edge.first.id == rhs_edge.first.id && lhs_edge.second.id == rhs_edge.second.id) ||
      (lhs_edge.first.id == rhs_edge.second.id && lhs_edge.second.id == rhs_edge.first.id);
}
TempTriangle GenerateTriangle(const Edge &edge, const TempPoint &point);
bool IsAdjacentEdge(const Edge &edge, const TempTriangle &triangle);

bool Delaunay(const std::vector<TempPoint> &points, std::vector<TempTriangle> &result);

template<class T>
struct WrapPoint {
  int id;
  std::shared_ptr<T> entity;

  double x() const { return entity->x;}
  double y() const {return entity->y;}
  double z() const {return entity->z;}
};
template<class T>
struct WrapEdge {
  WrapPoint<T> first, second;
};

template<class T>
struct WrapTriangle {
  int id;
  std::vector<WrapPoint<T>> vertexs;

  WrapEdge<T> GetEdge(int idx) const {
    WrapEdge<T> edge;
    edge.first = vertexs[idx];
    edge.second = vertexs[(idx + 1) % 3];
    return std::move(edge);
  }

};

template<class T>
bool IsSameEdge(const WrapEdge<T> &lhs_edge, const WrapEdge<T> &rhs_edge) {
  return (lhs_edge.first.id == rhs_edge.first.id && lhs_edge.second.id == rhs_edge.second.id) ||
      (lhs_edge.first.id == rhs_edge.second.id && lhs_edge.second.id == rhs_edge.first.id);
}
template<class T>
bool IsBadTriangle(const WrapPoint<T>& point, const WrapTriangle<T>& triangle) {
  const auto& p0 = triangle.vertexs[0];
  const auto& p1 = triangle.vertexs[1];
  const auto& p2 = triangle.vertexs[2];
  Eigen::Vector3d p0_vec, p1_vec, p2_vec;
  p0_vec << p0.entity->x, p0.entity->y, p0.entity->z;
  p1_vec << p1.entity->x, p1.entity->y, p1.entity->z;
  p2_vec << p2.x(), p2.y(), p2.z();

  Eigen::Vector3d a = p1_vec - p0_vec;
  Eigen::Vector3d b_ = p2_vec - p0_vec;
  Eigen::Vector3d cross_vector;
  cross_vector << a(1, 0) * b_(2, 0) - a(2, 0) * b_(1, 0),
                  -a(0, 0) * b_(2, 0) + a(2, 0) * b_(0, 0),
                  a(0, 0) * b_(1, 0) - a(1, 0) * b_(0,0);
  Eigen::Matrix3d A;
  A << p0.entity->x - p1.entity->x, p0.entity->y - p1.entity->y, p0.entity->z - p1.entity->z,
       p0.entity->x - p2.entity->x, p0.entity->y - p2.entity->y, p0.entity->z - p2.entity->z,
       cross_vector(0, 0) , cross_vector(1, 0), cross_vector(2, 0);
  A = 2.0 * A;
  Eigen::Vector3d b;
  b << p0_vec.dot(p0_vec) - p1_vec.dot(p1_vec),
       p0_vec.dot(p0_vec) - p2_vec.dot(p2_vec),
       cross_vector.dot(p0_vec);
  Eigen::ColPivHouseholderQR<Eigen::Matrix3d> dec(A);
  Eigen::Vector3d x = dec.solve(b);
  Eigen::Vector3d p_vec;
  p_vec << point.x() , point.y() , point.z();
  double lhs = (x - p0_vec).norm();
  double rhs = (x - p_vec).norm();
  return lhs - rhs >= 1e-6;
}
template<class T>
bool IsAdjacentEdge(const WrapEdge<T> &edge, const WrapTriangle<T>& triangle) {
  return IsSameEdge(edge, triangle.GetEdge(0)) ||
      IsSameEdge(edge, triangle.GetEdge(1)) ||
      IsSameEdge(edge, triangle.GetEdge(2));
}
template<class T>
auto GenerateTriangle(const WrapEdge<T>& edge, const WrapPoint<T>& point) -> decltype(WrapTriangle<T>()){
  WrapTriangle<T> triangle;
  triangle.vertexs.push_back(point);
  triangle.vertexs.push_back(edge.first);
  triangle.vertexs.push_back(edge.second);
  return std::move(triangle);
}
template<class T>
bool TriangleHasPoint(const WrapTriangle<T>& triangle, const WrapPoint<T>& point) {

  return triangle.vertexs[0].id == point.id || triangle.vertexs[1].id == point.id || triangle.vertexs[2].id == point.id;
}
template<class T>
bool Delaunay(const std::vector<T>& points, std::vector<WrapTriangle<T>> &result) {
  if (points.size() < 3) {
    return false;
  }
  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::min();
  double max_y = std::numeric_limits<double>::min();
  double min_z = std::numeric_limits<double>::max();
  double max_z = std::numeric_limits<double>::min();

  std::vector<WrapPoint<T>> v_temp_points;
  int id = 0;
  for(const auto& p : points) {
    WrapPoint<T> tmp_point;
    tmp_point.id = id++;
    tmp_point.entity = std::make_shared<T>(p);
    v_temp_points.emplace_back(tmp_point);
    min_x = std::min(min_x, p.x - 10.0);
    min_y = std::min(min_y, p.x - 10.0);
    max_x = std::max(max_x, p.y + 10.0);
    max_y = std::max(max_y, p.y + 10.0);
    max_z = std::max(max_z, p.z + 10.0);
    min_z = std::min(min_z, p.z - 10.0);
  }
  WrapPoint<T> left_top, left_bottom, right_top, right_bottom;
  left_top.id = points.size();
  left_bottom.id = points.size() + 1;
  right_top.id = points.size() + 2;
  right_bottom.id = points.size() + 3;

  left_top.entity = std::make_shared<T>();
  left_top.entity->x = min_x;
  left_top.entity->y = max_y;
  left_top.entity->z = (max_z + min_z) / 2.0;

  left_bottom.entity = std::make_shared<T>();
  left_bottom.entity->x = min_x;
  left_bottom.entity->y = min_y;
  left_bottom.entity->z = (max_z + min_z) / 2.0;

  right_top.entity = std::make_shared<T>();
  right_top.entity->x = max_x;
  right_top.entity->y = max_y;
  right_top.entity->z = (max_z + min_z) / 2.0;

  right_bottom.entity = std::make_shared<T>();
  right_bottom.entity->x = max_x;
  right_bottom.entity->y = min_y;
  right_bottom.entity->z = (max_z + min_z) / 2.0;

  WrapTriangle<T> init_triangle_1, init_triangle_2;
  init_triangle_1.vertexs.push_back(left_top);
  init_triangle_1.vertexs.push_back(left_bottom);
  init_triangle_1.vertexs.push_back(right_bottom);

  init_triangle_2.vertexs.push_back(right_bottom);
  init_triangle_2.vertexs.push_back(right_top);
  init_triangle_2.vertexs.push_back(left_top);

  std::vector<WrapTriangle<T>> triangle_set;
  triangle_set.emplace_back(init_triangle_1);
  triangle_set.emplace_back(init_triangle_2);
#pragma omp parallel for
  for (const auto& insert_point : v_temp_points) {
    std::vector<WrapTriangle<T>> bad_triangle_set;
    auto iterator = triangle_set.begin();
    while (iterator != triangle_set.end()) {
      if (IsBadTriangle(insert_point, *iterator)) {
        bad_triangle_set.push_back(*iterator);
        iterator = triangle_set.erase(iterator);
      } else {
        ++iterator;
      }
    }

    for(int i  = 0; i < bad_triangle_set.size(); ++i) {
      WrapTriangle<T> current_triangle = bad_triangle_set[i];
      WrapEdge<T> one, two, three;
      one.first = current_triangle.vertexs[0];
      one.second = current_triangle.vertexs[1];
      two.first = current_triangle.vertexs[1];
      two.second = current_triangle.vertexs[2];
      three.first = current_triangle.vertexs[2];
      three.second = current_triangle.vertexs[0];
      bool one_flag =false, two_flag = false, three_flag = false;

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
  for (auto &t : triangle_set) {
    if (TriangleHasPoint(t, left_top) || TriangleHasPoint(t, left_bottom) ||
        TriangleHasPoint(t, right_top) || TriangleHasPoint(t, right_bottom)) {

    } else {
      t.id = triangle_id++;
      result.push_back(std::move(t));
    }
  }
  return true;

}
} // namespace algorithm
#endif //OPENCV_TEST_ALGORITHM_DELAUNAY_HPP_
