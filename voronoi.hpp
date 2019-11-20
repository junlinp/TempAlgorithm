//
// Created by junlinp on 2019-11-17.
//

#ifndef OPENCV_TEST__VORONOI_HPP_
#define OPENCV_TEST__VORONOI_HPP_
#include "algorithm/delaunay.hpp"
#include <algorithm>
#include <stack>
struct Polygon {
  std::vector<Eigen::Vector2d> vertexs;
};
struct Boundary {
  double x_max, x_min, y_max, y_min;
};
int voronoi(const std::vector<Eigen::Vector2d> points,Boundary boundary, std::vector<Polygon>& result);

template <class T>
bool ConvexHull(std::vector<T> point, std::vector<T>& convex_hull) {
  if (point.size() < 3) {
    return false;
  }
 std::sort(point.begin(), point.end(), [](T lhs, T rhs) { if (lhs.x == rhs.x) return lhs.y < rhs.y;return lhs.x < rhs.x;}) ;
 T base_point = *point.begin();
  std::sort(point.begin() + 1, point.end(), [&base_point](T lhs, T rhs) {
    return atan( (lhs.y - base_point.y) / (lhs.x - base_point.x)) <
          atan( (rhs.y - base_point.y) / (rhs.x - base_point.x));
  });

  std::stack<T> _stack;
  _stack.push(base_point);
  _stack.push(point[1]);

  std::function<bool(T, T, T)> lay_on_right = [](T point, T start, T end) {
    return (end.x - start.x) * point.y - (end.y - start.y) * point.x < 0.0;
  };
  for(int i = 2; i < point.size(); i++) {
    if (i != point.size() - 1) {
      while (lay_on_right(point[i], base_point, _stack.top())) {
        _stack.pop();
      }
      _stack.push(point[i]);
    } else {
      _stack.push(point[i]);
    }
  }

  while(_stack.size()) {
    convex_hull.push_back(_stack.top());
    _stack.pop();
  }
  return true;
}

#endif //OPENCV_TEST__VORONOI_HPP_
