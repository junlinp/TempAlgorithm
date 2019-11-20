//
// Created by junlinp on 2019-11-09.
//

#ifndef OPENCV_TEST_ALGORITHM_DELAUNAY_HPP_
#define OPENCV_TEST_ALGORITHM_DELAUNAY_HPP_
#include <vector>
#include <Eigen/Dense>

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

} // namespace algorithm
#endif //OPENCV_TEST_ALGORITHM_DELAUNAY_HPP_
