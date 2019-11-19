//
// Created by junlinp on 2019-11-17.
//
#include "algorithm/delaunay.hpp"
#include "Eigen/Dense"
#include "voronoi.hpp"
#include "functional"

bool IsTriangleHasPoint(algorithm::TempPoint point, algorithm::TempTriangle triangle) {

  return triangle.vertexs[0].id == point.id || triangle.vertexs[1].id == point.id || triangle.vertexs[2].id == point.id;
}

std::vector<algorithm::TempTriangle> TriangleWithThePoint(algorithm::TempPoint point,
                                                          std::vector<algorithm::TempTriangle> triangles) {
  std::vector<algorithm::TempTriangle> result;
  for (auto tri : triangles) {
    if (IsTriangleHasPoint(point, tri)) {
      result.push_back(tri);
    }
  }

  return result;
}

bool TriangleHasEdge(algorithm::TempTriangle triangle, algorithm::Edge edge) {

  return true;
}

bool IsOpenEdge(algorithm::Edge edge, std::vector<algorithm::TempTriangle> triangles) {

  int count = 0;
  for (auto &tri : triangles) {
    if (algorithm::IsAdjacentEdge(edge, tri)) {
      count++;
    }
  }
  return count == 1;
}

bool IsOpenTriangle(algorithm::TempTriangle triangle,
                    std::vector<algorithm::TempTriangle> triangles,
                    algorithm::TempPoint related_point) {
  bool is_open_triangle = false;
  algorithm::Edge one, two;
  one.first = related_point;
  two.first = related_point;
  if (triangle.vertexs[0].id == related_point.id) {
    one.second = triangle.vertexs[1];
    two.second = triangle.vertexs[2];
  } else if (triangle.vertexs[1].id == related_point.id) {
    one.second = triangle.vertexs[0];
    two.second = triangle.vertexs[2];
  } else if (triangle.vertexs[2].id == related_point.id) {
    one.second = triangle.vertexs[1];
    two.second = triangle.vertexs[0];
  }
  is_open_triangle |= IsOpenEdge(one, triangles);
  is_open_triangle |= IsOpenEdge(two, triangles);
  return is_open_triangle;
}

int OpenTriangleCount(algorithm::TempPoint point, std::vector<algorithm::TempTriangle> delaunay_triangle_network) {
  std::vector<algorithm::TempTriangle> triangle_has_point = TriangleWithThePoint(point, delaunay_triangle_network);
  int count = 0;
  for(auto& triangle : delaunay_triangle_network) {
    if (IsOpenTriangle(triangle, delaunay_triangle_network, point)) {
      count++;
    }
  }
  return count;
}

std::vector<algorithm::Edge> GetOpenEdge(algorithm::TempTriangle triangle, std::vector<algorithm::TempTriangle> related_triangle, algorithm::TempPoint related_point) {
  algorithm::Edge one, two;
  one.first = related_point;
  two.first = related_point;
  if (triangle.vertexs[0].id == related_point.id) {
    one.second = triangle.vertexs[1];
    two.second = triangle.vertexs[2];
  } else if (triangle.vertexs[1].id == related_point.id) {
    one.second = triangle.vertexs[0];
    two.second = triangle.vertexs[2];
  } else if (triangle.vertexs[2].id == related_point.id) {
    one.second = triangle.vertexs[1];
    two.second = triangle.vertexs[0];
  }

  std::vector<algorithm::Edge> result;
  if (IsOpenEdge(one,related_triangle)) {
    result.push_back(one);
  }

  if (IsOpenEdge(two, related_triangle)) {
    result.push_back(two);
  }
  return result;
}
Eigen::Vector2d CirumCircle(const algorithm::TempTriangle& triangle) {
  Eigen::Vector2d p0 = triangle.vertexs[0].coordinate;
  Eigen::Vector2d p1 = triangle.vertexs[1].coordinate;
  Eigen::Vector2d p2 = triangle.vertexs[2].coordinate;
  Eigen::Matrix2d A;
  A << 2 * (p1(0, 0) - p0(0, 0)), 2 * (p1(1, 0) - p0(1, 0)),
      2 * (p2(0, 0) - p0(0, 0)), 2 * (p2(1, 0) - p0(1, 0));
  Eigen::Vector2d b;
  b << p1(0, 0) * p1(0, 0) + p1(1, 0) * p1(1, 0) - p0(0, 0) * p0(0, 0) - p0(1, 0) * p0(1, 0),
      p2(0, 0) * p2(0, 0) + p2(1, 0) * p2(1, 0) - p0(0, 0) * p0(0, 0) - p0(1, 0) * p0(1, 0);
  Eigen::Vector2d x = A.inverse() * b;
  return x;
}

void solve(Eigen::Vector2d center_point, Eigen::Vector2d edge_center_point, algorithm::Edge edge, double& d, double& alpha) {
  Eigen::Vector2d a = edge_center_point - center_point;
  Eigen::Matrix2d A;
  A << (edge.first.coordinate - edge.second.coordinate)(0, 0) , a(0, 0),
      (edge.first.coordinate - edge.second.coordinate)(1, 0) , a(1, 0);
  Eigen::Vector2d b;
  b << edge.second.coordinate(0, 0) - center_point(0, 0),
      edge.second.coordinate(1, 0) - center_point(1, 0);

  Eigen::Vector2d x = A.inverse() * b;
  alpha = x(0, 0);
  d = x(1, 0);

}
Eigen::Vector2d Intersect(Eigen::Vector2d center_point, Eigen::Vector2d edge_center_point, Boundary boundary) {
  algorithm::TempPoint p00, p10, p01 , p11;
  p00.coordinate = Eigen::Vector2d(boundary.x_min, boundary.y_min);
  p10.coordinate = Eigen::Vector2d(boundary.x_max, boundary.y_min);
  p01.coordinate = Eigen::Vector2d(boundary.x_min, boundary.y_max);
  p11.coordinate = Eigen::Vector2d(boundary.x_max, boundary.y_max);
  algorithm::Edge one, two, three, four;
  one.first = p00;
  one.second = p10;
  two.first = p10;
  two.second = p11;
  three.first = p11;
  three.second = p01;
  four.first = p01;
  four.second = p00;

  double d, alpha;
  solve(center_point, edge_center_point, one, d, alpha);
  if (d > 0.0 && alpha >= 0.0 && alpha <= 1.0) {
    return center_point + d * (edge_center_point - center_point);
  }
  solve(center_point, edge_center_point, two, d, alpha);
  if (d > 0.0 && alpha >= 0.0 && alpha <= 1.0) {
    return center_point + d * (edge_center_point - center_point);
  }
  solve(center_point, edge_center_point, three, d, alpha);
  if (d > 0.0 && alpha >= 0.0 && alpha <= 1.0) {
    return center_point + d * (edge_center_point - center_point);
  }
  solve(center_point, edge_center_point, four, d, alpha);
  if (d > 0.0 && alpha >= 0.0 && alpha <= 1.0) {
    return center_point + d * (edge_center_point - center_point);
  }
}
std::vector<Eigen::Vector2d> BoundaryFilter(Eigen::Vector2d cirumcircle_center, Eigen::Vector2d intersect_point,
    algorithm::TempPoint point,std::vector<Eigen::Vector2d> boundary_points) {
  Eigen::Vector2d p;
  p << point.coordinate;

  std::function<double(Eigen::Vector2d, Eigen::Vector2d, Eigen::Vector2d)> functor =
      [](Eigen::Vector2d p, Eigen::Vector2d line_pt1, Eigen::Vector2d line_pt2) {
        return (p - line_pt1)(0, 0) * (line_pt2 - line_pt1)(1, 0) -
            (p - line_pt1)(1, 0) * (line_pt2 - line_pt1)(0, 0);
  };
  bool is_left = functor(p, cirumcircle_center, intersect_point) <= 0.0;

  std::vector<Eigen::Vector2d> result;
  for(auto& boundary_point : boundary_points) {
    if (is_left == (functor(boundary_point, cirumcircle_center, intersect_point) <= 0.0)) {
      result.push_back(boundary_point);
    }
  }
  return result;
}
int voronoi(const std::vector<Eigen::Vector2d> points, Boundary boundary, std::vector<Polygon> result) {

  std::vector<algorithm::TempTriangle> delaunay_triangle_network;
  int id = 0;
  std::vector<algorithm::TempPoint> temp_points;
  for (auto &p : points) {
    algorithm::TempPoint tp;
    tp.coordinate = p;
    tp.id = id++;
    temp_points.emplace_back(tp);
  }
  algorithm::Delaunay(temp_points, delaunay_triangle_network);

  for (auto &point : temp_points) {
    // find the triagnle include the point
    std::vector<algorithm::TempTriangle> triangle = TriangleWithThePoint(point, delaunay_triangle_network);

    // find the triangle which has the open edge (normally has zero or more then one)
    int open_triangle_count = OpenTriangleCount(point, delaunay_triangle_network);
    //Polygon polygon;
    // if two signle edge then compute the two signle triangle and they intersect with the boundary

    std::vector<Eigen::Vector2d> points_need_convex_hull;
    std::vector<Eigen::Vector2d> boundary_points;

    if (open_triangle_count == 0) {
      for(auto& tri : triangle) {
        Eigen::Vector2d cirumcircle_center = CirumCircle(tri);
        points_need_convex_hull.push_back(cirumcircle_center);
      }

    } else if (open_triangle_count == 1 || open_triangle_count == 2) {

      for(auto& tri : triangle) {
        // find the center
        Eigen::Vector2d cirumcircle_center = CirumCircle(tri);

        if (IsOpenTriangle(tri, triangle, point)) {
          // find the openedge
          std::vector<algorithm::Edge> open_edges = GetOpenEdge(tri, triangle, point);

          for(auto& edge : open_edges) {
            Eigen::Vector2d edge_center = (edge.first.coordinate + edge.second.coordinate)  / 2.0;
            Eigen::Vector2d intersect_point = Intersect(cirumcircle_center, edge_center, boundary);

            points_need_convex_hull.push_back(intersect_point);
            // filter the boundary
            boundary_points = BoundaryFilter(cirumcircle_center, intersect_point, point, boundary_points);
          }
        }
        points_need_convex_hull.push_back(cirumcircle_center);

        for(auto b_p : boundary_points) {
          points_need_convex_hull.push_back(b_p);
        }


      }

    } else {
      // error
      return false;
    }

    // TODO: make convex hull for points_need_convex_hull
    struct SimplePoint {
      double x, y;
    };
    std::vector<SimplePoint> temp_point_need_convex_hull;
    for_each(points_need_convex_hull.begin(), points_need_convex_hull.end(), [&temp_point_need_convex_hull](Eigen::Vector2d p) {
      SimplePoint simple_point;
      simple_point.x = p(0, 0);
      simple_point.y = p(1, 0);
      temp_point_need_convex_hull.push_back(simple_point);
    });
    std::vector<SimplePoint> polygon_points;
    ConvexHull(temp_point_need_convex_hull, polygon_points);


    Polygon voronoi_polygon;
    for(auto item : polygon_points) {
      Eigen::Vector2d v_p;
      v_p << item.x, item.y;
      voronoi_polygon.vertexs.push_back(v_p);
    }
    result.push_back(voronoi_polygon);
    // TODO:
    // add to the result

    // finish
  }
  return true;
}
#include "voronoi.hpp"
