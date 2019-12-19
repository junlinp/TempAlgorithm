//
// Created by junlinp on 2019-12-09.
//

#ifndef ALGORITHM__KDTREE_TEST_HPP_
#define ALGORITHM__KDTREE_TEST_HPP_
#include "kdtree.hpp"
#include "gtest/gtest.h"
#include "random"
struct Point {
  double data[3];
  double& operator[](int idx) {
    return data[idx];
  }
  double operator[](int idx) const {
    return data[idx];
  }
};

std::default_random_engine engine;
std::uniform_real_distribution<double> distribution(-100, 100);

TEST(KDTree, 1K) {
  const size_t N = 1024;
  std::vector<Point> points;
  for(size_t i = 0; i < N; i++) {
    Point p;
    for(int j = 0; j < 3; j++) {
      p[j] = distribution(engine);
    }
    points.push_back(p);
  }
  KDTree<Point, 3> kd_tree(points);
}

TEST(KDTree, 1M) {
  const size_t N = 1024 * 1024;
  std::vector<Point> points;
  for(size_t i = 0; i < N; i++) {
    Point p;
    for(int j = 0; j < 3; j++) {
      p[j] = distribution(engine);
    }
    points.push_back(p);
  }
  KDTree<Point, 3> kd_tree(points);
}


TEST(KDTree, 32M) {
  const size_t N = 1024 * 1024 * 32;
  std::vector<Point> points;
  for(size_t i = 0; i < N; i++) {
    Point p;
    for(int j = 0; j < 3; j++) {
      p[j] = distribution(engine);
    }
    points.push_back(p);
  }
  KDTree<Point, 3> kd_tree(points);
}

#endif //ALGORITHM__KDTREE_TEST_HPP_
