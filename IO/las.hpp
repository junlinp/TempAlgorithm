//
// Created by junlinp on 2019-12-19.
//

#ifndef ALGORITHM_IO_LAS_HPP_
#define ALGORITHM_IO_LAS_HPP_
#include <iostream>
struct PointXYZRGB {
  double X, Y, Z;
  unsigned char R, G, B;
};
class las {
 public:
  void WriteHeader(std::ostream& os);
  void ReadHeader(std::istream& is);
  void WritePoint(std::ostream& os, PointXYZRGB point);
};

#endif //ALGORITHM_IO_LAS_HPP_
