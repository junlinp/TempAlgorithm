#include <iostream>
#include <strstream>
#include <memory>

//#include <opencv2/core.hpp>
//#include <opencv2/highgui.hpp>
#include <random>
#include "kdtree.hpp"
#include <chrono>
#include "immintrin.h"
#include "IO/las.hpp"
#include <fstream>
//#include <Eigen/Dense>

//#include "dense_mapping.hpp"
#include "dense_mapping_slam.hpp"

int main(int argc, char** argv) {
  slam_main(argc, argv);
  return 0;
}