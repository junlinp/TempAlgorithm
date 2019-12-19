#include <iostream>
#include <strstream>
#include <memory>

//#include <opencv2/core.hpp>
//#include <opencv2/highgui.hpp>
#include <random>
#include "kdtree.hpp"
#include <chrono>
#include "omp.h"
#include "immintrin.h"
#include "IO/las.hpp"
#include <fstream>
//#include <Eigen/Dense>

//#include "ImageProcess.hpp"
//#include "lei_group.hpp"

//#include "omp.h"

//#include "dense_mapping.hpp"
//#include "dense_mapping_slam.hpp"
//#include "camera_calib.hpp"
//#include "Eigen/Dense"
//#include "geometry_algorithm.hpp"
//uchar table[256];
/*
void eigen_test() {
  const int N = 128;
  Matrix<double, N, N> matrix_NN = MatrixXd::Random(N, N);
  matrix_NN = matrix_NN * matrix_NN.transpose();
  Matrix<double, N, 1> v_Nd = MatrixXd::Random(N, 1);

  clock_t time_stt = clock();
  Matrix<double, N, 1> x = matrix_NN.inverse() * v_Nd;
  std::cout << "time of normal inverse is "
            << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << std::endl;
  std::cout << "x = " << x.transpose() << std::endl;

  time_stt = clock();
  x = matrix_NN.colPivHouseholderQr().solve(v_Nd);
  std::cout << "time of Qr is "
            << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << std::endl;

  std::cout << "x = " << x.transpose() << std::endl;
  time_stt = clock();
  x = matrix_NN.ldlt().solve(v_Nd);
  std::cout << "time of ldlt is "
            << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << std::endl;
  std::cout << "x = " << x.transpose() << std::endl;
}
 */
const int thread_nums = 1024 * 1024;
double shared_data[thread_nums] = {0.0};
int main(int argc, char **argv) {
  /*
  std::cout << "hello world" << std::endl;
  for (int i = 0; i < 256; i++) {
    table[i] = i / 20 * 20;
  }
  cv::Mat m = cv::Mat(4, 4, CV_8UC(1));
  randu(m, cv::Scalar::all(0), cv::Scalar::all(255));
  std::cout << m << std::endl;
  int nChannel = m.channels();
  int nRow = m.rows * nChannel;
  int nCol = m.cols;

  cv::Mat LookUpTable(1, 256, CV_8U);
  uchar *p = LookUpTable.data;
  for (int i = 0; i < 256; i++) {
    p[i] = i / 10 * 10;
  }

  //cv::LUT(m, LookUpTable, m);

  //gaussian_blur_process(m);
  //std::cout << m << std::endl;
  //gaussian_blur_filter(m);

  //pyr_compare(m);
  make_border(m);
  cv::Mat image = cv::imread("IMG_20180903_172737.jpg");
  sobel_demo(image);
  lei_main();
  //dense_mapping_main();
  eigen_test();
  //slam_main(0, 0);

  std::vector<std::string> img_paths;
  for (int i = 26; i < 41; i++) {
    std::strstream ss;
    ss << "./imgs/IMG_20191104_2035" << i << ".jpg";
    std::string tmp;
    ss >> tmp;
    img_paths.push_back(tmp);
  }
  cv::Mat K(3, 3, CV_64F);

  //calibration(img_paths, K);

  class A {};
  std::unique_ptr<A> ptr_a(new A);
  std::shared_ptr<A> shared_ptr_a = std::move(ptr_a);
  */
  /*
#pragma omp parallel num_threads(4)
  {

    int id = omp_get_thread_num();
    printf("hello (%d)", id);
    printf("world(%d)\n", id);
  }

  const double step = 1.0 / thread_nums;
  omp_set_num_threads(1024);
  double sum = 0.0;
#pragma omp parallel
  {
#pragma omp for reduction(+ : sum)
  for (int i = 0; i < thread_nums; i++) {
    double x = (i + 0.5) * step;
//#pragma omp atomic
    sum += 4.0 / (1.0 + x * x) * step;
  }
};
*/
//for(int i = 0; i < thread_nums ;i++) {
// sum += shared_data[i] * step;
//}
/*
Eigen::Vector3d a(1, 2, 3);
  Eigen::Vector3d b(4, 5, 6);
  Eigen::Vector3d c(7, 8, 9);
Eigen::Matrix3d matrix;
matrix << a.transpose(), b.transpose(), c.transpose();
std::cout << matrix << std::endl;
Eigen::FullPivLU<Eigen::Matrix3d> lu(matrix);
std::cout << lu.rank() << std::endl;
std::cout << IsLineIntersect(Eigen::Vector3d(0.0, 0.0,0.0),
      Eigen::Vector3d(0.0, 1.0, 0.0),
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(2.0, 0.0, 0.0)
      ) << std::endl;
  std::cout << IsLineIntersect(Eigen::Vector3d(0.0, -1.0,0.0),
                               Eigen::Vector3d(0.0, 1.0, 0.0),
                               Eigen::Vector3d(1.0, 0.0, 0.0),
                               Eigen::Vector3d(-2.0, 0.0, 0.0)
  ) << std::endl;
*/
/*
  double x_lt = -10;
  double y_lt = 10;
  double cellsize = 0.1;
  size_t ROW = 10;
  size_t COL = 1000;

 // struct PointXYZ {
  //  double x, y, z;
  //};
  std::vector<PointXYZ> points;
  std::default_random_engine engine;
  std::uniform_real_distribution<double> dis(-10, 10);
  for(int i = 0; i < 1024 * 1024 * 8; i++) {
    double x = dis(engine);
    double y = dis(engine);
    double z = exp(- x * x - y * y);
    points.push_back({x, y, z});
  }
  RectTree rect_tree;
  rect_tree.SetPoints(points);

  //cv::Mat image(ROW, COL, CV_64F);
  auto start = std::chrono::system_clock::now();
#pragma omp parallel for private(rect_tree)
  for(int row = 0; row < ROW; row++) {
    for(int col = 0; col < COL; col++) {
      double x = x_lt + col * cellsize;
      double y = y_lt - row * cellsize;
      double z = 0.0;
      double weight_sum = 0.0;
      auto r = rect_tree.Search(x, y, cellsize);
      for(auto& point : r) {
        double distance = pow((x - point.x), 2) + pow((y - point.y), 2);
        double weight = 1.0 / (distance + 1e-9);
        z += weight * point.z;
        weight_sum += weight;
      }
      //image.at<double>(row, col) = z / weight_sum;
    }
  }
  auto end = std::chrono::system_clock::now();
  std::chrono::duration<double> du = end - start;
  std::cout << du.count() << " seconds"<< std::endl;
  //cv::imshow("hello world", image);
  //cv::waitKey(0);
  */
  las l;
  std::ofstream fout("test.las");
  l.WriteHeader(fout);
  fout.close();
  std::ifstream fin("test.las");
  l.ReadHeader(fin);
  fin.close();
  return 0;
}