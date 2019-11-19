#include <iostream>
#include <strstream>
#include <memory>

#include <opencv2/core.hpp>
#include <Eigen/Dense>

#include "ImageProcess.hpp"
#include "lei_group.hpp"
//#include "dense_mapping.hpp"
#include "dense_mapping_slam.hpp"
#include "camera_calib.hpp"

using namespace Eigen;
uchar table[256];
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
int main(int argc, char **argv) {
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
  /*
  make_border(m);
  cv::Mat image = cv::imread("IMG_20180903_172737.jpg");
  sobel_demo(image);
   */
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

}