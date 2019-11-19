//
// Created by junlinp on 2019-10-30.
//

#ifndef OPENCV_TEST__IMAGEPROCESS_HPP_
#define OPENCV_TEST__IMAGEPROCESS_HPP_
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <cmath>

void blur_process(cv::Mat src) {
  cv::Mat dst = src.clone();

  cv::blur(src, dst, cv::Size(3, 3), cv::Point(-1, -1));
  cv::imshow("blur", dst);
  std::cout << dst << std::endl;
}

void blur_filter(cv::Mat src) {
  cv::Mat dst;

  cv::Mat kernel = cv::Mat::ones(3, 3, CV_32F) / 9.0;
  cv::filter2D(src, dst, src.depth(),kernel);
  std::cout << dst  << std::endl;
}

void gaussian_blur_process(cv::Mat src) {
  cv::Mat dst;

  cv::GaussianBlur(src, dst, cv::Size(3, 3), 1, 1);
  std::cout << dst << std::endl;
}

double Gaussian(double x, double y, double delta_x, double delta_y) {
  double pre_product = 1.0 / (2 * acos(-1.0) / delta_x / delta_y);
  return  pre_product * exp(-(x * x + y * y) / 2.0 / delta_x / delta_y);
}
void gaussian_blur_filter(cv::Mat src) {
  cv::Mat dst;
  cv::Mat kernel = cv::Mat::zeros(cv::Size(3, 3), CV_64F);
  kernel.at<double>(0, 0) = Gaussian(-1, -1, 1.0, 1.0);
  kernel.at<double>(0, 1) = Gaussian(0, -1, 1.0, 1.0);
  kernel.at<double>(0, 2) = Gaussian(1, -1, 1.0, 1.0);
  kernel.at<double>(1, 0) = Gaussian(-1, 0, 1.0, 1.0);
  kernel.at<double>(1, 1) = Gaussian(0, 0, 1.0, 1.0);
  kernel.at<double>(1, 2) = Gaussian(1, 0, 1.0, 1.0);
  kernel.at<double>(2, 0) = Gaussian(-1, 1, 1.0, 1.0);
  kernel.at<double>(2, 1) = Gaussian(0, 1, 1.0, 1.0);
  kernel.at<double>(2, 2) = Gaussian(1, 1, 1.0, 1.0);
  double sum = 0.0;
  for(auto it = kernel.begin<double>(); it != kernel.end<double>(); ++it) {
    sum += *it;
  }
  std::cout << sum << std::endl;
  kernel = kernel / sum;
  std::cout << "kernel : " << kernel << std::endl;
  cv::filter2D(src, dst, src.depth(), kernel);
  std::cout << dst << std::endl;
}

void pyr_compare(cv::Mat src) {
  cv::Mat opencv_dst, my_dst;

  cv::pyrDown(src, opencv_dst, cv::Size(src.rows / 2, src.cols / 2));

  cv::GaussianBlur(src, my_dst, cv::Size(5, 5), 0, 0);
  cv::Mat tmp_dst = cv::Mat(cv::Size(my_dst.rows / 2, my_dst.cols / 2), src.depth());
  for(int i = 0; i < my_dst.rows; i += 2) {
    for(int j = 0; j < my_dst.cols; j+= 2) {
      tmp_dst.at<uchar>(i / 2, j / 2) = my_dst.at<uchar>(i, j);
    }
  }
  std::cout << "pyrDown : " << opencv_dst << std::endl;
  std::cout << "myDown : " << tmp_dst << std::endl;
}

void make_border(cv::Mat src) {
 cv::Mat dst;
 cv::copyMakeBorder(src, dst, 1, 1, 1, 1, cv::BORDER_DEFAULT, 0);
 std::cout << "Border : " << dst << std::endl;

}

void sobel_demo(cv::Mat src) {
  int scale = 1;
  int delta = 0;
  int ddepth = CV_16S;

  char* window_name = "Sobel Demo - Simple Edge Detector";
  cv::Mat blur_dst;
  cv::GaussianBlur(src, blur_dst, cv::Size(3, 3), 0);

  cv::Mat src_gray;
  cv::cvtColor(blur_dst, src_gray, cv::COLOR_RGB2GRAY);
  cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
  cv::Mat grad_x, grad_y;
  cv::Mat abs_grad_x, abs_grad_y;
  cv::Mat grad;
  cv::Sobel(src_gray, grad_x, ddepth, 1, 0, 3, 1, 0);
  cv::convertScaleAbs(grad_x, abs_grad_x);
  cv::Sobel(src_gray, grad_y, ddepth, 0, 1, 3, 1, 0);
  cv::convertScaleAbs(grad_y, abs_grad_y);

  cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);
  imshow(window_name, grad);
  cv::waitKey(0);

}
#endif //OPENCV_TEST__IMAGEPROCESS_HPP_
