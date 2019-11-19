//
// Created by junlinp on 2019-11-04.
//

#ifndef OPENCV_TEST__CAMERA_CALIB_HPP_
#define OPENCV_TEST__CAMERA_CALIB_HPP_
#include <vector>
#include <cstring>
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>

bool calibration(std::vector<std::string> corner_paths, cv::Mat K) {
  cv::Size pattern(3,3);
  for(std::string path : corner_paths) {
    cv::Mat src = cv::imread(path, cv::IMREAD_GRAYSCALE);
    //cv::imshow("src", src);
    //waitKey(0);
    std::vector<cv::Point2f> corners;
    bool ret = cv::findChessboardCorners(src, pattern, corners);
    if (ret) {
      std::cout << " find " << corners.size() << "corners " << std::endl;
    } else {
      std::cout << " find corner failure" << std::endl;
    }
  }
}
#endif //OPENCV_TEST__CAMERA_CALIB_HPP_
