//
// Created by junlinp on 2019-11-03.
//

#ifndef OPENCV_TEST__DENSE_MAPPING_HPP_
#define OPENCV_TEST__DENSE_MAPPING_HPP_

#include <string>
#include <vector>
#include <fstream>

#include <opencv2/core.hpp>
#include <sophus/se3.hpp>

using Sophus::SE3d;
#include <Eigen/Core>
#include <Eigen/Geometry>


#include <assert.h>

using namespace Eigen;

const int boarder = 20;
const int width = 640;
const int height = 480;
const double fx = 481.2f;
const double fy = -480.0f;
const double cx = 319.5f, cy = 239.5f;
const int ncc_window_size = 3;
const int ncc_area = (2 * ncc_window_size + 1) * (2 * ncc_window_size + 1);
const double min_cov = 0.1;
const double max_cov = 10;

bool readDatasetFiles(
    const std::string &path,
    std::vector<std::string> &color_image_files,
    std::vector<SE3d> &poses,
    cv::Mat &ref_depth
) {
  std::ifstream fin(path + "/first_200_frames_traj_over_table_input_sequence.txt");

  if (!fin) {
    std::cout << "Can't open file" << std::endl;
    return false;
  }

  while (!fin.eof()) {
    std::string file_name;
    fin >> file_name;
    color_image_files.push_back(path + "/images/" + file_name);
    double data[7];
    for (double &d : data) fin >> d;
    poses.push_back(
        SE3d(
            Eigen::Quaterniond(data[6], data[3], data[4], data[5]),
            Eigen::Vector3d(data[0], data[1], data[2])
        )
    );
  }

  fin.close();
  fin.open(path + "/depthmaps/scene_000.depth");
  ref_depth = cv::Mat(height, width, CV_64F);
  if (!fin) {
    return false;
  }

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      double v = 0.0;
      fin >> v;
      ref_depth.ptr<double>(y)[x] = v / 100.0;
    }
  }

  return true;
  // tx ty tz qx qy qz qw

}

inline Vector3d px2cam(const Vector2d px) {
  return Vector3d(
      (px(0, 0) - cx) / fx,
      (px(1, 0) - cy) / fy,
      1
  );
}

// 相机坐标系到像素
inline Vector2d cam2px(const Vector3d p_cam) {
  return Vector2d(
      p_cam(0, 0) * fx / p_cam(2, 0) + cx,
      p_cam(1, 0) * fy / p_cam(2, 0) + cy
  );
}
Vector3d pt2cam(Vector2d pt) {

  return Vector3d((pt(0, 0) - cx) / fx, (pt(1, 0) - cy) / fy, 1);
}

Vector2d cam2pt(Vector3d pt) {
  return Vector2d(pt[0] / pt[2] * fx + cx, pt[1] / pt[2] * fy + cy);
}

inline bool inside(Vector2d pt) {
  return pt[0] >= 0 && pt[0] < width && pt[1] >= 0 && pt[1] < height;
}
double getBilinearInterpolatedValue(cv::Mat img, Vector2d pt) {
  uchar *d = &img.data[int(pt(1, 0)) * img.step + int(pt(0, 0))];
  double xx = pt(0, 0) - floor(pt(0, 0));
  double yy = pt(1, 0) - floor(pt(1, 0));
  return ((1 - xx) * (1 - yy) * double(d[0]) +
      xx * (1 - yy) * double(d[1]) +
      (1 - xx) * yy * double(d[img.step]) +
      xx * yy * double(d[img.step + 1])) / 255.0;
}
double NCC2(
    const cv::Mat &ref, const cv::Mat &curr,
    const Vector2d &pt_ref, const Vector2d &pt_curr) {
  // 零均值-归一化互相关
  // 先算均值
  double mean_ref = 0, mean_curr = 0;
  std::vector<double> values_ref, values_curr; // 参考帧和当前帧的均值
  for (int x = -ncc_window_size; x <= ncc_window_size; x++)
    for (int y = -ncc_window_size; y <= ncc_window_size; y++) {
      double value_ref = double(ref.ptr<uchar>(int(y + pt_ref(1, 0)))[int(x + pt_ref(0, 0))]) / 255.0;
      mean_ref += value_ref;

      double value_curr = getBilinearInterpolatedValue(curr, pt_curr + Vector2d(x, y));
      mean_curr += value_curr;

      values_ref.push_back(value_ref);
      values_curr.push_back(value_curr);
    }

  mean_ref /= ncc_area;
  mean_curr /= ncc_area;

  // 计算 Zero mean NCC
  double numerator = 0, demoniator1 = 0, demoniator2 = 0;
  for (int i = 0; i < values_ref.size(); i++) {
    double n = (values_ref[i] - mean_ref) * (values_curr[i] - mean_curr);
    numerator += n;
    demoniator1 += (values_ref[i] - mean_ref) * (values_ref[i] - mean_ref);
    demoniator2 += (values_curr[i] - mean_curr) * (values_curr[i] - mean_curr);
  }
  return numerator / sqrt(demoniator1 * demoniator2 + 1e-10);   // 防止分母出现零
}

double NCC(const cv::Mat &ref,const cv::Mat &curr,const Vector2d &ref_pt,const Vector2d &curr_pt) {
  double mean_ref = 0.0, mean_curr = 0.0;
  std::vector<double> ref_values, curr_values;
  for (int y = -ncc_window_size; y <= ncc_window_size; y++) {
    for (int x = -ncc_window_size; x <= ncc_window_size; x++) {
      double ref_value = ref.ptr<double>(int(ref_pt[1]) + y)[int(ref_pt[0]) + x] / 255.0;
      mean_ref += ref_value;
      double curr_value = getBilinearInterpolatedValue(curr, curr_pt + Vector2d(x, y));
      mean_curr += curr_value;

      ref_values.push_back(ref_value);
      curr_values.push_back(curr_value);
    }
  }
  mean_ref /= ncc_area;
  mean_curr /= ncc_area;

  double numerator = 0, demoniator1 = 0, demoniator2 = 0;
  for (int i = 0; i < ref_values.size(); i++) {
    numerator += (ref_values[i] - mean_ref) * (curr_values[i] - mean_curr);
    demoniator1 += (ref_values[i] - mean_ref) * (ref_values[i] - mean_ref);
    demoniator2 += (curr_values[i] - mean_curr) * (curr_values[i] - mean_curr);
  }

  return numerator / (demoniator1 * demoniator2 + 1e-10);
}
bool epipolar_search(cv::Mat &ref,
                     cv::Mat &curr,
                     SE3d T_C_R,
                     Vector2d ref_pt,
                     double ref_depth,
                     double ref_depth_cov,
                     Vector2d &curr_pt,
                     Vector2d &epipolar_direction) {
  Vector3d p_ref = pt2cam(ref_pt);
  p_ref.normalize();
  Vector3d P_ref = ref_depth * p_ref;

  Vector2d px_mean_curr = cam2pt(T_C_R * P_ref);
  double d_min = ref_depth - 3 * ref_depth_cov, d_max = ref_depth + 3 * ref_depth_cov;
  if (d_min < 0.1) d_min = 0.1;
  Vector2d px_min_curr = cam2pt(T_C_R * (d_min * p_ref));
  Vector2d px_max_curr = cam2pt(T_C_R * (d_max * p_ref));

  epipolar_direction = px_max_curr - px_min_curr;
  double half_length = epipolar_direction.norm() * 0.5;
  epipolar_direction.normalize();

  Vector2d best_pt;
  double best_ncc = 0.0;
  for (double l = -half_length; l < half_length; l += 0.7) {
    Vector2d px_curr = px_mean_curr + l * epipolar_direction;

    if (!inside(px_curr)) {
      continue;
    }

    double ncc = NCC(ref, curr, ref_pt, px_curr);
    if (ncc > best_ncc) {
      best_ncc = ncc;
      best_pt = px_curr;
    }
  }

  if (best_ncc > 0.85) {
    curr_pt = best_pt;
    return true;
  }
  return false;
}
bool updateDepthFilter2(
    const Vector2d &pt_ref,
    const Vector2d &pt_curr,
    const SE3d &T_C_R,
    const Vector2d &epipolar_direction,
    cv::Mat &depth,
    cv::Mat &depth_cov2) {
  // 不知道这段还有没有人看
  // 用三角化计算深度
  SE3d T_R_C = T_C_R.inverse();
  Vector3d f_ref = px2cam(pt_ref);
  f_ref.normalize();
  Vector3d f_curr = px2cam(pt_curr);
  f_curr.normalize();

  // 方程
  // d_ref * f_ref = d_cur * ( R_RC * f_cur ) + t_RC
  // f2 = R_RC * f_cur
  // 转化成下面这个矩阵方程组
  // => [ f_ref^T f_ref, -f_ref^T f2 ] [d_ref]   [f_ref^T t]
  //    [ f_cur^T f_ref, -f2^T f2    ] [d_cur] = [f2^T t   ]
  Vector3d t = T_R_C.translation();
  Vector3d f2 = T_R_C.so3() * f_curr;
  Vector2d b = Vector2d(t.dot(f_ref), t.dot(f2));
  Matrix2d A;
  A(0, 0) = f_ref.dot(f_ref);
  A(0, 1) = -f_ref.dot(f2);
  A(1, 0) = -A(0, 1);
  A(1, 1) = -f2.dot(f2);
  Vector2d ans = A.inverse() * b;
  Vector3d xm = ans[0] * f_ref;           // ref 侧的结果
  Vector3d xn = t + ans[1] * f2;          // cur 结果
  Vector3d p_esti = (xm + xn) / 2.0;      // P的位置，取两者的平均
  double depth_estimation = p_esti.norm();   // 深度值

  // 计算不确定性（以一个像素为误差）
  Vector3d p = f_ref * depth_estimation;
  Vector3d a = p - t;
  double t_norm = t.norm();
  double a_norm = a.norm();
  double alpha = acos(f_ref.dot(t) / t_norm);
  double beta = acos(-a.dot(t) / (a_norm * t_norm));
  Vector3d f_curr_prime = px2cam(pt_curr + epipolar_direction);
  f_curr_prime.normalize();
  double beta_prime = acos(f_curr_prime.dot(-t) / t_norm);
  double gamma = M_PI - alpha - beta_prime;
  double p_prime = t_norm * sin(beta_prime) / sin(gamma);
  double d_cov = p_prime - depth_estimation;
  double d_cov2 = d_cov * d_cov;

  // 高斯融合
  double mu = depth.ptr<double>(int(pt_ref(1, 0)))[int(pt_ref(0, 0))];
  double sigma2 = depth_cov2.ptr<double>(int(pt_ref(1, 0)))[int(pt_ref(0, 0))];

  double mu_fuse = (d_cov2 * mu + sigma2 * depth_estimation) / (sigma2 + d_cov2);
  double sigma_fuse2 = (sigma2 * d_cov2) / (sigma2 + d_cov2);

  depth.ptr<double>(int(pt_ref(1, 0)))[int(pt_ref(0, 0))] = mu_fuse;
  depth_cov2.ptr<double>(int(pt_ref(1, 0)))[int(pt_ref(0, 0))] = sigma_fuse2;

  return true;
}
// BUG: sometimes occur nan value
bool updateDepthFilter(Vector2d ref_pt,
                       Vector2d curr_pt,
                       SE3d T_C_R,
                       Vector2d epipolar_direction,
                       cv::Mat depth,
                       cv::Mat depth_cov2) {
  SE3d T_R_C = T_C_R.inverse();
  Vector3d f_ref = pt2cam(ref_pt);
  f_ref.normalize();
  Vector3d f_curr = pt2cam(curr_pt);
  f_curr.normalize();

  // 方程
  // d_ref * f_ref = d_cur * ( R_RC * f_cur ) + t_RC
  // f2 = R_RC * f_cur
  // 转化成下面这个矩阵方程组
  // => [ f_ref^T f_ref, -f_ref^T f2 ] [d_ref]   [f_ref^T t]
  //    [ f_cur^T f_ref, -f2^T f2    ] [d_cur] = [f2^T t   ]
  Vector3d t = T_R_C.translation();
  Vector3d f2 = T_R_C.so3() * f_curr;
  Vector2d b = Vector2d(t.dot(f_ref), t.dot(f2));
  Matrix2d A;
  A(0, 0) = f_ref.dot(f_ref);
  A(0, 1) = -f_ref.dot(f2);
  A(1, 0) = -A(0, 1);
  A(1, 1) = -f2.dot(f2);
  Vector2d ans = A.inverse() * b;
  Vector3d xm = ans[0] * f_ref;           // ref 侧的结果
  Vector3d xn = t + ans[1] * f2;          // cur 结果
  Vector3d p_esti = (xm + xn) / 2.0;      // P的位置，取两者的平均
  double depth_estimation = p_esti.norm();   // 深度值

  // myself
  /*
  SE3d T_R_C = T_C_R.inverse();
  Vector3d f_ref = pt2cam(ref_pt);
  Vector3d f_curr = pt2cam(curr_pt);
  Vector3d t = T_C_R.translation();
  Vector3d f_R_curr = T_C_R.so3() * f_curr - t;

  double alpha = acos(f_ref.dot(t) / t.norm());
  double beta = acos(f_R_curr.dot(-t) / t.norm());
  double gamm = 3.1415926 - alpha - beta;
  Matrix2d A;
  A << sin(gamm), 0,
      0, sin(gamm);
  Vector2d b;
  b << t.norm() * sin(alpha), t.norm() * sin(beta);
  Vector2d ans = A.inverse() * b;
  Vector3d pm = f_ref * ans[0];
  Vector3d pn = f_R_curr * ans[1];
  //std::cout << (pm - pn) << std::endl;
  Vector3d p_esti = (pm + pn) / 2.0;

  double depth_estimation = p_esti.norm();
   */
  // 接下来要算标准差
  Vector3d p = f_ref * depth_estimation;
  Vector3d a = p - t;
  double t_norm = t.norm();
  double a_norm = a.norm();
  double alpha = acos(f_ref.dot(t) / t_norm);
  double beta = acos(a.dot(-t) / (a_norm * t_norm));
  Vector3d f_curr_prime = T_C_R.so3() *  pt2cam(curr_pt + epipolar_direction);
  f_curr_prime.normalize();
  double beta_prime = acos(f_curr_prime.dot(-t) / t_norm);
  double gamma = M_PI - alpha - beta_prime;
  double p_prime = t_norm * sin(beta_prime) / sin(gamma);
  double d_cov = p_prime - depth_estimation;
  double d_cov2 = d_cov * d_cov;

  // 高斯融合
  double mu = depth.ptr<double>(int(ref_pt(1, 0)))[int(ref_pt(0, 0))];
  double sigma2 = depth_cov2.ptr<double>(int(ref_pt(1, 0)))[int(ref_pt(0, 0))];

  double mu_fuse = (d_cov2 * mu + sigma2 * depth_estimation) / (sigma2 + d_cov2);
  double sigma_fuse2 = (sigma2 * d_cov2) / (sigma2 + d_cov2);

  //std::cout << mu_fuse << " : " << sigma_fuse2 << std::endl;
  if (isnan(mu_fuse) || isnan(sigma_fuse2)) {
    std::cout << "ref_pt : " << ref_pt << std::endl;
    std::cout << "curr_pt : " << curr_pt << std::endl;
    std::cout << "T_C_R : " << T_C_R.matrix3x4() << std::endl;
    std::cout << "epipolar_direction : " << epipolar_direction << std::endl;
    std::cout << "d_cov : " << d_cov2 << std::endl;
    std::cout << "mu : " << mu << std::endl;
    std::cout << "sigma2 : " << sigma2 << std::endl;
    std::cout << "depth_estimation : " << depth_estimation << std::endl;
  } else {
    depth.ptr<double>(int(ref_pt(1, 0)))[int(ref_pt(0, 0))] = mu_fuse;
    depth_cov2.ptr<double>(int(ref_pt(1, 0)))[int(ref_pt(0, 0))] = sigma_fuse2;
  }

  return true;

}

void evaludateDepth(const cv::Mat &depth_truth, const cv::Mat &depth_estimate) {
  double ave_depth_error = 0;     // 平均误差
  double ave_depth_error_sq = 0;      // 平方误差
  int cnt_depth_data = 0;
  for (int y = boarder; y < depth_truth.rows - boarder; y++)
    for (int x = boarder; x < depth_truth.cols - boarder; x++) {
      double error = depth_truth.ptr<double>(y)[x] - depth_estimate.ptr<double>(y)[x];
      ave_depth_error += error;
      ave_depth_error_sq += error * error;
      cnt_depth_data++;
    }
  ave_depth_error /= cnt_depth_data;
  ave_depth_error_sq /= cnt_depth_data;

  std::cout << "Average squared error = " << ave_depth_error_sq << ", average error: " << ave_depth_error << std::endl;
}


// 极线搜索
// 方法见书 12.2 12.3 两节
bool epipolarSearch(
    const cv::Mat &ref, const cv::Mat &curr,
    const SE3d &T_C_R, const Vector2d &pt_ref,
    const double &depth_mu, const double &depth_cov,
    Vector2d &pt_curr, Vector2d &epipolar_direction) {
  Vector3d f_ref = px2cam(pt_ref);
  f_ref.normalize();
  Vector3d P_ref = f_ref * depth_mu;    // 参考帧的 P 向量

  Vector2d px_mean_curr = cam2px(T_C_R * P_ref); // 按深度均值投影的像素
  double d_min = depth_mu - 3 * depth_cov, d_max = depth_mu + 3 * depth_cov; // 高斯分布， 大多数在三个标准差之内
  if (d_min < 0.1) d_min = 0.1;
  Vector2d px_min_curr = cam2px(T_C_R * (f_ref * d_min));    // 按最小深度投影的像素
  Vector2d px_max_curr = cam2px(T_C_R * (f_ref * d_max));    // 按最大深度投影的像素

  Vector2d epipolar_line = px_max_curr - px_min_curr;    // 极线（线段形式）
  epipolar_direction = epipolar_line;        // 极线方向
  epipolar_direction.normalize();
  double half_length = 0.5 * epipolar_line.norm();    // 极线线段的半长度
  if (half_length > 100) half_length = 100;   // 我们不希望搜索太多东西

  // 取消此句注释以显示极线（线段）
  // showEpipolarLine( ref, curr, pt_ref, px_min_curr, px_max_curr );

  // 在极线上搜索，以深度均值点为中心，左右各取半长度
  double best_ncc = -1.0;
  Vector2d best_px_curr;
  for (double l = -half_length; l <= half_length; l += 0.7) { // l+=sqrt(2)
    Vector2d px_curr = px_mean_curr + l * epipolar_direction;  // 待匹配点
    if (!inside(px_curr))
      continue;
    // 计算待匹配点与参考帧的 NCC
    double ncc = NCC(ref, curr, pt_ref, px_curr);
    if (ncc > best_ncc) {
      best_ncc = ncc;
      best_px_curr = px_curr;
    }
  }
  if (best_ncc < 0.85f)      // 只相信 NCC 很高的匹配
    return false;
  pt_curr = best_px_curr;
  return true;
}

void update(cv::Mat ref, cv::Mat curr, SE3d T_C_R, cv::Mat &depth, cv::Mat &depth_cov2) {
  // 对每一个像素进行搜索
  for (int x = boarder; x < width - boarder; x++) {
    for (int y = boarder; y < height - boarder; y++) {
      // 1 检查方差时候合适
      double conv = depth_cov2.ptr<double>(y)[x];
      if (conv < min_cov || conv > max_cov) continue;

      // 2 进行极线搜索， 找到极线在curr上的方向和找到的像素坐标
      Vector2d curr_pt;
      Vector2d epipolar_direction;
      Vector2d curr_pt_target;
      Vector2d epipolar_direction_target;
      bool ret = epipolar_search(
          ref,
          curr,
          T_C_R,
          Vector2d(x, y),
          depth.ptr<double>(y)[x],
          sqrt(depth_cov2.ptr<double>(y)[x]),
          curr_pt,
          epipolar_direction
      );
      bool ret2 = epipolarSearch(
          ref,
          curr,
          T_C_R,
          Vector2d(x, y),
          depth.ptr<double>(y)[x],
              sqrt(depth_cov2.ptr<double>(y)[x]),
              curr_pt_target,
              epipolar_direction_target
          );

      assert(ret == ret2);
      assert((curr_pt - curr_pt_target).norm() < 1e-6);
      assert((epipolar_direction - epipolar_direction_target).norm() < 1e-6);
      if (ret) {

        // 3 如果找到， 则进行深度图的融合
        updateDepthFilter2(Vector2d(x, y), curr_pt, T_C_R, epipolar_direction, depth, depth_cov2);
      }

    }
  }
}
void plotDepth(const cv::Mat &depth_truth, const cv::Mat &depth_estimate) {
  imshow("depth_truth", depth_truth * 0.4);
  imshow("depth_estimate", depth_estimate * 0.4);
  imshow("depth_error", depth_truth - depth_estimate);
  cv::waitKey(1);
}


void unittest() {
  cv::RNG rng;

  for(int i = 0; i < 1024; i++) {
    Vector2d pt(rand() % width, rand() % height);
    Vector3d myself = pt2cam(pt);
    Vector3d other = px2cam(pt);

    assert(myself(0, 0) - other(0, 0) < 1e-6);
    assert(myself(1, 0) - other(1, 0) < 1e-6);
    Vector2d pt_myself = cam2pt(myself);
    Vector2d pt_other = cam2px(other);
    assert(pt_myself(0, 0) - pt_other(0, 0) < 1e-6);
    assert(pt_myself(1, 0) - pt_other(1, 0) < 1e-6);
  }
}
void dense_mapping_main() {
  unittest();
  std::string directory_path = "./test_data";
  std::vector<std::string> images_path;
  std::vector<Sophus::SE3d> poses;
  cv::Mat ref_depth;

  readDatasetFiles(directory_path, images_path, poses, ref_depth);
  std::cout << "read : " << images_path.size() << std::endl;

  cv::Mat ref = cv::imread(images_path[0], 0); // gray
  SE3d ref_TWC = poses[0];
  const double init_depth = 3.0;
  const double init_cov = 3.0;
  cv::Mat depth = cv::Mat(height, width, CV_64F, init_depth);
  cv::Mat depth_cov = cv::Mat(height, width, CV_64F, init_cov);

  for (int index = 1; index < images_path.size(); index++) {
    std::cout << "***loop " << index << "***" << std::endl;
    cv::Mat curr = cv::imread(images_path[index], 0);
    SE3d curr_T_W_C = poses[index];
    SE3d curr_T_C_R = curr_T_W_C.inverse() * ref_TWC;
    update(ref, curr, curr_T_C_R, depth, depth_cov);
    evaludateDepth(ref_depth, depth);
    plotDepth(ref_depth, depth);
  }
}
#endif //OPENCV_TEST__DENSE_MAPPING_HPP_
