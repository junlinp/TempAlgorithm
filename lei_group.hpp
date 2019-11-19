//
// Created by junlinp on 2019-11-01.
//

#ifndef OPENCV_TEST__LEI_GROUP_HPP_
#define OPENCV_TEST__LEI_GROUP_HPP_
#include <Eigen/Dense>
using namespace Eigen;
Vector3f SkewSymmetricMatrixToLeiVector(Matrix<float, 3, 3> rotateMatrix) {
    Vector3f result;
    result << rotateMatrix(2, 1) , rotateMatrix(0, 2), rotateMatrix(1, 0);
    return result;
}
Matrix3f VectorToSkewSymmetricMatrix(Vector3f vec) {
  Matrix3f result;
  result << 0, -vec[2], vec[1],
            vec[2], 0, -vec[0],
            -vec[1], vec[0], 0;
  return result;
}

Matrix3f leiVectorToRotateMatrix(Vector3f vec) {
  double theta = (vec.transpose() * vec)(0);
  theta = sqrt(theta);
  vec = vec / theta;
  return cos(theta) * Matrix3f::Identity() + (1 - cos(theta)) * (vec * vec.transpose()) + sin(theta) * VectorToSkewSymmetricMatrix(vec);
}

Vector3f rotateMatrixToleiVector(Matrix3f matrix) {
  double theta = acos((matrix(0, 0) + matrix(1, 1) + matrix(2, 2) - 1) / 2);
  std::cout << "theta : " << theta << std::endl;
  EigenSolver<Matrix3f> solver(matrix);
  Vector3f vec;
  for(int i = 0; i < 3; i++) {
    if(solver.eigenvalues()[i].real() - 1.0 < 1e-6) {
      vec = solver.eigenvectors().real().block(0, i, 3, 1);
    }
  }
  vec = vec.normalized();
  std::cout << "vec : " << vec << std::endl;
  return theta * vec;
}

void fitting_target(Vector3f target_pos, Vector3f input_pos) {
  Matrix3f rotate_matrix;
  rotate_matrix << 1.0 , 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, 1.0;

  int epoch = 4;
  for(int i = 0; i < epoch; i++) {
    Vector3f est_pos = rotate_matrix * input_pos;

    Vector3f d_RP = -2 * (target_pos - est_pos);
    Matrix3f d_phi = - VectorToSkewSymmetricMatrix( est_pos );
    Vector3f grad = d_RP.transpose() * d_phi;

    rotate_matrix = leiVectorToRotateMatrix(-grad * 0.3) * rotate_matrix;

    std::cout << "error : " << (target_pos - est_pos).norm() << std::endl;
    std::cout << "target : " << target_pos << "estimate : " << est_pos << std::endl;
  }

  std::cout << "final rotate_matrix : " << rotate_matrix << std::endl;
}

void gaussNewton() {
  cv::RNG rng;
  const int N = 1024;
  std::vector<double> x, y;
  double w_sigma = 1.0;
  double ar = 1.0, br = 2.0, cr = 1.0;
  for(int i = 0; i < N; i++) {
    x.push_back(1.0 * i / N);
    y.push_back(exp(ar * x[i] * x[i] + br * x[i] + cr) + rng.gaussian(w_sigma * w_sigma));

  }

}
void lei_main() {
  Matrix3f rotate;
  double PI = 3.1415926;
  rotate << cos(PI / 4), -sin(PI / 4), 0,
            sin(PI / 4), cos(PI / 4), 0,
            0,0,1;
  std::cout << "rotate matrix : " << rotate << std::endl;
  std::cout << "Vector : " << SkewSymmetricMatrixToLeiVector(rotate) << std::endl;

  Vector3f pos = Vector3f::Random();
  Vector3f lei;
  lei << 2, 2, 3;
  auto m = leiVectorToRotateMatrix(lei);
  std::cout << m << std::endl << m * m.transpose() << std::endl;
  std::cout << rotateMatrixToleiVector(m) << std::endl;

  fitting_target(rotate * pos, pos);
}

#endif //OPENCV_TEST__LEI_GROUP_HPP_
