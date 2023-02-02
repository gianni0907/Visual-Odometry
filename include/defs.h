#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Cholesky>
#include <Eigen/StdVector>
#include <Eigen/Dense>
#include "opencv2/opencv.hpp"
#include <iostream>
#include <unistd.h>

namespace pr {
    typedef std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> Vec3fVector;
    typedef std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>> Vec2fVector;

    typedef Eigen::Matrix<float, 3, 2> Matrix3_2f;
    typedef Eigen::Matrix<float, 2, 3> Matrix2_3f;

    typedef cv::Mat_< cv::Vec3b > RGBImage;
}

