#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Cholesky>
#include <Eigen/StdVector>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include "opencv2/opencv.hpp"
#include <iostream>
#include <unistd.h>

namespace pr {
    typedef std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> Vec3fVector;
    typedef std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>> Vec2fVector;

    typedef Eigen::Matrix<float, 9, 1> Vector9f;
    typedef Eigen::Matrix<float, 3, 2> Matrix3_2f;
    typedef Eigen::Matrix<float, 2, 3> Matrix2_3f;

    typedef cv::Mat_< cv::Vec3b > RGBImage;

    // compute the smallest eigenvector of a 9x9 matrix              
    template <typename SquareMatrixType_>
    inline Eigen::Matrix<typename SquareMatrixType_::Scalar,
                  SquareMatrixType_::RowsAtCompileTime, 1>
    smallestEigenVector(const SquareMatrixType_& m){
        Eigen::SelfAdjointEigenSolver<SquareMatrixType_> es;
        es.compute(m);
        return es.eigenvectors().col(0);
    }

    inline Eigen::Matrix3f skew(const Eigen::Vector3f& v){
        Eigen::Matrix3f S;
        S << 0, -v[2], v[1],
             v[2], 0, -v[0],
             -v[1], v[0], 0;
        return S;
    }

    inline Eigen::Matrix3f Rx(float rot_x){
        float c=cos(rot_x);
        float s=sin(rot_x);
        Eigen::Matrix3f R;
        R << 1,  0, 0,
             0,  c,  -s,
             0,  s,  c;
        return R;
    }
  
    inline Eigen::Matrix3f Ry(float rot_y){
        float c=cos(rot_y);
        float s=sin(rot_y);
        Eigen::Matrix3f R;
        R << c,  0,  s,
             0 , 1,  0,
             -s,  0, c;
        return R;
    }

    inline Eigen::Matrix3f Rz(float rot_z){
        float c=cos(rot_z);
        float s=sin(rot_z);
        Eigen::Matrix3f R;
        R << c,  -s,  0,
             s,  c,  0,
             0,  0,  1;
        return R;
    }
}

