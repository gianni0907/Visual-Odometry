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
    typedef Eigen::Matrix<float, 2, 1> Vector2f;
    typedef Eigen::Matrix<float, 3, 1> Vector3f;
    typedef Eigen::Matrix<float, 6, 1> Vector6f;
    typedef Eigen::Matrix<float, 9, 1> Vector9f;
    typedef Eigen::Matrix<float, 10, 1> Vector10f;

    typedef Eigen::Matrix<float, 3, 3> Matrix3f;
    typedef Eigen::Matrix<float, 6, 6> Matrix6f;
    typedef Eigen::Matrix<float, 10, 10> Matrix10f;
    typedef Eigen::Matrix<float, 3, 2> Matrix3_2f;
    typedef Eigen::Matrix<float, 2, 3> Matrix2_3f;
    typedef Eigen::Matrix<float, 3, 6> Matrix3_6f;
    typedef Eigen::Matrix<float, 2, 6> Matrix2_6f;

    typedef cv::Mat_< cv::Vec3b > RGBImage;

    typedef std::pair<int,int> IntPair;
    typedef std::vector<IntPair > IntPairVector;

    typedef std::vector<Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f> > TransfVector;
    typedef std::pair<Matrix3f,Matrix3f> Matrix3fPair;
    typedef std::vector<Vector3f, Eigen::aligned_allocator<Vector3f> > Vector3fVector;
    typedef std::vector<Vector10f, Eigen::aligned_allocator<Vector10f> > Vector10fVector;

    typedef struct Point3d{
        int id;
        Vector3f p;
        Vector10f appearance;
        Point3d(){
            id=-1;
            p.setZero();
            appearance.setZero();
        }
        Point3d(int id_,Vector3f p_, Vector10f a_){
            id=id_;
            p=p_;
            appearance=a_;
        }
    } Point3d;

    typedef struct Point2d{
        int id;
        Vector2f p;
        Vector10f appearance;
        Point2d(){
            id=-1;
            p.setZero();
            appearance.setZero();
        }
        Point2d(int id_,Vector2f p_, Vector10f a_){
            id=id_;
            p=p_;
            appearance=a_;
        }
    } Point2d;

    typedef struct Point{
        int id;
        Vector10f appearance;
        Point(){
            id=-1;
            appearance.setZero();
        }
        Point(int id_, Vector10f a_){
            id=id_;
            appearance=a_;
        }
    } Point;

    typedef std::vector<Point> PointsVector;
    typedef std::vector<Point2d> Points2dVector;
    typedef std::vector<Point3d> Points3dVector;
    
    typedef struct{
        Vector3f gt_pose;
        Points2dVector points;
    } Observations;
        
    typedef std::vector<Observations> ObsVector;

    // this invokes the Eigen functions to compute the eigenvalues of
    // a symmetric matrix. We extract the largest EigenVector of the covariance
    // which denotes the direction of highest variation.
    template <typename SquareMatrixType_>
    inline Eigen::Matrix<typename SquareMatrixType_::Scalar,
                         SquareMatrixType_::RowsAtCompileTime, 1>
    largestEigenVector(const SquareMatrixType_& m) {
        Eigen::SelfAdjointEigenSolver<SquareMatrixType_> es;
        es.compute(m);
        return es.eigenvectors().col(SquareMatrixType_::RowsAtCompileTime-1);
    }

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

    inline Eigen::Isometry3f v2t_aug(const Eigen::Vector3f& t){
        Eigen::Isometry3f T;
        T.setIdentity();
        T.translation() << t(0), t(1), 0;
        float c = cos(t(2));
        float s = sin(t(2));
        T.linear() << c, -s, 0,
                      s, c, 0,
                      0, 0, 1;
        return T;
    }

    inline Eigen::Vector3f t2v(const Eigen::Isometry3f& t){
        Eigen::Vector3f v;
        v(0)=t.translation().x();
        v(1)=t.translation().y();
        v(2) = atan2(t.linear()(1,0), t.linear()(0,0));
        return v;
    }

    inline Eigen::Isometry3f v2tEuler(const Vector6f& v){
        Eigen::Isometry3f T;
        T.linear()=Rx(v[3])*Ry(v[4])*Rz(v[5]);
        T.translation()=v.head<3>();
        return T;
    }
}

