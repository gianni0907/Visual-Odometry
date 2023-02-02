#pragma once
#include "defs.h"

namespace pr{
    //given two images, returns on each image only the points present on both images
    //assuming correspondences between points with same index in the two images
    //return number of matching points
    int pruneUnmatchingProjectedPoints(const Vec2fVector& img1_points,
                                       const Vec2fVector& img2_points,
                                       Vec2fVector& img1_matching_points,
                                       Vec2fVector& img2_matching_points);

    //triangulates a point given two lines
    //one passing through the origin, with direction d1
    //one passing through point o2, with direction d2
    bool triangulatePoint(const Eigen::Vector3f& o2,
                          const Eigen::Vector3f& d1,
                          const Eigen::Vector3f& d2,
                          Eigen::Vector3f& point,
                          float& error);
    //triangulates a set of points
    //given the projection of the points in two images
    //relative transformation is the pose of the world wrt 2nd camera
    //return the number of trinagulated points
    int triangulatePoints(const Eigen::Matrix3f& K,
                           const Eigen::Isometry3f& X,
                           const Vec2fVector& img1_points,
                           const Vec2fVector& img2_points,
                           Vec3fVector& points,
                           std::vector<float>& errors);


    void essential2transform(const Eigen::Matrix3f& E,
                             Eigen::Isometry3f& X1,
                             Eigen::Isometry3f& X2);

    const Eigen::Matrix3f transform2essential(const Eigen::Isometry3f& X);
}