#pragma once
#include "defs.h"
#include "correspondence_finder.h"

namespace vo{
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
    //return the number of triangulated points
    int triangulatePoints(const Eigen::Matrix3f& K,
                          const Eigen::Isometry3f& X,
                          const Points2dVector& img1_points,
                          const Points2dVector& img2_points,
                          Points3dVector& points,
                          std::vector<float>& errors);

    //merge two sets of 3d points
    //taking the most recent for corresponding points
    //and expressing all them in the ref frame of new_points
    //using the transformation X to express old points in new_points frame
    Points3dVector mergePoints(const Eigen::Isometry3f& X,
                               const Points3dVector& new_points,
                               const Points3dVector& points);

    void essential2transform(const Eigen::Matrix3f& E,
                             Eigen::Isometry3f& X1,
                             Eigen::Isometry3f& X2);

    const Eigen::Matrix3f transform2essential(const Eigen::Isometry3f& X);

    void fundamental2transform(const Eigen::Matrix3f& K,
                               const Eigen::Matrix3f& F,
                               Eigen::Isometry3f& X1,
                               Eigen::Isometry3f& X2);

    const Eigen::Matrix3f transform2fundamental(const Eigen::Matrix3f& K,
                                                const Eigen::Isometry3f& X);

    //for each set of images points, return a 3x3 transformation matrix to normalize coordinates in [-1;1]
    const Matrix3fPair normTransform(const IntPairVector& correspondences,
                                     const Points2dVector& img1_points,
                                     const Points2dVector& img2_points);

    //estimates the fundamental matrix given known correspondences of points
    //using 8-points algorithm
    const Eigen::Matrix3f estimateFundamental(const Points2dVector& img1_points,
                                              const Points2dVector& img2_points);

    //estimates the relative transformation between two cameras given known correspondences of points
    const Eigen::Isometry3f estimateTransform(const Eigen::Matrix3f& K,
                                              const Points2dVector& img1_points,
                                              const Points2dVector& img2_points);
}