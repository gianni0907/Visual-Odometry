#pragma once
#include "defs.h"
#include "correspondence_finder.h"

namespace vo{

    /// triangulate a point given two lines
    /// one passing through the origin, with direction d1
    /// one passing through point o2, with direction d2
    /// @param o2: a 3dpoint along the line with d2 direction 
    /// @param d1: direction of first line
    /// @param d2: direction of second line
    /// @param point: the triangulated 3dpoint in world
    /// @param error: the error expressed as distance between the two closest points on the two lines
    /// @return: true if the triangulation is done, false if the point is behind one of the two cameras
    bool triangulatePoint(const Eigen::Vector3f& o2,
                          const Eigen::Vector3f& d1,
                          const Eigen::Vector3f& d2,
                          Eigen::Vector3f& point,
                          float& error);

    /// @param K: camera matrix
    /// @param X: pose of 1st camera in 2nd camera frame
    /// @param img1_points: projected points on 1st camera image plane
    /// @param img2_points: projected points on 2nd camera image plane
    /// @param points: set of triangulated 3d points
    /// @param errors: array of errors
    /// @return: number of triangulated points
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

    /// merge two sets of 3d points
    /// keeping the most recent for corresponding points
    /// @param X: transformation needed to express points in the same frame of new_points
    /// @param new_points: most recent points
    /// @param points: old points
    /// @return: the vector of 3d merged points
    Points3dVector mergePoints(const Eigen::Isometry3f& X,
                               const Points3dVector& new_points,
                               const Points3dVector& points);

    /// @param E: essential matrix 
    /// @param X1: one possible transformation
    /// @param X2: other possible transformation
    void essential2transform(const Eigen::Matrix3f& E,
                             Eigen::Isometry3f& X1,
                             Eigen::Isometry3f& X2);


    /// @return: the essential matrix 
    const Eigen::Matrix3f transform2essential(const Eigen::Isometry3f& X);

    /// @param K: camera matrix 
    /// @param F: fundamental matrix
    /// @param X1: one possible transformation
    /// @param X2: other possible transformation
    void fundamental2transform(const Eigen::Matrix3f& K,
                               const Eigen::Matrix3f& F,
                               Eigen::Isometry3f& X1,
                               Eigen::Isometry3f& X2);
 
    /// @return: the fundamental matrix 
    const Eigen::Matrix3f transform2fundamental(const Eigen::Matrix3f& K,
                                                const Eigen::Isometry3f& X);

    //for each set of images points, return a 3x3 transformation matrix to normalize coordinates in [-1;1]

    /// compute the transformations to normalize points coordinate within [-1;1]
    /// @param correspondences: between the two sets of points
    /// @param img1_points: 1st set of projected points
    /// @param img2_points: 2nd set of projected points
    /// @return: a pair of 3x3 transformation matrices
    const Matrix3fPair normTransform(const IntPairVector& correspondences,
                                     const Points2dVector& img1_points,
                                     const Points2dVector& img2_points);

    /// estimate the fundamental matrix using 8-points algorithm
    /// @param img1_points: 1st set of projected points
    /// @param img2_points: 2nd set of projected points
    /// @return: estimated fundamental matrix
    const Eigen::Matrix3f estimateFundamental(const Points2dVector& img1_points,
                                              const Points2dVector& img2_points);
    
    /// estimate the pose of 1st camera in 2nd camera frame
    /// @param K: camera matrix
    /// @param img1_points: 1st set of projected points
    /// @param img2_points: 2nd set of projected points
    /// @return: estimated transformation
    const Eigen::Isometry3f estimateTransform(const Eigen::Matrix3f& K,
                                              const Points2dVector& img1_points,
                                              const Points2dVector& img2_points);
}