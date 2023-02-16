#pragma once
#include "defs.h"

namespace vo {
    /// create a random world of 3d points
    /// @param world_points: the array of Point3d filled with world points 
    /// @param lower_left_bottom: one extrema of the bounding box containing the world
    /// @param upper_right_top: other extrema of bounding box
    /// @param num_points: number of points to create
    void makeWorld(Points3dVector& world_points,
                   const Eigen::Vector3f& lower_left_bottom,
                   const Eigen::Vector3f& upper_right_top,
                   int num_points);

    /// draw a set of 2d points on an image
    /// @param img: the (preallocated) dest image
    /// @param points: the array of points
    /// @param color: the color of the points
    /// @param radius: the size of the point in the image (in pixels)
    void drawPoints(RGBImage& img,
                    const Points2dVector& points,
                    const cv::Scalar& color,
                    int radius);
    
    /// draws a set of correspondences between 2d points
    /// @param img: the dest image (preallocated)
    /// @param reference_image_points: the first vector of points
    /// @param current_image_points: the second vector of points
    /// @param correspondences: the array of correspondences.
    /// if correspondences[k] = (i,j), a line between reference_image_points[i] 
    /// and current_image_points[j] will be drawn
    /// @param color: the color
    void drawCorrespondences(RGBImage& img,
	                         const Points2dVector& reference_image_points,
			                 const Points2dVector& current_image_points,
			                 const IntPairVector& correspondences,
			                 cv::Scalar color=cv::Scalar(0,255,0));
}
