#pragma once
#include "defs.h"

namespace pr {
    // create a random world of 3d points
    void makeWorld(Vec3fVector& world_points,
                   const Eigen::Vector3f& lower_left_bottom,
                   const Eigen::Vector3f& upper_right_top,
                   int num_points);

    // draw a set of 2d points on an image
    void drawPoints(RGBImage& img,
                    const Vec2fVector& points,
                    const cv::Scalar& color,
                    int radius);
    
    // draw a set of correspondences between 2d points
    void drawCorrespondences(RGBImage& img,
	                         const Vec2fVector& reference_image_points,
			                 const Vec2fVector& current_image_points,
			                 const IntPairVector& correspondences,
			                 cv::Scalar color=cv::Scalar(0,255,0));
}
