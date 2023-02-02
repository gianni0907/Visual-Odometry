#include "points_utils.h"

namespace pr {
    using namespace std;

    void makeWorld(Vec3fVector& world_points,
                   const Eigen::Vector3f& lower_left_bottom,
                   const Eigen::Vector3f& upper_right_top,
                   int num_points){
        Eigen::Vector3f ranges=upper_right_top-lower_left_bottom;
        world_points.resize(num_points);
        for (int i=0; i<num_points; i++){
            world_points[i]=lower_left_bottom + 
                            Eigen::Vector3f(ranges.x()*drand48(),
                                            ranges.y()*drand48(),
                                            ranges.z()*drand48());
        }
    }

    void drawPoints(RGBImage& img,
                    const Vec2fVector& points,
                    const cv::Scalar& color,
                    int radius){
        int rows=img.rows;
        int cols=img.cols;
        for (const Eigen::Vector2f point: points){
            int r=point.y();
            int c=point.x();
            if( r<0 || r>=rows)
                continue;
            if( c<0 || c>=cols)
                continue;
            cv::circle(img, cv::Point(c,r), radius, color);
        }
    }
}