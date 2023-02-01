#include "camera.h"

namespace pr {
    Camera::Camera(int height,
                   int width,
                   float z_near,
                   float z_far,
                   const Eigen::Matrix3f& camera_matrix,
                   const Eigen::Isometry3f& world_in_cam_pose):
        _height(height),
        _width(width),
        _z_near(z_near),
        _z_far(z_far),
        _camera_matrix(camera_matrix),
        _world_in_cam_pose(world_in_cam_pose){}

    int Camera::projectPoints(Vec2fVector& img_points,
                              const Vec3fVector& world_points,
                              bool keep_indices){
        img_points.resize(world_points.size());
        int num_img_points=0;
        const Eigen::Vector2f point_outside(-1,-1);
        int num_points_inside=0;
        for(size_t i=0; i<world_points.size(); i++){
            const Eigen::Vector3f world_point=world_points[i];
            Eigen::Vector2f& img_point=img_points[num_img_points];
            bool is_inside=projectPoint(img_point,world_point);
            if (is_inside)
                num_points_inside++;
            else
                img_point=point_outside;
            if (keep_indices || is_inside)
                num_img_points++;            
        }
        img_points.resize(num_img_points);
        return num_points_inside;
    }
}