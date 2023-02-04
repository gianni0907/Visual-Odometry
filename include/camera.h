#pragma once
#include "defs.h"

namespace pr {
    class Camera{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    
        //ctor
        Camera(int height=100,
               int width=100,
               float z_near=0,
               float z_far=10,
               const Eigen::Matrix3f& camera_matrix=Eigen::Matrix3f::Identity(),
               const Eigen::Isometry3f& world_in_cam_pose=Eigen::Isometry3f::Identity());

        // project a single point on the image plane
        inline bool projectPoint(Eigen::Vector2f& img_point,
                                 const Eigen::Vector3f& world_point){
            Eigen::Vector3f camera_point=_world_in_cam_pose*world_point;
            if (camera_point.z()<_z_near || camera_point.z()>_z_far)
                return false;
        
            Eigen::Vector3f projected_point=_camera_matrix*camera_point;
            img_point=projected_point.head<2>()*(1./projected_point.z());
            if (img_point.x()<0 || img_point.x()>_width-1)
                return false;
            if (img_point.y()<0 || img_point.y()>_height-1)
                return false;
            return true;
        }    

        // project the world points on the image plane
        int projectPoints(Points2dVector& img_points,
                          const Points3dVector& world_points,
                          bool keep_indices=false);

        inline const Eigen::Isometry3f& worldInCameraPose() const {return _world_in_cam_pose;}
        inline void setWorldInCameraPose(const Eigen::Isometry3f& pose)  {_world_in_cam_pose=pose;}
        inline const Eigen::Matrix3f& cameraMatrix() const {return _camera_matrix;}

    protected:
        int _height;
        int _width;
        float _z_near;
        float _z_far;
        Eigen::Isometry3f _world_in_cam_pose;
        Eigen::Matrix3f _camera_matrix;
    };
}