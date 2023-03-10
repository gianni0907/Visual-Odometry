#pragma once
#include "defs.h"

namespace vo {
    /**
        simple pinhole camera class
        Has
        - the position (world with respect to camers)
        - the camera matrix
        - the size of the image plane in pixels
        Supports simple projection operations
    */
    class Camera{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    
        //~ ctor, initialize a camera according to the arguments
        /// here I add also the pose of the camera in robot frame
        Camera(int height=100,
               int width=100,
               float z_near=0,
               float z_far=10,
               const Eigen::Matrix3f& camera_matrix=Eigen::Matrix3f::Identity(),
               const Eigen::Isometry3f& world_in_cam_pose=Eigen::Isometry3f::Identity(),
               const Eigen::Isometry3f& cam_in_rob_pose=Eigen::Isometry3f::Identity());

        //! projects a single point on the image plane
        //! @returns false if the point is outside the camera view frustrum
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

        //! project the world points on the image plane
        /// @param img_points: the points on the image
        /// @param world_points: the input world points
        /// @param keep_indices: if true, img_points has the same size of world points
        /// Invalid points are marked with (-1, -1). If false only the points in the set are returned
        /// @return the number of points that fall inside the image
        int projectPoints(Points2dVector& img_points,
                          const Points3dVector& world_points,
                          bool keep_indices=false);

        inline const Eigen::Isometry3f& worldInCameraPose() const {return _world_in_cam_pose;}
        inline const Eigen::Isometry3f& camInRobPose() const {return _cam_in_rob_pose;}
        inline void setWorldInCameraPose(const Eigen::Isometry3f& pose)  {_world_in_cam_pose=pose;}
        inline const Eigen::Matrix3f& cameraMatrix() const {return _camera_matrix;}
        inline const Eigen::Vector2f getDimension() const {
            Eigen::Vector2f dimension;
            dimension << _height, _width;
            return dimension;
        }

    protected:
        int _height;
        int _width;
        float _z_near;
        float _z_far;
        Eigen::Matrix3f _camera_matrix;
        Eigen::Isometry3f _world_in_cam_pose;
        Eigen::Isometry3f _cam_in_rob_pose;
    };
}