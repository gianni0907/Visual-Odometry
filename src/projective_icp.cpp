#include "projective_icp.h"

namespace pr {
  
  PICPSolver::PICPSolver(){
    _world_points=0;
    _reference_image_points=0;
    _damping=1;
    _min_num_inliers=0;
    _num_inliers=0;
    _kernel_thereshold=1000; // 33 pixels
  }

  void PICPSolver::init(const Camera& camera_,
                        const Points3dVector& world_points,
                        const Points2dVector& reference_image_points,
                        const int& num_iterations,
                        const bool& keep_indices){
    _camera=camera_;
    _world_points=&world_points;
    _reference_image_points=&reference_image_points;
    _num_iterations=num_iterations;
    _keep_indices=keep_indices;
  }

  bool PICPSolver::errorAndJacobian(Eigen::Vector2f& error,
                                    Matrix2_6f& jacobian,
                                    const Eigen::Vector3f& world_point,
                                    const Eigen::Vector2f& reference_image_point){
    // compute the prediction
    Eigen::Vector2f predicted_image_point;
    bool is_good=_camera.projectPoint(predicted_image_point, world_point);
    if (! is_good)
      return false;
    error=predicted_image_point-reference_image_point;
    
    // compute the jacobian of the transformation
    Eigen::Vector3f camera_point=_camera.worldInCameraPose()*world_point;
    Matrix3_6f Jr=Eigen::Matrix<float, 3,6>::Zero();
    Jr.block<3,3>(0,0).setIdentity();
    Jr.block<3,3>(0,3)=skew(-camera_point);

    Eigen::Vector3f phom=_camera.cameraMatrix()*camera_point;
    float iz=1./phom.z();
    float iz2=iz*iz;
    // jacobian of the projection
    Matrix2_3f Jp;
    Jp << 
      iz, 0, -phom.x()*iz2,
      0, iz, -phom.y()*iz2;
      
    jacobian=Jp*_camera.cameraMatrix()*Jr;
    return true;
  }

  void PICPSolver::linearize(const IntPairVector& correspondences, bool keep_outliers){
    _H.setZero();
    _b.setZero();
    _num_inliers=0;
    _chi_inliers=0;
    _chi_outliers=0;
    for (const IntPair& correspondence: correspondences){
      Eigen::Vector2f e;
      Matrix2_6f J;
      int ref_idx=correspondence.second;
      int curr_idx=correspondence.first;
      bool inside=errorAndJacobian(e,
                                   J,
                                   (*_world_points)[curr_idx].p,
                                   (*_reference_image_points)[ref_idx].p);
      if (! inside)
        continue;

      float chi=e.dot(e);
      float lambda=1;
      bool is_inlier=true;
      if (chi>_kernel_thereshold){
        lambda=sqrt(_kernel_thereshold/chi);
        is_inlier=false;
        _chi_outliers+=chi;
      } else {
        _chi_inliers+=chi;
        _num_inliers++;
      }
      
      if (is_inlier || keep_outliers){
        _H+=J.transpose()*J*lambda;
        _b+=J.transpose()*e*lambda;
      }
    }
  }

  bool PICPSolver::oneRound(const IntPairVector& correspondences, bool keep_outliers){
    using namespace std;
    linearize(correspondences, keep_outliers);
    _H+=Matrix6f::Identity()*_damping;
    if(_num_inliers<_min_num_inliers) {
      cerr << "too few inliers, skipping" << endl;
      return false;
    }
    //compute a solution
    Vector6f dx = _H.ldlt().solve(-_b);
    _camera.setWorldInCameraPose(v2tEuler(dx)*_camera.worldInCameraPose());
    return true;
  }

  void PICPSolver::run(){
    Points2dVector current_image_points;
    IntPairVector imgs_correspondences,wrld_correspondences;
    const Eigen::Vector2f dimension=_camera.getDimension();
    char key=0;
    const char ESC_key=27;
    int max_points_leaf=20;
    float radius=0.1;
    CorrespondenceFinder wrld_corr_finder=CorrespondenceFinder<Points3dVector,Points2dVector>();
    wrld_corr_finder.init(*_world_points,max_points_leaf,radius);
    wrld_corr_finder.compute(*_reference_image_points);
    wrld_correspondences=wrld_corr_finder.correspondences();

    //only for visualization on opencv
    CorrespondenceFinder img_corr_finder=CorrespondenceFinder<Points2dVector,Points2dVector>();
    img_corr_finder.init(*_reference_image_points,max_points_leaf,radius);  
    //

    for (int i=0; i<_num_iterations && key!=ESC_key; i++){
      //only for visualization on opencv
      _camera.projectPoints(current_image_points, *_world_points, _keep_indices);
      img_corr_finder.compute(current_image_points);
      imgs_correspondences=img_corr_finder.correspondences();
      RGBImage shown_image(dimension(0),dimension(1));
      shown_image=cv::Vec3b(255,255,255);
      drawPoints(shown_image,*_reference_image_points,cv::Scalar(0,0,255),3);
      drawPoints(shown_image,current_image_points,cv::Scalar(255,0,0),3);
      drawCorrespondences(shown_image,
		                      *_reference_image_points,
		                      current_image_points,
		                      imgs_correspondences,
		                      cv::Scalar(0,255,0));
      cv::imshow("picp_solver_test", shown_image);
      key=cv::waitKey(5);
      //

      oneRound(wrld_correspondences,false);
    }
  }
}
