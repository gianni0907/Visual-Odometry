#include "camera.h"
#include "points_utils.h"
#include "epipolar.h"
#include "projective_icp.h"

using namespace std;
using namespace vo;

int main (int argc, char** argv) {

  Points3dVector world_points;
  Points2dVector reference_image_points;
  Eigen::Vector3f lower_left_bottom(-10,-10,-10);
  Eigen::Vector3f upper_right_top(10,10,10);
  int num_points=1000;
  int num_ref_img_points=0;
  int num_iterations=20;
  const bool keep_indices=false;

  // generate 3d points
  makeWorld(world_points,
            lower_left_bottom,
            upper_right_top,
            num_points);

  cout << "Generated Model with " << world_points.size() << " points" << endl;

  // set camera parameters
  Eigen::Matrix3f camera_matrix;
  camera_matrix << 180, 0, 320,
                   0, 180,240,
                   0, 0, 1;
  
  int width=640;
  int height=480;
  float z_near=0;
  float z_far=5;

  //Consider a ground truth pose of the world wrt cam
  //this is what we want to estimate via projective_icp
  Eigen::Isometry3f gt_X=Eigen::Isometry3f::Identity();
  gt_X.linear()=Rz(0.6);
  gt_X.translation()=Eigen::Vector3f(-2.0f, 1.7f, 0.0f);
  cout << "Ground truth transformation:" << endl;
  cout << gt_X.linear() << endl << gt_X.translation() << endl;
  cout << endl;

  //instantiate the camera with ground truth pose
  Camera gt_cam(height, width, z_near, z_far, camera_matrix, gt_X);

  //project all points in world on the image plane of the camera
  //these will be the measurement for the picp procedure
  num_ref_img_points=gt_cam.projectPoints(reference_image_points, world_points, keep_indices); 
  cout << "Number of reference points in the image: " << num_ref_img_points << endl;

  // set initial guess for the pose
  Eigen::Isometry3f X=Eigen::Isometry3f::Identity();

  //instantiate the camera with initial guess pose
  Camera cam(height, width, z_near, z_far, camera_matrix, X);

  // construct a solver
  PICPSolver solver;
  solver.init(cam,world_points,reference_image_points,num_iterations,keep_indices);
  solver.setKernelThreshold(10000);
  solver.run();
  cam=solver.camera();
  X=cam.worldInCameraPose();
  cout << "Estimated transformation:" << endl;
  cout << X.linear() << endl << X.translation() << endl;
  cout << endl;
  cout << "Ground truth/estimated transformation ratio:" << endl;
  for (int i=0;i<3;i++){
      for (int j=0;j<4;j++)
          cout << gt_X(i,j)/X(i,j) << " ";
      cout << endl;
  }
  return 0;
}
