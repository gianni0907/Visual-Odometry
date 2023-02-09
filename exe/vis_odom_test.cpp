#include "points_utils.h"
#include "camera.h"
#include "epipolar.h"
#include "projective_icp.h"
#include "read_data.h"

using namespace std;
using namespace pr;

int main (int argc, char** argv){
    //Initialization from files
    Points3dVector gt_world=getWorld();
    Camera cam=getCamera();
    Eigen::Matrix3f K=cam.cameraMatrix();
    Eigen::Isometry3f cam_in_rob=cam.camInRobPose();
    ObsVector measurements=getObservations();
    Vector3fVector trajectory=getGroundTruthTrajectory();
    
    //Instantiate useful variables
    TransfVector est_transf(N_POSES-1);
    int num_iterations=20;
    const bool keep_indices=false;
    Points3dVector curr_3dpoints;
    vector<float> errors;
    int num_triangulated_points=0;
    for (size_t i=0;i<est_transf.size();i++)
        est_transf[i].setIdentity();
    
    //Instantiate file handlers to write ground truth and estimated robot trajectory in files
    ofstream out_gt,out_est;
    char* path=getenv("HOME");
    out_gt.open(string(path)+"/Desktop/probrob_proj/to_plot/gt_trajectory.dat");
    out_est.open(string(path)+"/Desktop/probrob_proj/to_plot/est_trajectory.dat");

    //estimate the transformation (cam0 in cam1 frame)
    //considering first pair of measurements
    est_transf[0]=estimateTransform(K,
                                    measurements[0].points,
                                    measurements[1].points);
    cout << "Estimated transformation cam0 in cam1:" << endl;
    cout << est_transf[0].linear() << endl << est_transf[0].translation() << endl;
    cout << "#######################################" << endl;
    
    //Extract the gt robot in world pose
    Eigen::Isometry3f gt_rob_pose=v2t_aug(measurements[1].gt_pose);
    out_gt << gt_rob_pose.translation().transpose() << endl;
    //Find the estimated robot in world pose
    Eigen::Isometry3f est_rob_pose;
    est_rob_pose=cam_in_rob*est_transf[0].inverse()*cam_in_rob.inverse();
    out_est << est_rob_pose.translation().transpose() << endl;
    // cout << "Ground truth transformation rob in world:" << endl;
    // cout << gt_rob_pose.linear() << endl << gt_rob_pose.translation() << endl;
    // cout << endl;
    // cout << "Estimated transformation rob in world:" << endl;
    // cout << est_rob_pose.linear() << endl << est_rob_pose.translation() << endl;
    // cout << endl;
    // cout << "Ground truth/estimated transformation ratio:" << endl;
    for (int i=0;i<3;i++){
        for (int j=0;j<4;j++)
            cout << gt_rob_pose(i,j)/est_rob_pose(i,j) << " ";
        cout << endl;
    }

    //triangulate points, expressing them in the cam1 frame
    num_triangulated_points=triangulatePoints(K,
                                              est_transf[0].inverse(),
                                              measurements[1].points,
                                              measurements[0].points,
                                              curr_3dpoints,
                                              errors);

    PICPSolver solver;
    //set initial guess used for the picp procedure
    Eigen::Isometry3f init_X=Eigen::Isometry3f::Identity();

    //at i-th iteration of picp we estimate cam(i) pose in cam(i+1)
    for (int i=1; i<N_POSES-1;i++){
        cam.setWorldInCameraPose(init_X);
        solver.init(cam,
                    curr_3dpoints,
                    measurements[i+1].points,
                    num_iterations,
                    keep_indices);
        solver.setKernelThreshold(10000);
        solver.run();
        est_transf[i]=solver.camera().worldInCameraPose();
        cout << "Estimated transformation cam" << i << " in cam" << i+1 << " : " << endl;
        cout << est_transf[i].linear() << endl << est_transf[i].translation() << endl;
        cout << "#######################################" <<endl;
        //Extract the gt robot in world pose
        gt_rob_pose=v2t_aug(measurements[i+1].gt_pose);
        out_gt << gt_rob_pose.translation().transpose() << endl;
        //Find the estimated robot in world pose
        est_rob_pose=est_rob_pose*cam_in_rob*est_transf[i].inverse()*cam_in_rob.inverse();
        out_est << est_rob_pose.translation().transpose() << endl;
        // cout << "Ground truth transformation rob in world:" << endl;
        // cout << gt_rob_pose.linear() << endl << gt_rob_pose.translation() << endl;
        // cout << endl;
        // cout << "Estimated transformation rob in world:" << endl;
        // cout << est_rob_pose.linear() << endl << est_rob_pose.translation() << endl;
        // cout << endl;
        // cout << "Ground truth/estimated transformation ratio:" << endl;
        // for (int i=0;i<3;i++){
        //     for (int j=0;j<4;j++)
        //         cout << gt_rob_pose(i,j)/est_rob_pose(i,j) << " ";
        //     cout << endl;
        // }
        Points3dVector new_triang;
        num_triangulated_points=triangulatePoints(K,
                                                  est_transf[i].inverse(),
                                                  measurements[i+1].points,
                                                  measurements[i].points,
                                                  new_triang,
                                                  errors);
        curr_3dpoints=new_triang;
        //curr_3dpoints=mergePoints(est_transf[i],new_triang,curr_3dpoints); //uncomment this if you want to merge points
    }
    out_gt.close();
    out_est.close();
}