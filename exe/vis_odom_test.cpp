#include "points_utils.h"
#include "camera.h"
#include "epipolar.h"
#include "projective_icp.h"
#include "read_data.h"

using namespace std;
using namespace vo;

int main (int argc, char** argv){
    //Initialization from files
    //Take from keyboard the path to the dataset folder
    std::filesystem::path dataset;
    cout << "Insert the path to dataset folder" << endl;
    cin >> dataset;
    Camera cam=getCamera(dataset);
    Eigen::Matrix3f K=cam.cameraMatrix();
    Eigen::Isometry3f cam_in_rob=cam.camInRobPose();
    ObsVector measurements=getObservations(dataset);
    
    //Instantiate useful variables
    Eigen::Isometry3f est_transf=Eigen::Isometry3f::Identity();
    Eigen::Isometry3f est_rob_pose=Eigen::Isometry3f::Identity();
    int num_iterations=20;
    const bool keep_indices=false;
    Points3dVector curr_3dpoints;
    vector<float> errors;
    int num_triangulated_points=0;
    double t_start_solve=0;
    double t_end_solve=0;
    
    //Instantiate file handlers to write ground truth and estimated robot trajectory in files
    //and also to save ground truth and estimated world (i.e. 3d points)
    ofstream out_est_traj,out_est_wrld,out_est_pose;
    const std::filesystem::path& path=std::filesystem::current_path();
    out_est_traj.open(string(path)+"/../estimation/est_trajectory.dat");
    if(!out_est_traj){
        cerr << "Error: est_trajectory.dat file could not be opened" << endl;
        exit(1);
    }
    out_est_wrld.open(string(path)+"/../estimation/est_world.dat");
    if(!out_est_wrld){
        cerr << "Error: est_world.dat file could not be opened" << endl;
        exit(1);
    }
    out_est_pose.open(string(path)+"/../estimation/est_pose.dat");
    if(!out_est_pose){
        cerr << "Error: est_pose.dat file could not be opened" << endl;
        exit(1);
    }
    //save the first estimated robot pose (identity)
    //each time store: 
    //  -the whole estimated Isometry3f in a file, for evaluation
    //  -the x-y position and theta angle in another file, to ease the trajectory plotting
    out_est_traj << est_rob_pose.translation().transpose() << endl;
    out_est_pose << est_rob_pose.matrix() << endl << endl;

    //estimate the transformation (cam0 in cam1 frame)
    //considering first pair of measurements
    est_transf=estimateTransform(K,
                                 measurements[0].points,
                                 measurements[1].points);
    cout << "Estimated transformation cam0 in cam1:" << endl;
    cout << est_transf.matrix() << endl;
    cout << "#######################################" << endl;
    
    //Find the estimated robot in world pose and store it
    est_rob_pose=cam_in_rob*est_transf.inverse()*cam_in_rob.inverse();
    out_est_traj << est_rob_pose.translation().transpose() << endl;
    out_est_pose << est_rob_pose.matrix() << endl << endl;

    //triangulate points, expressing them in the cam1 frame
    num_triangulated_points=triangulatePoints(K,
                                              est_transf.inverse(),
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
        t_start_solve=getTime();
        solver.init(cam,
                    curr_3dpoints,
                    measurements[i+1].points,
                    num_iterations,
                    keep_indices);
        solver.setKernelThreshold(10000);
        solver.run();
        est_transf=solver.camera().worldInCameraPose();
        t_end_solve=getTime();
        cout << "Estimated transformation cam" << i << " in cam" << i+1 << " : " << endl;
        cout << est_transf.matrix() << endl;
        cout << "Solve tooks: " << (t_end_solve-t_start_solve) << "ms" << endl;
        cout << "Error (inliers): " << solver.chiInliers() << endl;
        cout << "#######################################" <<endl;

        //Find the estimated robot in world pose and save
        est_rob_pose=est_rob_pose*cam_in_rob*est_transf.inverse()*cam_in_rob.inverse();
        out_est_traj << est_rob_pose.translation().transpose() << endl;
        out_est_pose << est_rob_pose.matrix() << endl << endl;

        //triangulate the points on the considered images and with the estimated transformation,
        //obtain a new set of triangulated points expressed in cam i+1
        Points3dVector new_triang;
        num_triangulated_points=triangulatePoints(K,
                                                  est_transf.inverse(),
                                                  measurements[i+1].points,
                                                  measurements[i].points,
                                                  new_triang,
                                                  errors);
        //here choose ONLY one of the following two lines
        //curr_3dpoints=new_triang; //run this line to consider ONLY new triangulated points for next picp iteration
        curr_3dpoints=mergePoints(est_transf,new_triang,curr_3dpoints); //run this if you want to merge new triangulate points with the old ones
    }

    //express estimated world points in world frame and save them in file
    for (auto& point: curr_3dpoints){
        Point3d world_point(point.id,est_rob_pose*cam_in_rob*point.p,point.appearance);
        out_est_wrld << world_point.id << "\t" << world_point.p.transpose() << "\t" << world_point.appearance.transpose() << endl;
    }
    out_est_traj.close();
    out_est_pose.close();
    out_est_wrld.close();
}