#include "read_data.h"

using namespace std;
using namespace vo;

int main (int argc, char** argv){
    //Read the ground truth world points
    Points3dVector gt_world=getWorld();
    //Read the ground truth trajectory
    Vector3fVector gt_trajectory=getGroundTruthTrajectory();

    //Read the estimated world points
    Vector3fVector est_world=getEstimatedWorld();
    //Read the estimated robot in world pose
    TransfVector est_poses=getEstimatedPoses();

    //augment the ground truth to SE(3)
    TransfVector gt_poses(N_POSES);
    for (size_t i=0;i<N_POSES;i++)
        gt_poses[i]=v2t_aug(gt_trajectory[i]);

    //write on a file the errors between two consecutive poses
    ofstream out_rot_err,out_trans_err,out_est_points_scaled;
    char* path=getenv("HOME");
    out_rot_err.open(string(path)+"/Desktop/probrob_proj/errors/rotation_err.dat");
    if(!indata){
        cerr << "Error: rotation_err.dat file could not be opened" << endl;
        exit(1);
    }
    out_trans_err.open(string(path)+"/Desktop/probrob_proj/errors/translation_err.dat");
    if(!indata){
        cerr << "Error: translation_err.dat file could not be opened" << endl;
        exit(1);
    }
    out_est_points_scaled.open(string(path)+"/Desktop/probrob_proj/estimation/est_points_scaled.dat");
    if(!indata){
        cerr << "Error: est_points_scaled.dat file could not be opened" << endl;
        exit(1);
    }

    float mean_ratio=0;
    Vector3fVector est_world_scaled(est_world.size());
    for (size_t i=0;i<N_POSES-1;i++){
        Eigen::Isometry3f rel_T=Eigen::Isometry3f::Identity();
        Eigen::Isometry3f rel_GT=Eigen::Isometry3f::Identity();
        Eigen::Isometry3f error_T=Eigen::Isometry3f::Identity();
        float rot_err,trans_err;
        
        rel_T=est_poses[i].inverse()*est_poses[i+1];
        rel_GT=gt_poses[i].inverse()*gt_poses[i+1];
        error_T=rel_T.inverse()*rel_GT;
        rot_err=(Eigen::Matrix3f::Identity()-error_T.linear()).trace();
        trans_err=(rel_T.translation().norm())/(rel_GT.translation().norm());
        if (trans_err != INFINITY)
            mean_ratio+=trans_err;
        out_rot_err << rot_err << endl;
        out_trans_err << trans_err << endl;
    }
    mean_ratio/=N_POSES-1;
    for (auto& point: est_world){
        est_world_scaled=est_world/mean_ratio;
        cout << est_world_scaled.transpose() << endl;
    }
    out_rot_err.close();
    out_trans_err.close();
    out_est_points_scaled.close();
}