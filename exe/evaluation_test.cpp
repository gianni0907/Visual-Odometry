#include "read_data.h"
#include "correspondence_finder.h"

using namespace std;
using namespace vo;

int main (int argc, char** argv){
    //Read the ground truth world points
    Points3dVector gt_world=getWorld();
    //Read the ground truth trajectory
    Vector3fVector gt_trajectory=getGroundTruthTrajectory();

    //Read the estimated world points
    Points3dVector est_world=getEstimatedWorld();
    //Read the estimated robot in world pose
    TransfVector est_poses=getEstimatedPoses();

    //augment the ground truth to SE(3)
    TransfVector gt_poses(gt_trajectory.size());
    for (size_t i=0;i<gt_poses.size();i++)
        gt_poses[i]=v2t_aug(gt_trajectory[i]);

    //write on a file the errors between two consecutive poses
    ofstream out_rot_err,out_trans_err,out_est_points_scaled;
    char* path=getenv("HOME");
    out_rot_err.open(string(path)+"/Desktop/probrob_proj/errors/rotation_err.dat");
    if(!out_rot_err){
        cerr << "Error: rotation_err.dat file could not be opened" << endl;
        exit(1);
    }
    out_trans_err.open(string(path)+"/Desktop/probrob_proj/errors/translation_err.dat");
    if(!out_trans_err){
        cerr << "Error: translation_err.dat file could not be opened" << endl;
        exit(1);
    }
    out_est_points_scaled.open(string(path)+"/Desktop/probrob_proj/estimation/est_points_scaled.dat");
    if(!out_est_points_scaled){
        cerr << "Error: est_points_scaled.dat file could not be opened" << endl;
        exit(1);
    }

    //compute the rotation and translation errors
    float mean_ratio=0;
    for (size_t i=0;i<est_poses.size()-1;i++){
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

    //compute the scaled version of the estimated world points
    Points3dVector est_world_scaled(est_world.size());
    mean_ratio/=N_POSES-1;
    for (auto& point: est_world){
        Point3d scaled_point(point.id,point.p/mean_ratio,point.appearance);
        est_world_scaled.push_back(scaled_point);
        out_est_points_scaled << scaled_point.id << "\t" << scaled_point.p.transpose() << "\t" << scaled_point.appearance.transpose() << endl;
    }

    //compute the whole RMSE considering correspondences between gt and estimated points
    CorrespondenceFinder world_corr=CorrespondenceFinder<Points3dVector,Points3dVector>();
    world_corr.init(gt_world,20,0.1);
    world_corr.compute(est_world_scaled);
    IntPairVector correspondences=world_corr.correspondences();
    float rmse=0;
    for (size_t i=0;i<correspondences.size();i++){
        int gt_idx=correspondences[i].first;
        int est_idx=correspondences[i].second; 
        rmse+=(gt_world[gt_idx].p-est_world_scaled[est_idx].p).norm();
    }
    rmse/=correspondences.size();
    rmse=sqrt(rmse);
    cout << rmse << endl;

    out_rot_err.close();
    out_trans_err.close();
    out_est_points_scaled.close();
}