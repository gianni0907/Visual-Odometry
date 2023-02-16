#include "read_data.h"
#include "correspondence_finder.h"

using namespace std;
using namespace vo;

int main (int argc, char** argv){
    //Take from keyboard the path to the dataset folder
    std::filesystem::path dataset;
    cout << "Insert the path to dataset folder" << endl;
    cin >> dataset;
    //Read the ground truth world points
    Points3dVector gt_world=getWorld(dataset);
    //Read the ground truth trajectory
    Vector3fVector gt_trajectory=getGroundTruthTrajectory(dataset);

    //Read the estimated world points
    Points3dVector est_world=getEstimatedWorld();
    //Read the estimated robot in world pose
    TransfVector est_poses=getEstimatedPoses();

    //augment the ground truth to SE(3)
    TransfVector gt_poses(gt_trajectory.size());
    for (size_t i=0;i<gt_poses.size();i++)
        gt_poses[i]=v2t_aug(gt_trajectory[i]);

    //Retrieve the path to the current folder
    const std::filesystem::path& path=std::filesystem::current_path();
    //write on files the errors between two consecutive poses
    ofstream out_rot_err,out_trans_err;
    //write on a files the estimated scaled points, the corresponding gt point and the correspondences
    ofstream out_est_points_scaled,out_gt_points_pruned;
    ofstream out_corr;
    //write in a file also the gt trajectory, to ease comparison
    ofstream out_gt_traj;
    out_gt_traj.open(string(path)+"/../estimation/gt_trajectory.dat");
    if(!out_gt_traj){
        cerr << "Error gt_trajectory.dat file could not be opened" << endl;
        exit(1);
    }
    for (auto& pose : gt_trajectory)
        out_gt_traj << pose.x() << "\t" << pose.y() << "\t" << 0 << endl;

    out_rot_err.open(string(path)+"/../errors/rotation_err.dat");
    if(!out_rot_err){
        cerr << "Error: rotation_err.dat file could not be opened" << endl;
        exit(1);
    }
    out_trans_err.open(string(path)+"/../errors/translation_err.dat");
    if(!out_trans_err){
        cerr << "Error: translation_err.dat file could not be opened" << endl;
        exit(1);
    }
    out_est_points_scaled.open(string(path)+"/../estimation/est_points_scaled.dat");
    if(!out_est_points_scaled){
        cerr << "Error: est_points_scaled.dat file could not be opened" << endl;
        exit(1);
    }
    out_gt_points_pruned.open(string(path)+"/../estimation/gt_points_pruned.dat");
    if(!out_gt_points_pruned){
        cerr << "Error: gt_points_pruned.dat file could not be opened" << endl;
        exit(1);
    }
    out_corr.open(string(path)+"/../estimation/correspondences.dat");
    if(!out_corr){
        cerr << "Error: correspondences.dat file could not be opened" << endl;
        exit(1);
    }

    //compute the rotation and translation errors
    float ratio=0.0f;
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
        if (static_cast<int>(i)==0)
            ratio=trans_err;
        out_rot_err << rot_err << endl;
        out_trans_err << trans_err << endl;
    }

    //compute the scaled version of the estimated world points
    Points3dVector est_world_scaled(est_world.size());
    for (auto& point: est_world){
        Point3d scaled_point(point.id,point.p/ratio,point.appearance);
        est_world_scaled.push_back(scaled_point);
        out_est_points_scaled << scaled_point.id << "\t"
                              << scaled_point.p.transpose() << "\t"
                              << scaled_point.appearance.transpose() << endl;
    }

    //compute the whole RMSE considering correspondences between gt and estimated points
    Points3dVector gt_world_pruned(est_world.size());
    CorrespondenceFinder world_corr=CorrespondenceFinder<Points3dVector,Points3dVector>();
    world_corr.init(gt_world,20,0.1);
    world_corr.compute(est_world_scaled);
    IntPairVector correspondences=world_corr.correspondences();
    float rmse=0;
    for (size_t i=0;i<correspondences.size();i++){
        int gt_idx=correspondences[i].first;
        int est_idx=correspondences[i].second;
        out_gt_points_pruned << gt_world[gt_idx].id << "\t"
                             << gt_world[gt_idx].p.transpose() << "\t"
                             << gt_world[gt_idx].appearance.transpose() << endl;
        out_corr << gt_world[gt_idx].p.transpose() << "\t" << est_world_scaled[est_idx].p.transpose() << endl;
        float norm=(gt_world[gt_idx].p-est_world_scaled[est_idx].p).norm();
        rmse+=norm*norm;
    }
    rmse/=correspondences.size();
    rmse=sqrt(rmse);
    cout << "RMSE: " << rmse << endl;

    out_rot_err.close();
    out_trans_err.close();
    out_est_points_scaled.close();
    out_gt_points_pruned.close();
    out_corr.close();
}