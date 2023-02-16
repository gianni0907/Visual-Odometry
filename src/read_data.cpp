#include "read_data.h"

namespace vo{
    using namespace std;

    Camera getCamera(std::filesystem::path& dataset){
        Eigen::Matrix3f K;
        Eigen::Isometry3f cam_in_rob;
        float z_near,z_far,width,height;
        char trash[100];
        ifstream indata;
        indata.open(string(dataset) + "/camera.dat");
        if(!indata){
            cerr << "Error: camera.dat file could not be opened" << endl;
            exit(1);
        }
        indata.getline(trash,100,'\n');
        for (int i=0;i<9;i++){
            indata >> K(i/3,i%3);
        }
        indata.getline(trash,100,'\n');
        indata.getline(trash,100,'\n');
        for (int i=0;i<16;i++){
            indata >> cam_in_rob(i/4,i%4);
        }
        indata.getline(trash,10,' ');
        indata >> z_near;
        indata.getline(trash,10,' ');
        indata >> z_far;
        indata.getline(trash,10,' ');
        indata >> width;
        indata.getline(trash,10,' ');
        indata >> height;
        indata.close();
        Camera cam(height,width,z_near,z_far,K,cam_in_rob.inverse(),cam_in_rob);
        return cam;
    }

    Vector3fVector getGroundTruthTrajectory(std::filesystem::path& dataset){
        Vector3fVector trajectory(N_POSES);
        ifstream indata;
        float trash;
        int curr_pose=0;
        indata.open(string(dataset) + "/trajectory.dat");
        if(!indata){
            cerr << "Error: trajectory.dat file could not be opened" << endl;
            exit(1);
        }
        while(!indata.eof()){
            for (int i=0;i<4;i++)
                indata >> trash;
            indata >> trajectory[curr_pose].x()
                   >> trajectory[curr_pose].y()
                   >> trajectory[curr_pose].z();
            curr_pose++;
        }
        indata.close();
        return trajectory;
    }

    Points3dVector getWorld(std::filesystem::path& dataset){
        Points3dVector world_points(N_POINTS);
        int curr_point=0;
        ifstream indata;
        indata.open(string(dataset) + "/world.dat");
        if(!indata){
            cerr << "Error: world.dat file could not be opened" << endl;
            exit(1);
        }
        while(!indata.eof()){
            indata >> world_points[curr_point].id;
            indata >> world_points[curr_point].p.x()
                   >> world_points[curr_point].p.y()
                   >> world_points[curr_point].p.z();
            for (int i=0;i<world_points[curr_point].appearance.size();i++)
                indata >> world_points[curr_point].appearance(i);
            curr_point++;
        }
        indata.close();
        return world_points;
    }

    ObsVector getObservations(std::filesystem::path& dataset){
        ObsVector observations(N_POSES);
        int curr_meas=0;
        int curr_point=0;
        char trash_word[100];
        float trash_float;
        ifstream indata;
        const char* path1="/meas-0000";
        const char* path2="/meas-000";
        const char* path3="/meas-00";
        for (auto& meas: observations){
            meas.points.resize(N_POINTS);
            if (curr_meas<10)
                indata.open(string(dataset) + string(path1) + to_string(curr_meas) + ".dat");
            else if (curr_meas<100)
                indata.open(string(dataset) + string(path2) + to_string(curr_meas) + ".dat");
            else
                indata.open(string(dataset) + string(path3) + to_string(curr_meas) + ".dat");
            if(!indata){
            cerr << "Error: file could not be opened" << endl;
            exit(1);
            }
            indata.getline(trash_word,100,'\n');
            indata.getline(trash_word,100,' ');
            indata >> meas.gt_pose.x()
                   >> meas.gt_pose.y()
                   >> meas.gt_pose.z();
            indata.getline(trash_word,100,'\n');
            indata.getline(trash_word,100,' ');
            for (int i=0;i<3;i++)
                indata >> trash_float;
            while(!indata.eof()){
                indata.getline(trash_word,100,' ');
                indata >> curr_point;
                indata >> meas.points[curr_point].id;
                indata >> meas.points[curr_point].p.x()
                       >> meas.points[curr_point].p.y();
                for (int j=0;j<meas.points[curr_point].appearance.size();j++)
                    indata >> meas.points[curr_point].appearance(j);
            }
            meas.points.resize(curr_point+1);
            indata.close();
            curr_meas++;
        }
        return observations;
    }

    Vector3fVector getEstimatedTrajectory()
    {
        Vector3fVector trajectory(N_POSES);
        ifstream indata;
        const std::filesystem::path& path=std::filesystem::current_path();
        int curr_pose=0;
        indata.open(string(path) + "/../estimation/est_trajectory.dat");
        if(!indata){
            cerr << "Error: est_trajectory.dat file could not be opened" << endl;
            exit(1);
        }
        while(!indata.eof()){
            indata >> trajectory[curr_pose].x()
                   >> trajectory[curr_pose].y()
                   >> trajectory[curr_pose].z();
            curr_pose++;
        }
        indata.close();
        return trajectory;
    }

    TransfVector getEstimatedPoses()
    {
        TransfVector est_pose(N_POSES);
        int curr_pose=0;
        ifstream indata;
        char trash[100];
        const std::filesystem::path& path=std::filesystem::current_path();
        indata.open(string(path) + "/../estimation/est_pose.dat");
        if(!indata){
            cerr << "Error: est_pose.dat file could not be opened" << endl;
            exit(1);
        }
        while(!indata.eof()){
            for (int i=0;i<16;i++){
                indata >> est_pose[curr_pose](i/4,i%4);
            }
            curr_pose++;
            indata.getline(trash,100,'\n');
        }
        indata.close();
        est_pose.resize(curr_pose-1);
        return est_pose;
    }

    Points3dVector getEstimatedWorld()
    {
        Points3dVector est_points(N_POINTS);
        int curr_point=0;
        ifstream indata;
        const std::filesystem::path& path=std::filesystem::current_path();
        indata.open(string(path) + "/../estimation/est_world.dat");
        if(!indata){
            cerr << "Error: est_world.dat file could not be opened" << endl;
            exit(1);
        }
        while(!indata.eof()){
            indata >> est_points[curr_point].id;
            indata >> est_points[curr_point].p.x()
                   >> est_points[curr_point].p.y()
                   >> est_points[curr_point].p.z();
            for (int i=0;i<est_points[curr_point].appearance.size();i++)
                indata >> est_points[curr_point].appearance(i);
            curr_point++;
        }
        indata.close();
        est_points.resize(curr_point-1);
        return est_points;
    }
}