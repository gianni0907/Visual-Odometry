#include "read_data.h"

namespace pr{
    using namespace std;

    Camera getCamera(){
        Eigen::Matrix3f K;
        Eigen::Isometry3f cam_in_rob;
        float z_near,z_far,width,height;
        char trash[100];
        ifstream indata;
        char* path=getenv("HOME");
        indata.open(string(path) + "/Desktop/probrob_proj/02-VisualOdometry/data/camera.dat");
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

    Vector3fVector getGroundTruthTrajectory(){
        Vector3fVector trajectory(N_POSES);
        ifstream indata;
        float trash;
        char* path=getenv("HOME");
        int curr_pose=0;
        indata.open(string(path) + "/Desktop/probrob_proj/02-VisualOdometry/data/trajectory.dat");
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

    Points3dVector getWorld(){
        Points3dVector world_points(N_POINTS);
        int curr_point=0;
        ifstream indata;
        char* path=getenv("HOME");
        indata.open(string(path) + "/Desktop/probrob_proj/02-VisualOdometry/data/world.dat");
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

    ObsVector getObservations(){
        ObsVector observations(N_POSES);
        int curr_meas=0;
        int curr_point=0;
        char trash_word[100];
        float trash_float;
        ifstream indata;
        const char* path1="/Desktop/probrob_proj/02-VisualOdometry/data/meas-0000";
        const char* path2="/Desktop/probrob_proj/02-VisualOdometry/data/meas-000";
        const char* path3="/Desktop/probrob_proj/02-VisualOdometry/data/meas-00";
        const char* home=getenv("HOME");
        for (auto& meas: observations){
            meas.points.resize(N_POINTS);
            if (curr_meas<10)
                indata.open(string(home) + string(path1) + to_string(curr_meas) + ".dat");
            else if (curr_meas<100)
                indata.open(string(home) + string(path2) + to_string(curr_meas) + ".dat");
            else
                indata.open(string(home) + string(path3) + to_string(curr_meas) + ".dat");
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

}