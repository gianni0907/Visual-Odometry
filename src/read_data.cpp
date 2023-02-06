#include "read_data.h"

namespace pr{
    using namespace std;

    Camera getCamera(){
        Eigen::Matrix3f K;
        float z_near,z_far,width,height;
        char word[100];
        ifstream indata;
        char* path=getenv("HOME");
        indata.open(string(path) + "/Desktop/probrob_proj/02-VisualOdometry/data/camera.dat");
        if(!indata){
            cerr << "Error: camera.dat file could not be opened" << endl;
            exit(1);
        }
        indata.getline(word,100,'\n');
        for (int i=0;i<9;i++){
            indata >> K(i/3,i%3);
        }
        for (int i=0;i<6;i++){
            indata.getline(word,100,'\n');
        }
        indata.getline(word,10,' ');
        indata >> z_near;
        indata.getline(word,10,' ');
        indata >> z_far;
        indata.getline(word,10,' ');
        indata >> width;
        indata.getline(word,10,' ');
        indata >> height;
        indata.close();
        Camera cam(height,width,z_near,z_far,K);
        return cam;
    }

    int getGroundTruthTrajectory(Vector3fVector& trajectory){
        trajectory.resize(150);
        ifstream indata;
        float trash;
        char* path=getenv("HOME");
        int num_poses=0;
        indata.open(string(path) + "/Desktop/probrob_proj/02-VisualOdometry/data/trajectory.dat");
        if(!indata){
            cerr << "Error: trajectory.dat file could not be opened" << endl;
            exit(1);
        }
        while(!indata.eof()){
            for (int i=0;i<4;i++)
                indata >> trash;
            indata >> trajectory[num_poses].x()
                   >> trajectory[num_poses].y()
                   >> trajectory[num_poses].z();
            num_poses++;
        }
        num_poses--;
        trajectory.resize(num_poses);
        return num_poses;
    }
}