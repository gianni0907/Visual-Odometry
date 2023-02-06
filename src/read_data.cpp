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
            cerr << "Error: file could not be opened" << endl;
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
}