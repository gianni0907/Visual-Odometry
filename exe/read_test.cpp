#include "camera.h"
#include "read_data.h"

using namespace std;
using namespace pr;

int main (int args, char** argv){

    //test camera.dat file read
    Camera cam;
    cam=getCamera();
    Eigen::Vector2f dimension;
    dimension=cam.getDimension();
    cout << dimension(0) << "," << dimension(1) << endl;

    //test trajectory.dat file read
    Vector3fVector trajectory=getGroundTruthTrajectory();
    int num_poses=trajectory.size();
    cout << num_poses << endl;
    trajectory.resize(num_poses);
    cout << trajectory[120].x() << "," <<
            trajectory[120].y() << "," <<
            trajectory[120].z() << endl;
}