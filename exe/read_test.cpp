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

    //test world.dat file read
    Points3dVector world_points=getWorld();
    int num_points=world_points.size();
    cout << num_points << endl;
    world_points.resize(num_points);
    cout << world_points[598].id << "\t[" <<
            world_points[598].p.x() << "," <<
            world_points[598].p.y() << "," <<
            world_points[598].p.z() << "]\t[";
         for (int i=0;i<world_points[598].appearance.size();i++)
            cout << world_points[598].appearance(i) << ",";
         cout << "]" << endl;
}