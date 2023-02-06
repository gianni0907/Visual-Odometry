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
    cout << trajectory[120].x() << "," <<
            trajectory[120].y() << "," <<
            trajectory[120].z() << endl;

    //test world.dat file read
    Points3dVector world_points=getWorld();
    int num_points=world_points.size();
    cout << num_points << endl;
    cout << world_points[598].id << "\t[" <<
            world_points[598].p.x() << "," <<
            world_points[598].p.y() << "," <<
            world_points[598].p.z() << "]\t[";
    for (int i=0;i<world_points[598].appearance.size();i++)
        cout << world_points[598].appearance(i) << ",";
    cout << "]" << endl;

    //test meas-XXXXX.dat files read
    ObsVector observations=getObservations();
    int num_obs=observations.size();
    cout << num_obs << endl;
    cout << "[" << observations[65].gt_pose.x() << ","
                << observations[65].gt_pose.y() << "," 
                << observations[65].gt_pose.z() << "]\t" << endl;
    cout << "[" << observations[65].points[10].p.x() << ","
                << observations[65].points[10].p.y() << "]\t[";
    for (int i=0;i<observations[65].points[10].appearance.size();i++)
        cout << observations[65].points[10].appearance(i) << ",";
    cout << "]" << endl;
    for (size_t i=0;i<observations[65].points.size();i++)
        cout << observations[65].points[i].id << endl;
}