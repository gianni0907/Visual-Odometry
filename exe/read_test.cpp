#include "camera.h"
#include "read_data.h"

using namespace std;
using namespace vo;

int main (int args, char** argv){
    //Take from keyboard the path to the dataset folder
    std::filesystem::path dataset;
    cout << "Insert the path to dataset folder" << endl;
    cin >> dataset;

    //test camera.dat file read
    Camera cam;
    Eigen::Isometry3f cam_in_rob;
    cam=getCamera(dataset);
    Eigen::Vector2f dimension;
    dimension=cam.getDimension();
    cam_in_rob=cam.camInRobPose();
    cout << dimension(0) << "," << dimension(1) << endl;
    cout << cam_in_rob.matrix() << endl;

    //test trajectory.dat file read
    Vector3fVector trajectory=getGroundTruthTrajectory(dataset);
    int num_poses=trajectory.size();
    cout << num_poses << endl;
    cout << trajectory[120].x() << "," <<
            trajectory[120].y() << "," <<
            trajectory[120].z() << endl;

    //test world.dat file read
    Points3dVector world_points=getWorld(dataset);
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
    ObsVector observations=getObservations(dataset);
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

    //test est_pose.dat read
    TransfVector est_pose=getEstimatedPoses();
    cout << est_pose[N_POSES-1].matrix() << endl;

    //test est_world.dat read
    Points3dVector est_points=getEstimatedWorld();
    for (size_t i=0;i<est_points.size();i++)
        cout << est_points[i].id << "\t" << est_points[i].p.transpose() << endl;
}