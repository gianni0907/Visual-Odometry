#include "points_utils.h"
#include "camera.h"
#include "epipolar.h"
#include "projective_icp.h"
#include "read_data.h"
#include "correspondence_finder.h"

using namespace std;
using namespace pr;

int main(int argc, char** argv){
    //Initialization from files
    Points3dVector gt_world=getWorld();
    Camera cam=getCamera();
    Eigen::Matrix3f K=cam.cameraMatrix();
    Eigen::Isometry3f cam_in_rob=cam.camInRobPose();
    ObsVector measurements=getObservations();
    Vector3fVector trajectory=getGroundTruthTrajectory();
    int max_points_leaf=20;
    float radius=0.1;
    CorrespondenceFinder corr_finder=CorrespondenceFinder<Points2dVector,Points2dVector>();
    corr_finder.init(measurements[0].points,max_points_leaf,radius);
    corr_finder.compute(measurements[1].points);
    IntPairVector corr=corr_finder.correspondences();  
    cout << corr.size() << endl;
    for (size_t i=0;i<corr.size();i++){
        cout << corr[i].first << "\t" << corr[i].second << endl;
    }
    return 0;
}