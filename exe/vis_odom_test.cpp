#include "points_utils.h"
#include "camera.h"
#include "epipolar.h"
#include "projective_icp.h"
#include "read_data.h"

using namespace std;
using namespace pr;

int main (int argc, char** argv){
    Points3dVector gt_world=getWorld();
    Camera cam=getCamera();
    ObsVector measurements=getObservations();
    TransfVector est_transf(N_POSES);
    int num_iterations=20;
    const bool keep_indices=false;
    for (int i=0;i<N_POSES;i++)
        est_transf[i].setIdentity();
    
    //estimate the transformation (world in first cam frame)
    //considering first pair of measurements
    est_transf[0]=estimateTransform(cam.cameraMatrix(),
                                    measurements[0].points,
                                    measurements[1].points);
    cout << "Estimated transformation:" << endl;
    cout << est_transf[0].linear() << endl << est_transf[0].translation() << endl;
    cout << endl;
    Eigen::Isometry3f gt_transf=v2t(measurements[1].gt_pose);
    gt_transf=gt_transf.inverse();
    cout << "Ground truth transformation:" << endl;
    cout << gt_transf.linear() << endl << gt_transf.translation() << endl;
    cout << endl;
    cout << "Ground truth/estimated transformation ratio:" << endl;
    for (int i=0;i<3;i++){
        for (int j=0;j<4;j++)
            cout << gt_transf(i,j)/est_transf[0](i,j) << " ";
        cout << endl;
    }
}