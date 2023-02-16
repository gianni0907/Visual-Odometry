#include "points_utils.h"
#include "camera.h"
#include "epipolar.h"
#include "projective_icp.h"
#include "read_data.h"
#include "correspondence_finder.h"

using namespace std;
using namespace vo;

int main(int argc, char** argv){
    //Take from keyboard the path to the dataset folder
    std::filesystem::path dataset;
    cout << "Insert the path to dataset folder" << endl;
    cin >> dataset;
    //Initialization from files
    Points3dVector gt_world=getWorld(dataset);
    ObsVector measurements=getObservations(dataset);
    Vector3fVector trajectory=getGroundTruthTrajectory(dataset);
    int max_points_leaf=20;
    float radius=0.1;
    //test CorrespondenceFinder between 2 sets of 2d points
    CorrespondenceFinder img_corr_finder=CorrespondenceFinder<Points2dVector,Points2dVector>();
    img_corr_finder.init(measurements[0].points,max_points_leaf,radius);
    img_corr_finder.compute(measurements[1].points);
    IntPairVector img_corr=img_corr_finder.correspondences();  
    cout << img_corr.size() << endl;
    for (size_t i=0;i<img_corr.size();i++){
        cout << img_corr[i].first << "\t" << img_corr[i].second << endl;
    }
    //test CorrespondenceFinder between a set of 3d point and a set of 2d point
    CorrespondenceFinder wrld_corr_finder=CorrespondenceFinder<Points3dVector,Points2dVector>();
    wrld_corr_finder.init(gt_world,max_points_leaf,radius);
    wrld_corr_finder.compute(measurements[0].points);
    IntPairVector wrld_corr=wrld_corr_finder.correspondences();  
    cout << wrld_corr.size() << endl;
    for (size_t i=0;i<wrld_corr.size();i++){
        cout << wrld_corr[i].first << "\t" << wrld_corr[i].second << endl;
    }
    return 0;
}