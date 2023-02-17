#include "camera.h"
#include "points_utils.h"
#include "epipolar.h"
#include "projective_icp.h"

using namespace std;
using namespace vo;

int main (int argc, char** argv){
    Points3dVector world_points;
    int num_transf=2;
    Camera cam,est_camera;
    vector<Points2dVector> img_points(num_transf+1);
    Points3dVector triang_points;
    vector<float> errors;
    TransfVector rel_transf(num_transf);        //i wrt i-1
    TransfVector inv_rel_transf(num_transf);    //i-1 wrt i
    TransfVector abs_transf(num_transf);        //0(world) wrt i!!!
    TransfVector est_transf(num_transf);        //i-1 wrt i (in case of relative picp)
    Eigen::Vector3f lower_left_bottom(-10,-10,0.0f);
    Eigen::Vector3f upper_right_top(10,10,2.0f);
    int num_points=1000;
    int num_triangulated_points;
    IntPairVector world_corr;
    int num_iterations=20;
    const bool keep_indices=false;
    
    //generate points in world
    makeWorld(world_points,
              lower_left_bottom,
              upper_right_top,
              num_points);

    //set cameras parameters
    Eigen::Matrix3f camera_matrix;
    camera_matrix << 180, 0, 320,
                     0, 180,240,
                     0, 0, 1;
    int width=640;
    int height=480;
    float z_near=0;
    float z_far=5;

    //initialize to identity all the transformations
    for (int i=0;i<num_transf;i++){
        rel_transf[i]=Eigen::Isometry3f::Identity();
        abs_transf[i]=Eigen::Isometry3f::Identity();
        inv_rel_transf[i]=Eigen::Isometry3f::Identity();
        est_transf[i]=Eigen::Isometry3f::Identity();
    }        
    float x=0.2f;
    float y=0.15f;
    float th=0.0f;
    //consider num_transf relative ground truth transformations
    for (int i=0; i<num_transf;i++){
        rel_transf[i].linear()=Rz(th);
        rel_transf[i].translation()=Eigen::Vector3f(x,y,0.0f);
        inv_rel_transf[i]=rel_transf[i].inverse();
        if (i==0)
            abs_transf[i]=rel_transf[i].inverse();
        else
            abs_transf[i]=rel_transf[i].inverse()*abs_transf[i-1];
    }

    //Generate synthetics measurements
    cam=Camera(height,width,z_near,z_far,camera_matrix);
    cam.projectPoints(img_points[0],world_points,keep_indices);
    for (int i=1; i<num_transf+1;i++){
        cam.setWorldInCameraPose(abs_transf[i-1]);
        cam.projectPoints(img_points[i],world_points,keep_indices);
    }

    //estimate the transformation (world in first cam frame)
    //considering first pair of img_points
    est_transf[0]=estimateTransform(camera_matrix,img_points[0],img_points[1]);
    cout << "Estimated transformation:" << endl;
    cout << est_transf[0].linear() << endl << est_transf[0].translation() << endl;
    cout << endl;
    cout << "Ground truth/estimated transformation ratio:" << endl;
    for (int i=0;i<3;i++){
        for (int j=0;j<4;j++)
            cout << inv_rel_transf[0](i,j)/est_transf[0](i,j) << " ";
        cout << endl;
    }
    cout << endl;

    //then triangulate to obtain points expressed in cam1 ref frame
    //since relative icp is realized
    num_triangulated_points=triangulatePoints(camera_matrix,
                                                 est_transf[0].inverse(),
                                                 img_points[1],
                                                 img_points[0],
                                                 triang_points,
                                                 errors);
    cout << "Number of triangulated points after first triangulation: " << num_triangulated_points << endl;
    cout << endl;
    ////NOTE: PICP done considering pose of camera i-1 expressed in camera i
    //consider:
    //  triangulated points as world_points
    //  images of cameras 1 and 2
    //set initial guess for the pose of cameras[1] wrt cameras[2]
    Eigen::Isometry3f init_X=Eigen::Isometry3f::Identity();
    est_camera=Camera(height,width,z_near,z_far,camera_matrix,init_X);

    PICPSolver solver;
    solver.init(est_camera,
                triang_points,
                img_points[2],
                num_iterations,
                keep_indices);
    solver.setKernelThreshold(10000);
    solver.run();
    est_camera=solver.camera();
    est_transf[1]=est_camera.worldInCameraPose();
    cout << "Estimated transformation:" << endl;
    cout << est_transf[1].linear() << endl << est_transf[1].translation() << endl;
    cout << endl;
    cout << "Ground truth/estimated transformation ratio:" << endl;
    for (int i=0;i<3;i++){
        for (int j=0;j<4;j++)
            cout << inv_rel_transf[1](i,j)/est_transf[1](i,j) << " ";
        cout << endl;
    }
    cout << endl;
    Points3dVector new_triang;
    num_triangulated_points=triangulatePoints(camera_matrix,
                                                 est_transf[1].inverse(),
                                                 img_points[2],
                                                 img_points[1],
                                                 new_triang,
                                                 errors);

    Points3dVector merged_points;
    merged_points=mergePoints(est_transf[1],new_triang,triang_points);
    cout <<"Merged world_points: " << merged_points.size() << endl;
    for (const Point3d& point: merged_points){
        cout << "[" << point.p.x() <<
                "," << point.p.y() <<
                "," << point.p.z() << "]" << endl;
    }
}