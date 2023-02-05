#include "camera.h"
#include "points_utils.h"
#include "epipolar.h"
#include "projective_icp.h"

using namespace std;
using namespace pr;

int main (int argc, char** argv){
    Points3dVector world_points;
    int num_transf=3;
    vector<Camera> cameras(num_transf+1);
    vector<Camera> est_cameras(num_transf-1);
    vector<Points2dVector> img_points(num_transf+1);
    Points3dVector triang_points;
    vector<float> errors;
    TransfVector rel_transf(num_transf);
    TransfVector abs_transf(num_transf); //world wrt cameras!!!
    TransfVector est_transf(num_transf);
    Eigen::Vector3f lower_left_bottom(-10,-10,-10);
    Eigen::Vector3f upper_right_top(10,10,10);
    int num_points=1000;
    vector<int> num_projected_points(num_transf+1); 
    vector<int> num_triangulated_points(num_transf);
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

    //consider three relative ground truth transformations
    //Note:they represent the cam1 pose wrt world, cam2 pose wrt cam1 and so on
    for (int i=0;i<num_transf;i++){
        rel_transf[i]=Eigen::Isometry3f::Identity();
        abs_transf[i]=Eigen::Isometry3f::Identity();
    }        
    float x=0.2f,x_step=0.2f;
    float y=0.0f,y_step=0.1f;
    float th=0.2f,th_step=0.1f;
    for (int i=0; i<num_transf;i++){
        rel_transf[i].linear()=Rz(th);
        rel_transf[i].translation()=Eigen::Vector3f(x,y,0.0f);
        if (i==0)
            abs_transf[i]=rel_transf[i].inverse();
        else
            abs_transf[i]=rel_transf[i].inverse()*abs_transf[i-1];
    }
    for (int i=0; i<num_transf;i++){
        cout << "Ground truth relative transformation [" << i << "," << (i+1) << "]: " << endl;
        cout << rel_transf[i].linear() << endl << rel_transf[i].translation() << endl;
        cout << endl;
        cout << "Ground truth absolute transformation (inverted) [" << i+1 << ",0]: " << endl;
        cout << abs_transf[i].linear() << endl << abs_transf[i].translation() << endl;
        cout << endl;
    }

    //instantiate 4 cameras (1 for world, then the other 3)
    //and project points on image plane for each camera
    cameras[0]=Camera(height,width,z_near,z_far,camera_matrix);
    num_projected_points[0]=cameras[0].projectPoints(img_points[0],world_points,keep_indices);
    for (size_t i=1; i<cameras.size();i++){
        cameras[i]=Camera(height,width,z_near,z_far,camera_matrix,abs_transf[i-1]);
        num_projected_points[i]=cameras[i].projectPoints(img_points[i],world_points,keep_indices);
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
            cout << abs_transf[1](i,j)/est_transf[0](i,j) << " ";
        cout << endl;
    }

    //then triangulate to obtain points in the world
    num_triangulated_points[0]=triangulatePoints(camera_matrix,
                                                 est_transf[0],
                                                 img_points[0],
                                                 img_points[1],
                                                 triang_points,
                                                 errors);
    
    // //compare generated/triangulated[0,1] world_points
    // computeWrld2WrldCorrespondences(world_corr,
    //                                 world_points,
    //                                 triang_points);
    // cout <<"Compare generated/triangulated[0,1] world_points: " << num_triangulated_points[0] << endl;
    // for (const IntPair& corr: world_corr){
    //     int wrl_idx=corr.second;
    //     int tri_idx=corr.first;
    //     cout << "[" << world_points[wrl_idx].p.x() <<
    //             "," << world_points[wrl_idx].p.y() <<
    //             "," << world_points[wrl_idx].p.z() <<
    //             "]\t[" << triang_points[tri_idx].p.x() <<
    //             "," << triang_points[tri_idx].p.y() <<
    //             "," << triang_points[tri_idx].p.z() << "]" << endl;
    // }
    ////NOTE: PICP done considering pose of WORLD expressed in cameras
    //projective_icp
    //consider:
    //  triangulated points as world_points
    //  images of cameras 0 and 2
    //set initial guess for the pose of world (cameras[0]) wrt cameras[2]
    Eigen::Isometry3f init_X=Eigen::Isometry3f::Identity();
    est_cameras[0]=Camera(height,width,z_near,z_far,camera_matrix,init_X);

    PICPSolver solver;
    solver.init(est_cameras[0],
                triang_points,
                img_points[2],
                num_iterations,
                keep_indices);
    solver.setKernelThreshold(10000);
    solver.run();
    est_cameras[0]=solver.camera();
    est_transf[1]=est_cameras[0].worldInCameraPose();
    // cout << "Estimated transformation:" << endl;
    // cout << est_transf[1].linear() << endl << est_transf[1].translation() << endl;
    // cout << endl;
    // cout << "Ground truth/estimated transformation ratio:" << endl;
    // for (int i=0;i<3;i++){
    //     for (int j=0;j<4;j++)
    //         cout << abs_transf[1](i,j)/est_transf[1](i,j) << " ";
    //     cout << endl;
    // }
    Points3dVector new_triang;
    num_triangulated_points[1]=triangulatePoints(camera_matrix,
                                                 est_transf[1],
                                                 img_points[0],
                                                 img_points[2],
                                                 new_triang,
                                                 errors);
    // //compare generated/triangulated[0,2] world_points
    // computeWrld2WrldCorrespondences(world_corr,
    //                                 world_points,
    //                                 new_triang);
    // cout <<"Compare generated/triangulated[0,2] world_points: " << num_triangulated_points[1] << endl;
    // for (const IntPair& corr: world_corr){
    //     int wrl_idx=corr.second;
    //     int tri_idx=corr.first;
    //     cout << "[" << world_points[wrl_idx].p.x() <<
    //             "," << world_points[wrl_idx].p.y() <<
    //             "," << world_points[wrl_idx].p.z() <<
    //             "]\t[" << new_triang[tri_idx].p.x() <<
    //             "," << new_triang[tri_idx].p.y() <<
    //             "," << new_triang[tri_idx].p.z() << "]" << endl;
    // }
    Points3dVector merged_points;
    merged_points=mergePoints(new_triang,triang_points);
    // cout <<"Merged world_points: " << merged_points.size() << endl;
    // for (const Point3d& point: merged_points){
    //     cout << "[" << point.p.x() <<
    //             "," << point.p.y() <<
    //             "," << point.p.z() << "]" << endl;
    // }
    PICPSolver solver1;
    est_cameras[0].setWorldInCameraPose(init_X);
    solver1.init(est_cameras[0],
                 merged_points,
                 img_points[1],
                 num_iterations,
                 keep_indices);
    solver1.setKernelThreshold(10000);
    solver1.run();
    est_cameras[0]=solver1.camera();
    est_transf[0]=est_cameras[0].worldInCameraPose();
    cout << "Estimated transformation with merged points:" << endl;
    cout << est_transf[0].linear() << endl << est_transf[0].translation() << endl;
    cout << endl;
    cout << "Ground truth/estimated transformation ratio:" << endl;
    for (int i=0;i<3;i++){
        for (int j=0;j<4;j++)
            cout << abs_transf[1](i,j)/est_transf[0](i,j) << " ";
        cout << endl;
    }
    return 0;
}