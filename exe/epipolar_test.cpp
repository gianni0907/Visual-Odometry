#include "camera.h"
#include "points_utils.h"
#include "epipolar.h"

using namespace std;
using namespace pr;

int main(int argc, char** argv){

    Points3dVector world_points,triangulated_world_points;
    vector<float> errors;
    Points2dVector img1_points, img2_points;
    Points2dVector img1_matching_points, img2_matching_points;
    Eigen::Vector3f lower_left_bottom(-10,-10,-10);
    Eigen::Vector3f upper_right_top(10,10,10);
    int num_points=1000;
    int num_projected_points1=0;
    int num_projected_points2=0;
    int num_matching_points=0;
    int num_triangulated_points=0;
    const bool keep_indices=false;

    //create points in the world
    makeWorld(world_points,
              lower_left_bottom,
              upper_right_top,
              num_points);

    cout << "Generated model with " << world_points.size() << " points" << endl;

    //Consider an homogeneous transformation expressing the cam1(world) wrt cam2
    Eigen::Isometry3f relative_X=Eigen::Isometry3f::Identity();
    relative_X.linear()=Rz(0.15);
    relative_X.translation()=Eigen::Vector3f(-2.0f, 1.5f, 0.0f);
    cout << "Ground truth transformation:" << endl;
    cout << relative_X.linear() << endl << relative_X.translation() << endl;
    cout << endl;
    
    // set the camera parameters
    Eigen::Matrix3f camera_matrix;
    camera_matrix << 180, 0, 320,
                     0, 180, 240,
                     0, 0, 1;
    int width=640;
    int height=480;
    float z_near=0;
    float z_far=5;

    //instantiate the two cameras from which we take images
    Camera cam1(height, width, z_near, z_far, camera_matrix);
    Camera cam2(height, width, z_near, z_far, camera_matrix, relative_X);

    //TEST PROJECTION IN IMAGE PLANES
    //project the points in the world in both the cameras image planes
    char key=0;
    const char ESC_key=27;
    while (key!=ESC_key){
        num_projected_points1=cam1.projectPoints(img1_points, world_points, keep_indices);
        num_projected_points2=cam2.projectPoints(img2_points, world_points, keep_indices);
        cout << "Number of projected points on img1: " << num_projected_points1 << endl;
        cout << "Number of projected points on img2: " << num_projected_points2 << endl;        
        RGBImage shown_image1(height,width);
        RGBImage shown_image2(height,width);
        shown_image1=cv::Vec3b(255,255,255);
        shown_image2=cv::Vec3b(255,255,255);
        drawPoints(shown_image1, img1_points, cv::Scalar(255,0,0),3);
        drawPoints(shown_image2, img2_points, cv::Scalar(255,0,0),3);
        cv::imshow("projection_img1_test", shown_image1);
        cv::imshow("projection_img2_test", shown_image2);
        key=cv::waitKey(0);
    }
    //TEST PRUNING THE UNMATCHING PROJECTED POINTS
    num_matching_points=pruneUnmatchingProjectedPoints(img1_points,
                                                       img2_points,
                                                       img1_matching_points,
                                                       img2_matching_points);
    cout << "Number of matching points: " << num_matching_points << endl;

    //TEST TRIANGULATION
    num_triangulated_points=triangulatePoints(camera_matrix,
                                              relative_X,
                                              img1_points,
                                              img2_points,
                                              triangulated_world_points,
                                              errors);
    cout << "Number of triangulated points: " << num_triangulated_points << endl;
    key=0;
    while (key!=ESC_key){
        cam1.projectPoints(img1_points, triangulated_world_points, keep_indices);
        RGBImage shown_image3(height,width);
        shown_image3=cv::Vec3b(255,255,255);
        drawPoints(shown_image3, img1_points, cv::Scalar(255,0,0),3);
        cv::imshow("triangulation_test", shown_image3);
        key=cv::waitKey(0);
    }

    // //TEST transform2essential() and viceversa
    // Eigen::Matrix3f essential, fundamental;
    // Eigen::Isometry3f computed_X1,computed_X2;
    // computed_X1=Eigen::Isometry3f::Identity();
    // computed_X2=Eigen::Isometry3f::Identity();
    // essential=transform2essential(relative_X);
    // essential2transform(essential,computed_X1,computed_X2);
    // cout << computed_X1.linear() << endl;
    // cout << computed_X1.translation() << endl;
    // cout << endl;
    // cout << computed_X2.linear() << endl;
    // cout << computed_X2.translation() << endl;

    // //TEST transform2fundamental and viceversa
    // fundamental=transform2fundamental(camera_matrix,relative_X);
    // fundamental2transform(camera_matrix,fundamental,computed_X1,computed_X2);
    // cout << computed_X1.linear() << endl;
    // cout << computed_X1.translation() << endl;
    // cout << endl;
    // cout << computed_X2.linear() << endl;
    // cout << computed_X2.translation() << endl;

    //TEST estimateFundamental and estimateTransform
    Eigen::Matrix3f F_gt,F,E_gt,E;
    Eigen::Isometry3f X1,X2,X_est;
    F_gt=transform2fundamental(camera_matrix,relative_X);
    F=estimateFundamental(img1_points,img2_points);
    cout << endl;
    cout << "Ground truth F:" << endl;
    cout << F_gt << endl;
    cout << endl;
    cout << "Estimated F:" << endl;
    cout << F << endl;
    cout << endl;
    cout << "Ground truth/estimated fundamental ratio:" << endl;
    for (int i=0;i<3;i++){
        for (int j=0;j<3;j++)
            cout << F_gt(i,j)/F(i,j) << " ";
        cout << endl;
    }
    cout << endl;
    E_gt=transform2essential(relative_X);
    E=camera_matrix.transpose()*F*camera_matrix;
    cout << "Ground truth E:" << endl;
    cout << E_gt << endl;
    cout << endl;
    cout << "Estimated E:" << endl;
    cout << E << endl;
    cout << endl;
    fundamental2transform(camera_matrix,F,X1,X2);
    cout << "Estimated X1:" << endl;
    cout << X1.linear() << endl << X1.translation() << endl;
    cout << endl;
    cout << "Estimated X2:" << endl;
    cout << X2.linear() << endl << X2.translation() << endl;
    cout << endl;
    X_est=estimateTransform(camera_matrix,img1_points,img2_points);
    cout << "Estimated transformation:" << endl;
    cout << X_est.linear() << endl << X_est.translation() << endl;
    cout << endl;
    cout << "Ground truth/estimated transformation ratio:" << endl;
    for (int i=0;i<3;i++){
        for (int j=0;j<4;j++)
            cout << relative_X(i,j)/X_est(i,j) << " ";
        cout << endl;
    }
    return 0;
}