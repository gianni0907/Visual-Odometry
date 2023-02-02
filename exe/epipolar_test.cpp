#include "camera.h"
#include "points_utils.h"
#include "epipolar.h"

using namespace std;
using namespace pr;

int main(int argc, char** argv){

    
    // bool success=false;
    // float error=-1;
    // Eigen::Vector3f o2,d1,d2,p;
    // o2 << -1.5, 0, 0;
    // d1 << 1, 1, 2;
    // d2 << 2.5, 1, 2;
    // success=triangulatePoint(o2,d1,d2,p,error);
    // cout << p << endl;


    // generate 3d points in the world
    Vec3fVector world_points,triangulated_world_points;
    vector<float> errors;
    Vec2fVector img1_points, img2_points;
    Vec2fVector img1_matching_points, img2_matching_points;
    Eigen::Vector3f lower_left_bottom(-10,-10,-10);
    Eigen::Vector3f upper_right_top(10,10,10);
    int num_points=100;
    int num_projected_points1=0;
    int num_projected_points2=0;
    int num_matching_points=0;
    int num_triangulated_points=0;
    makeWorld(world_points,
              lower_left_bottom,
              upper_right_top,
              num_points);

    cout << "Generated model with " << world_points.size() << " points" << endl;

    //Consider an homogeneous transformation expressing the cam1(world) wrt cam2
    Eigen::Isometry3f relative_X;
    relative_X=Eigen::Isometry3f::Identity();
    relative_X.translation()=Eigen::Vector3f(-1.5f, -1.5f, 0.0f);
    cout << relative_X.linear() << endl;
    cout << relative_X.translation() << endl;
    // instantiate the camera
    Eigen::Matrix3f camera_matrix;
    camera_matrix << 180, 0, 320,
                     0, 180, 240,
                     0, 0, 1;
    int width=640;
    int height=480;
    float z_near=0;
    float z_far=5;

    Camera cam1(height, width, z_near, z_far, camera_matrix);
    Camera cam2(height, width, z_near, z_far, camera_matrix, relative_X);

    //project the points in the world in both the cameras image planes
    char key=0;
    const char ESC_key=27;
    while (key!=ESC_key){
        num_projected_points1=cam1.projectPoints(img1_points, world_points, true);
        num_projected_points2=cam2.projectPoints(img2_points, world_points, true);
        cout << "Number of projected points on img1: " << num_projected_points1 << endl;
        cout << "Number of projected points on img2: " << num_projected_points2 << endl;        
        RGBImage shown_image1(height,width);
        RGBImage shown_image2(height,width);
        shown_image1=cv::Vec3b(255,255,255);
        shown_image2=cv::Vec3b(255,255,255);
        drawPoints(shown_image1, img1_points, cv::Scalar(255,0,0),3);
        drawPoints(shown_image2, img2_points, cv::Scalar(255,0,0),3);
        cv::imshow("epipolar_test_img1", shown_image1);
        cv::imshow("epipolar_test_img2", shown_image2);
        key=cv::waitKey(0);
    }

    num_matching_points=pruneUnmatchingProjectedPoints(img1_points,
                                                       img2_points,
                                                       img1_matching_points,
                                                       img2_matching_points);

    cout << "Matching projected points in img1 plane:" << endl;
    for (int i=0; i<num_matching_points; i++){
        cout << img1_matching_points[i].x() << ", " << img1_matching_points[i].y() << endl;
    }
    cout << "Matching projected points in img2 plane:" << endl;
    for (int i=0; i<num_matching_points; i++){
        cout << img2_matching_points[i].x() << ", " << img2_matching_points[i].y() << endl;
    }

    cout << "Number of matching points: " << num_matching_points << endl;
    num_triangulated_points=triangulatePoints(camera_matrix,
                                              relative_X,
                                              img1_points,
                                              img2_points,
                                              triangulated_world_points,
                                              errors);
    cout << "Number of triangulated points: " << num_triangulated_points << endl;

    cout << "Triangulated points in world:" << endl;
    for (int i=0; i<num_triangulated_points; i++){
        cout << triangulated_world_points[i].x() << ", " << triangulated_world_points[i].y() << ", " << triangulated_world_points[i].z() << endl;
    }
    Vec2fVector img_points;
    key=0;
    while (key!=ESC_key){
        cam1.projectPoints(img_points, triangulated_world_points, true);
        RGBImage shown_image3(height,width);
        shown_image3=cv::Vec3b(255,255,255);
        drawPoints(shown_image3, img_points, cv::Scalar(255,0,0),3);
        cv::imshow("cam_test", shown_image3);
        key=cv::waitKey(0);
    }
    cout << "Triangulated points in img1 plane:" << endl;
    for (int i=0; i<num_triangulated_points; i++){
        cout << img_points[i].x() << ", " << img_points[i].y() << endl;
    }
}