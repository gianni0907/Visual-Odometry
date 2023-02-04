#include "camera.h"
#include "points_utils.h"

using namespace std;
using namespace pr;

int main (int argc, char** argv){
    
    Points3dVector world_points;
    Points2dVector img_points;
    int num_points=1000;
    Eigen::Vector3f lower_left_bottom(-10,-10,-10);
    Eigen::Vector3f upper_right_top(10,10,10);
    int num_projected_points=0;

    // generate 3d points in the world
    makeWorld(world_points,
              lower_left_bottom,
              upper_right_top,
              num_points);

    cout << "Generated model with " << world_points.size() << " points" << endl;

    // instantiate the camera
    Eigen::Matrix3f camera_matrix;
    camera_matrix << 180, 0, 320,
                     0, 180, 240,
                     0, 0, 1;
    int width=640;
    int height=480;
    float z_near=0;
    float z_far=5;

    Camera cam(height, width, z_near, z_far, camera_matrix);

    // test projection on image plane
    char key=0;
    const char ESC_key=27;
    while (key!=ESC_key){
        num_projected_points=cam.projectPoints(img_points, world_points, false);
        cout << "Number of projected points: " << num_projected_points << endl;
        RGBImage shown_image(height,width);
        shown_image=cv::Vec3b(255,255,255);
        drawPoints(shown_image, img_points, cv::Scalar(255,0,0),3);
        cv::imshow("cam_test", shown_image);
        Eigen::Isometry3f current_pose=cam.worldInCameraPose();
        key=cv::waitKey(0);
    }
    for (int i=0; i< num_projected_points; i++)
        cout << img_points[i].p << endl;

    return 0;
}
