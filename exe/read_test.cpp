#include "camera.h"
#include "read_data.h"

using namespace std;
using namespace pr;

int main (int args, char** argv){
    Camera cam;
    cam=getCamera();
    Eigen::Vector2f dimension;
    dimension=cam.getDimension();
    cout << dimension(0) << "," << dimension(1) << endl;
}