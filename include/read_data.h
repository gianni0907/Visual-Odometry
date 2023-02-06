#pragma once
#include <iostream>
#include <fstream>
#include <cstdlib>
#include "defs.h"
#include "camera.h"

namespace pr{

    //return a camera object with parameters specified in camera.dat file
    Camera getCamera();

    //read the ground truth trajectory from trajectory.dat file
    Vector3fVector getGroundTruthTrajectory();

    //read the ground truth world points from world.dat file
    Points3dVector getWorld();
}

