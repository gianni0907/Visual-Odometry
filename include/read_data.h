#pragma once
#include <iostream>
#include <fstream>
#include <cstdlib>
#include "defs.h"
#include "camera.h"

namespace pr{

    //return a camera object with parameters specified in camera.dat file
    Camera getCamera();

    //read the ground truth trajectory from the file trajecotry.dat
    //return the number of pose in the trajectory
    int getGroundTruthTrajectory(Vector3fVector& trajectory);
}

