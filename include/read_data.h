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
    Vector3fVector getGroundTruthTrajectory();
}

