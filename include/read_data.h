#pragma once
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <string>
#include "defs.h"
#include "camera.h"
#define N_POINTS 1000
#define N_POSES 121

namespace pr{

    //return a camera object with parameters specified in camera.dat file
    Camera getCamera();

    //read the ground truth trajectory from trajectory.dat file
    Vector3fVector getGroundTruthTrajectory();

    //read the ground truth world points from world.dat file
    Points3dVector getWorld();

    //read all meas-XXXXX.dat files and store observations in a vector
    ObsVector getObservations();
}

