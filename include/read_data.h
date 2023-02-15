#pragma once
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <string>
#include "camera.h"
#include "defs.h"

#define N_POINTS 1000
#define N_POSES 121

namespace vo{

    //return a camera object with parameters specified in camera.dat file
    Camera getCamera();

    //read the ground truth trajectory (x-y-theta) from trajectory.dat file
    Vector3fVector getGroundTruthTrajectory();

    //read the ground truth world points from world.dat file
    Points3dVector getWorld();

    //read all meas-XXXXX.dat files and store observations in a vector
    ObsVector getObservations();

    //read the estimated trajectory (x-y-z)
    Vector3fVector getEstimatedTrajectory();

    //read the vector of estimated transformations expressing the robot in world
    TransfVector getEstimatedPoses();

    //read the estimated world points
    Vector3fVector getEstimatedWorld();
}

