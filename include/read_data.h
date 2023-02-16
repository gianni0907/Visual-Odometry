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

    /// @param dataset: the dataset folder path 
    /// @return: a camera object with parameters specified in camera.dat file
    Camera getCamera(std::filesystem::path& dataset);

    /// @param dataset: the dataset folder path 
    /// @return: the ground truth trajectory (x-y-theta) read from trajectory.dat file
    Vector3fVector getGroundTruthTrajectory(std::filesystem::path& dataset);

    /// @param dataset: the dataset folder path 
    /// @return: the ground truth world points read from world.dat file
    Points3dVector getWorld(std::filesystem::path& dataset);

    //read 

    /// @param dataset: the dataset folder path
    /// @return: all the observations read from all meas-XXXXX.dat files 
    ObsVector getObservations(std::filesystem::path& dataset);

    /// @return: the estimated trajectory (x-y-z) 
    Vector3fVector getEstimatedTrajectory();

    /// @return: the vector of estimated transformations expressing the robot in world
    TransfVector getEstimatedPoses();

    /// @return: the estimated world points
    Points3dVector getEstimatedWorld();
}

