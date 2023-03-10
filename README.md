# Visual Odometry
Given as input the camera parameters and an image sequence (observations), the aim of the project is to estimate the robot trajectory and the position of fixed 3d points in world (landmarks).

## Data Structures
### Points
In order to properly handle all the informations related to world points (3d) or projected points (2d), I create the following structs:
- *Point3d*, with fields: *id* (int), *p* (3d float vector storing the point coordinates), *appearance* (10d float vector)
- *Point2d*, with fields: *id* (int), *p* (2d float vector storing the point coordinates), *appearance* (10d float vector)

Note that in both cases the *id* field is stored only for completeness, it is never used in the project.
### Observations
Moreover, data related to each image of the input sequence are stored in the following struct:
- *Observations*, with fields: 
    - *gt_pose* (3d float vector storing ground truth x-y position and theta angle)
    - *points* (vector of Points2d elements, containing all the image points related to the observation)

Also in this struct, the *gt_pose* is not used in the project, it is considered only for evaluation.
### Camera parameters
The camera parameters given in input are stored as attributes of a *Camera* object. They are:
- Camera matrix (*3x3* matrix)
- Pose of the world in camera frame (*Isometry3f*)
- Spatial dimension (*width*,*height*)
- Depth range (*z_near*,*z_far*)

Furthermore, I add to the given implementation of Camera class also the *Isometry3f* which represents the pose of the camera in the robot frame.

## Data Association
To find a correspondence between two points, their dimension is not relevant: they can be both 2d points (i.e projected on image planes), or both 3d points, or also mixed. Indeed, what matters are their appearances, so we need to compare them.

Since a point appearance is characterized by a 10-dimensional vector, to compute data association I rely on KD-Trees, because they are more suitable when the points dimension increases. In order to change the least possible the available implementation of KD-Tree I create an additional struct:
- *Point*, with fields: *id* (int), *appearance* (10d float vector)

Notice that loosing the information about point coordinates - useless in this context - I can treat in the same way both 2d and 3d points.  
Moreover, here the *id* field has a different meaning wrt *Points2d* and *Points3d* structs. In fact, here it no longer represents the unique point identifier, but the index of the point in the considered vector of points. It is necessary to explicitly store it since the KD-Tree machinery does not preserve the vector ordering.

Finally, I decided to create another class *CorrespondenceFinder* which has as attribute the object of *TreeNode_* class - instead of instantiate an object of *TreeNode_* class in a function which returns the correspondences. This is done in order to reuse for multiple searches the same precomputed KD_Tree structure, avoiding - whenever it is possible - the creation of a new one.

## Initialization
In the code I refer to pose of *camera(i)* to express the pose of the camera corresponding to the *i-th* image observed.

Initialization step consists in the estimation of the transformation expressing the *camera(0)* in *camera(1)* frame.
This is done using epipolar geometry on the first two observations (images) of the sequence.

Notice that in the estimation of the fundamental matrix two transformations are used to normalize image points within [-1,1] and avoid numerical instability.

Once obtained the estimated transformation (correct up to a scale), I use it (actually its inverse) to triangulate the points in the first two images of the sequence, obtaining the first set of 3d points expressed in *camera(1)* frame.

## Projective ICP 
Then I can start using the projective ICP machinery iteratively. At the *i-th* iteration of PICP the estimation of the pose of *camera(i)* in *camera(i+1)* frame is obtained.
At the end of the *i-th* PICP iteration:
- update the estimated robot pose (using the transformation expressing the camera pose in robot frame)
- triangulate the points on images *(i)* and *(i+1)*, expressing them in *camera(i+1)* frame
- merge the new triangulated points with the old ones, in order to use all of them for the next iteration.
## Compilation
- Clone the repository on github:
```
git clone https://github.com/gianni0907/prob_rob_proj <directory>
```
- On the system console, go in `<directory>/prob_rob_proj` folder and execute build sequence:
```
mkdir build
cd build
cmake ..
make
```
## Execution
The following 8 binaries are created in the `<directory>/prob_rob_proj/build/exe` folder.
The first 4 are realized on synthetic data:
- `./cam_test`: to test projection function of the implemented pinhole camera model
- `./epipolar_test`: to test epipolar geometry functions
- `./picp_test`: to test the least squares solver
- `./complete_test`: to test the sequence of registration step via epipolar geometry and then first projective icp run

Then, the remaining 4 binaries treat real data:
- `./read_test`: to test the reading of real data from files
- `./kdtree_test`: to test the correspondence finding algorithm based on KD-Trees
- `./vis_odom_test`: complete visual odometry program
- `./evaluation_test`: to evaluate the estimated quantities, comparing with ground truth values

NOTE: please run them from the `build/` directory, to correctly save files - whenever they are created - in `<directory>/prob_rob_proj/estimation` and `<directory>/prob_rob_proj/errors` folders

## Metrics
In the `./evaluation_test` executable the following metrics values are computed.
### Rotation and translation errors
The metric to evaluate the robot pose estimation is given by the following rotation and translation errors.

Given two consecutive robot poses in world $T_0,T_1$
- compute the relative transformation 
```math 
\textbf{T}_{rel}=\textbf{T}^{-1}_{0}\textbf{T}_{1}
```
- compute also the relative ground truth transformation 
```math
\textbf{T}_{rel,GT}=\textbf{T}^{-1}_{0,GT}\textbf{T}_{1,GT}
```
- compute the $SE(3)$ error: 
```math
\textbf{T}_{e}=\textbf{T}^{-1}_{rel}\textbf{T}_{rel,GT}
```

Then, consider two separate errors for rotational and translational parts:
```math
e_{\theta}=trace(\textbf{I}_{3}-\textbf{T}_{e}(1:3,1:3))
```
```math
e_t=\frac{||\textbf{T}_{rel}(1:3,4)||}{||\textbf{T}_{rel,GT}(1:3,4)||}
```

Actually, it is necessary to compute the translational error two times.

The first time the computation is done considering the estimated robot poses as they have been computed.

Then, I rescale the whole estimated trajectory using the estimated ratio *2.1433* (first of the translation errors[^1] just computed) and I recompute the translation errors considering the scaled estimated trajectory.

Note that the scaling only affects the translational part, hence it is sufficient to compute rotation errors once.
### RMSE
Moreover, another metric to compare the scaled version of the estimated trajectory with the ground truth trajectory is used: the *RMSE*.

The same metric is also used to compare the estimated map with the ground truth map, considering only the corresponding points
- use the estimated ratio *2.1433*  to scale the estimated points
- find correspondences between estimated points and real world points
- compute the *RMSE*

In the following table the two computed *RMSE* values
|*RMSE trajectory*|*RMSE map*|
|-----------------|----------|
|0.135233|0.203287|

[^1]: I use the first translation error because it corresponds to the ratio between the estimated translation with epipolar geometry and the ground truth one.

### Plots
It is possible to plot some quantities to compare ground truth data vs estimated ones.

To plot them on gnuplot, go in `<directory>/prob_rob_proj` folder, open gnuplot and run the command below each image.

![Ground truth vs estimated Trajectory](plots/gt_vs_est_traj.png)

```
set zrange [-0.1:0.1]
splot "estimation/gt_trajectory.dat" u 1:2:3 w lp pt 7 ps 0.5 t "gt trajectory", "estimation/est_trajectory.dat" u 1:2:3 w lp pt 7 ps 0.5 t "est trajectory"
```

![Ground truth vs scaled estimated Trajectory](plots/gt_vs_scaledest_traj.png)

```
set zrange [-0.1:0.1]
splot "estimation/gt_trajectory.dat" u 1:2:3 w lp pt 7 ps 0.5 t "gt trajectory", "estimation/est_trajectory.dat" u 1:2:3 w lp pt 7 ps 0.5 t "est trajectory"
```

![ground truth world points vs scaled estimated corresponding points](plots/gt_vs_est_world.png)

```
unset zrange
splot "estimation/gt_points_pruned.dat" u 2:3:4  pt 7 ps 0.7 t "ground truth points", "estimation/est_points_scaled.dat" u 2:3:4 pt 7 ps 0.7 t "scaled estimated points","estimation/correspondences.dat" u 1:2:3:($4-$1):($5-$2):($6-$3) with vectors nohead t "correspondences"
```

![rotation and translation errors (scaled and not)](plots/errors.png)

```
plot "errors/rotation_err.dat" t "rotation error" w lp pt 7 ps 0.5, "errors/translation_err.dat" t "translation error" w lp pt 7 ps 0.5, "errors/translation_err_scaled.dat" t "translation error scaled" w lp pt 7 ps 0.5
```