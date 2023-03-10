#pragma once
#include "defs.h"
#include "points_utils.h"
#include "camera.h"
#include "correspondence_finder.h"
#include "epipolar.h"

namespace vo {

  /**
     Solver for point-to-camera problem.
     Implements a least squares solver using translation+euler angles as minimal parameterization
     A simple saturating robust kernel, and an adjustable damping coefficient;
     To use it:
     - create an object
     - initialize it passing:
       - A camera whose pose is initialized at initial_guess
       - the world points (that represent the model)
       - the image points (that represent the measurements)
       - the number of iterations for ICP
       - the boolean to specify if the indices are kept or not
     - call run() function to run the PICP machinery
   */
  class PICPSolver{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    //! ctor
    PICPSolver();

    //! init method, call it at the beginning
    //! @param camera: the camera
    //! @param world_points: the points of the world
    //! @param image_points: the points of the reference
    /// @param num_iterations: number of iterations of ICP procedure
    /// @param keep_indices: boolean to specify if the indices are kept or not
    void init(const Camera& camera,
	            const Points3dVector& world_points,
	            const Points2dVector& image_points,
              const int& num_iterations,
              const bool& keep_indices);
  
    inline float kernelThreshold() const {return _kernel_thereshold;}

    inline void setKernelThreshold(float kernel_threshold) 
    {_kernel_thereshold=kernel_threshold;}


  
    //! accessor to the camera
    const Camera& camera() const {return _camera;}

    //! chi square of the "good" points
    const float chiInliers() const {return _chi_inliers;}
    
    //! chi square of the "bad" points
    const float chiOutliers() const {return _chi_outliers;}
    
    //! number of inliers (an inlier is a point whose error is below kernel threshold)
    const int numInliers() const {return _num_inliers;}
    
    //! performs one iteration of optimization
    //! @param correspondences: the correspondences (first: measurement, second:model);
    //! @param keep_outliers: if true, the outliers are considered in the optimization 
    //! (but cut by the kernel)
    bool oneRound(const IntPairVector& correspondences, bool keep_outliers);

    void run();

  protected:

    bool errorAndJacobian(Eigen::Vector2f& error,
			  Matrix2_6f& jacobian,
			  const Eigen::Vector3f& world_point,
			  const Eigen::Vector2f& reference_image_point);

    void linearize(const IntPairVector& correspondences, bool keep_outliers);

			       
    Camera _camera;                  //< this will hold our state
    float _kernel_thereshold;        //< threshold for the kernel
    float _damping;                  //< damping, to slow the solution
    int _min_num_inliers;            //< if less inliers than this value, the solver stops
    const Points3dVector* _world_points;
    const Points2dVector* _reference_image_points;
    Matrix6f _H;
    Vector6f _b;
    float _chi_inliers;
    float _chi_outliers;
    int _num_inliers;
    int _num_iterations;
    bool _keep_indices;
  };

}