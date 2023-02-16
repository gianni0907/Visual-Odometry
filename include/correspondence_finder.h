#pragma once
#include "kdtree.h"

namespace vo{

    /**
        Correspondence finder class based on kdtree.
        To use it:
        - create an instance of the object
        - initialize it passing:
            - reference points on which create the kdtree
            - maximum number of points in a leaf
            - ball radius
        - create correspondences using compute() function, passing the
          set of points for which find the corresponding reference
        - retrieve the correspondences found using correspondences() function
     */
    template <typename Container1Type_, typename Container2Type_>
    class CorrespondenceFinder{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        using Container1Type = Container1Type_;
        using Container2Type = Container2Type_;
        /// ctor
        CorrespondenceFinder(){
            _radius=0;
            _max_points_in_leaf=10;
        }

        /// init method, construct the kdtree on the passed reference_points
        /// @param reference_points: can be a vector of Point3d or Point2d
        /// @param max_points_in_leaf: maximum number of points in a leaf
        /// @param radius: the radius of the hypersphere where the search is performed
        void init(const Container1Type& reference_points,
                  int max_points_in_leaf,
                  float radius){
            _radius=radius;
            _max_points_in_leaf=max_points_in_leaf;
            _reference_points.resize(reference_points.size());
            for (size_t i=0;i<reference_points.size();i++)
                _reference_points[i]=Point(i,reference_points[i].appearance);
            
            _kdtree = TreeNode_(_reference_points.begin(),_reference_points.end(),_max_points_in_leaf);
        }


        /// compute theh associations for the points passed as input
        /// @param current_points: points to compute correspondences.
        /// Can be a vector of Point3d or Point2d 
        void compute(const Container2Type& current_points){
            _correspondences.resize(current_points.size());
            int num_correspondences=0;
            for (size_t i=0; i<current_points.size();i++){
                Point p=Point(i,current_points[i].appearance);
                Point* ref_point=_kdtree.bestMatchFast(p,_radius);
                if (ref_point && !(current_points[i].p.size()==2 && current_points[i].p.x()<0)){
                    _correspondences[num_correspondences].first=(*ref_point).id;
                    _correspondences[num_correspondences].second=p.id;
                    num_correspondences++;
                }
            }
            _correspondences.resize(num_correspondences);
        }


        /// @return: the vector of correspondences computed by the finder 
        inline const IntPairVector& correspondences() const {return _correspondences;}
        
        inline float maxDistance() const {return _radius;}
    protected:
        float _radius;
        int _max_points_in_leaf;
        TreeNode_<PointsVector::iterator> _kdtree;
        PointsVector _reference_points;
        IntPairVector _correspondences;
    };
}