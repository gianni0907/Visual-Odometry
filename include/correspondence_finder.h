#pragma once
#include "kdtree.h"

namespace pr{
    template <typename Container1Type_, typename Container2Type_>
    class CorrespondenceFinder{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        using Container1Type = Container1Type_;
        using Container2Type = Container2Type_;
        // ctor
        CorrespondenceFinder(){
            _radius=0;
            _max_points_in_leaf=10;
        }

        void init(const Container1Type& reference_points,
                  int max_points_in_leaf,
                  float radius){
            _radius=radius;
            _max_points_in_leaf=max_points_in_leaf;
            _reference_points.resize(reference_points.size());
            for (size_t i=0;i<reference_points.size();i++)
                _reference_points[i]=Point(reference_points[i].id,reference_points[i].appearance);
            
            _kdtree = TreeNode_(_reference_points.begin(),_reference_points.end(),_max_points_in_leaf);
        }

        void compute(const Container2Type& current_points){
            _correspondences.resize(current_points.size());
            int num_correspondences=0;
            for (auto& curr_p: current_points){
                Point p=Point(curr_p.id,curr_p.appearance);
                Point* ref_point=_kdtree.bestMatchFast(p,_radius);
                if (ref_point){
                    _correspondences[num_correspondences].first=(*ref_point).id;
                    _correspondences[num_correspondences].second=p.id;
                    num_correspondences++;
                }
            }
            _correspondences.resize(num_correspondences);
        }

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