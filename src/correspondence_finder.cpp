#include "correspondence_finder.h"

namespace pr{
    template <typename Container1Type_, typename Container2Type_>
    CorrespondenceFinder<Container1Type_,Container2Type_>::CorrespondenceFinder(){
        _radius=0;
        _max_points_in_leaf=10;
        _reference_points=0;
    }

    template <typename Container1Type_, typename Container2Type_>
    void CorrespondenceFinder<Container1Type_,Container2Type_>::init(const Container1Type& reference_points,
                                    int max_points_in_leaf,
                                    float radius){
        _radius=radius;
        _max_points_in_leaf=max_points_in_leaf;
        _reference_points=&reference_points;

        _kdtree= TreeNodeType((*_reference_points).begin(),(*_reference_points).end(),_max_points_in_leaf);
    }

    template <typename Container1Type_, typename Container2Type_>
    void CorrespondenceFinder<Container1Type_,Container2Type_>::compute(const Container2Type& current_points){
        _correspondences.resize(current_points.size());
        int num_correspondences=0;
        for (auto& p: current_points){
            PointType* ref_point=_kdtree.bestMatchFast(p,_radius);
            if (!ref_point){
                _correspondences[num_correspondences].first=(*ref_point).id;
                _correspondences[num_correspondences].second=p.id;
                num_correspondences++;
            }
        }
        _correspondences.resize(num_correspondences);
    }
}