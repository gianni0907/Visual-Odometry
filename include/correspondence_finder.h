#pragma once
#include "kdtree.h"

namespace pr{
    template <typename Container1Type_, typename Container2Type_>
    class CorrespondenceFinder{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        using Container1Type = Container1Type_;
        using Container2Type = Container2Type_;
        using TreeNodeType = typename Container1Type::iterator;
        using PointType = typename TreeNodeType::value_type;
        // ctor
        CorrespondenceFinder();

        void init(const Container1Type& reference_points,
                  int max_points_in_leaf,
                  float radius);

        void compute(const Container2Type& current_points);

    inline const IntPairVector& correspondences() const {return _correspondences;}
    inline float maxDistance() const {return _radius;}
    protected:
        float _radius;
        int _max_points_in_leaf;
        TreeNodeType _kdtree;
        const Container1Type* _reference_points;
        IntPairVector _correspondences;
    };
}