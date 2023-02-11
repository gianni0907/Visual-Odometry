#pragma once
#include "defs.h"

namespace pr{
    template <typename IteratorType_>
    int bruteForceSearch(std::vector<typename IteratorType_::value_type*>& answers,
                         IteratorType_ begin,
                         IteratorType_ end,
                         const typename IteratorType_::value_type& query,
                         const float norm) {
        const float squared_norm = norm*norm;
        int matches=0;
        for (auto it=begin; it!=end; ++it) {
            auto& p=*(it.p);
            if ((p-query.p).squaredNorm()<squared_norm) {
                answers.push_back(&p);
                ++matches;
            }
        }
        return matches;
    }

    template <typename IteratorType_>
    typename IteratorType_::value_type*
    bruteForceBestMatch(IteratorType_ begin,
                        IteratorType_ end,
                        const typename IteratorType_::value_type& query,
                        const float norm) {
        using PointType = typename IteratorType_::value_type;
        const float squared_norm = norm*norm;
        PointType* best=0;
        float best_squared_norm=norm*norm;
        for (auto it=begin; it!=end; ++it) {
            auto& p=*(it.p);
            float squared_distance  = (p-query.p).squaredNorm();
            if ( squared_distance < best_squared_norm) {
                best=&p;
                best_squared_norm = squared_distance;
            }
        }
        return best;
    }

    template <typename Iterator_>
    int computeMeanAndCovariance(Eigen::Matrix<float,
                                               Iterator_::value_type::RowsAtCompileTime, 1>& mean,
                                 Eigen::Matrix<float,
                                               Iterator_::value_type::RowsAtCompileTime,
                                               Iterator_::value_type::RowsAtCompileTime>& cov,
                                 Iterator_ begin,
                                 Iterator_ end) {
        // mean computed as 1/(end-start) Sum_k={start..end} x_k
        // cov computed as  (1/(end-start) Sum_k={start..end} x_k* x_k^T ) - mean*mean.transpose();
        mean.setZero();
        cov.setZero();
        int k=0;
        for (auto it=begin; it!=end; ++it) {
            const auto& v=*(it.p);
            mean += v;
            cov  += v * v.transpose();
            ++k;
        }
        mean *= (1./k);
        cov  *= (1./k);
        cov  -= mean * mean.transpose();
        cov  *= k/(k-1);
        return k;
    }

    // split functions
    // takes two iterators (begin and end)
    // and reorders the data so that the items for which the predicate
    // is true are at the beginning, tho others at the end
    // it returns the iterator to the splitting element between the two classes
    template <typename IteratorType_, typename PredicateType_>
    IteratorType_ split(IteratorType_ begin,
                        IteratorType_ end,
                        PredicateType_ predicate) {
        using ValueType = typename IteratorType_::value_type;
        auto lower=begin;  //first
        // upper is an iterator at the end, it is a reverse iterator,
        // and moves backward when incremented;
        // we apply the predicate to each point,
        // and if the result is positive
        //   we leave the point where it is
        // otherwise
        //   we move the point in the other extrema, and increment the extrema
        // the iterations end when the upper and lower ends match
        auto upper=make_reverse_iterator(end); 
        while (lower!=upper.base()) {
            ValueType& v_lower=*lower;
            ValueType& v_upper=*upper;
            if ( predicate(v_lower) ){
                ++lower;
            } else {
                std::swap(v_lower,v_upper);
                ++upper;
            }
        }
        return upper.base();
    }
}
