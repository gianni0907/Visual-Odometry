#include "epipolar.h"

namespace pr{
    using namespace std;

    /// @brief 
    /// @param img1_points 
    /// @param img2_points 
    /// @param img1_matching_points 
    /// @param img2_matching_points 
    /// @return 
    int pruneUnmatchingProjectedPoints(const Vec2fVector& img1_points,
                                       const Vec2fVector& img2_points,
                                       Vec2fVector& img1_matching_points,
                                       Vec2fVector& img2_matching_points){
        if (img1_points.size() != img2_points.size()){
            cout << "Different number of points in the two images" << endl;
            return -1;
        }
        int num_matching_points=0;
        img1_matching_points.resize(img1_points.size());
        img2_matching_points.resize(img2_points.size());
        const Eigen::Vector2f point_outside(-1,-1);
        for(size_t i=0; i<img1_points.size(); i++){
            if(img1_points[i].x()>0 && img2_points[i].x()>0 && img1_points[i].y()>0 && img2_points[i].y()>0){
                img1_matching_points[num_matching_points] << img1_points[i].x(),img1_points[i].y();
                img2_matching_points[num_matching_points] << img2_points[i].x(),img2_points[i].y();
                num_matching_points++;
            }
        }
        img1_matching_points.resize(num_matching_points);
        img2_matching_points.resize(num_matching_points);
        return num_matching_points;
    }

    bool triangulatePoint(const Eigen::Vector3f& o2,
                          const Eigen::Vector3f& d1,
                          const Eigen::Vector3f& d2,
                          Eigen::Vector3f& point,
                          float& error){
        Matrix3_2f dir_mat;
        Matrix2_3f dir_mat_t;
        Eigen::Vector3f triangulated_p1,triangulated_p2;
        Eigen::Vector2f s;
        dir_mat << -d1.x(), d2.x(),
                   -d1.y(), d2.y(),
                   -d1.z(), d2.z();
        dir_mat_t=dir_mat.transpose();
        s=-(dir_mat_t*dir_mat).inverse()*(dir_mat_t*o2);
        if (s(0)<0 || s(1)<0){
            cerr << "Point behind one of the two cameras" << endl;
            return false;
        }
        triangulated_p1=d1*s(0);
        triangulated_p2=d2*s(1)+o2;
        error=(triangulated_p1-triangulated_p2).norm();
        point=0.5*(triangulated_p1+triangulated_p2);
        return true;
    }

    int triangulatePoints(const Eigen::Matrix3f& K,
                           const Eigen::Isometry3f& X,
                           const Vec2fVector& img1_points,
                           const Vec2fVector& img2_points,
                           Vec3fVector& points,
                           std::vector<float>& errors){
        Eigen::Isometry3f iX=X.inverse();
        Eigen::Matrix3f iK=K.inverse();
        Eigen::Matrix3f iRiK=iX.linear()*iK;
        Eigen::Vector3f o2=iX.translation();
        Vec2fVector img1_matching_points,img2_matching_points;
        Eigen::Vector3f p1_cam,p2_cam,img1_3dpoint,img2_3dpoint;
        int num_success=0;
        bool success=false;

        //first select only points projected in both images
        img1_matching_points.resize(img1_points.size());
        img2_matching_points.resize(img2_points.size());
        points.resize(img1_points.size());
        errors.resize(img1_points.size());
        pruneUnmatchingProjectedPoints(img1_points,
                                       img2_points,
                                       img1_matching_points,
                                       img2_matching_points);

        //express the points in the 1st camera coordinates (i.e. the world)
        //and apply the triangulation
        for (size_t i=0; i<img1_matching_points.size(); i++){         
            img1_3dpoint << img1_matching_points[i].x(),
                            img1_matching_points[i].y(),
                            1;
            img2_3dpoint << img2_matching_points[i].x(),
                            img2_matching_points[i].y(),
                            1;
            p1_cam=iK*img1_3dpoint;
            p2_cam=iRiK*img2_3dpoint;
            success=triangulatePoint(o2,p1_cam,p2_cam,points[num_success],errors[num_success]);
            if (success)
                num_success++;
        }
        points.resize(num_success);
        errors.resize(num_success);
        return num_success;
    }

    void essential2transform(const Eigen::Matrix3f& E,
                             Eigen::Isometry3f& X1,
                             Eigen::Isometry3f& X2){
        Eigen::Matrix3f W,R1,R2,skew_t1,skew_t2;
        Eigen::Vector3f t1,t2;
        W << 0, -1, 0,
             1, 0, 0,
             0, 0, 1;
        Eigen::JacobiSVD<Eigen::Matrix3f> svd(E, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::JacobiSVD<Eigen::Matrix3f> svd_m(-E, Eigen::ComputeFullU | Eigen::ComputeFullV);
        R1=svd.matrixV()*W*svd.matrixU().transpose();
        R2=svd.matrixV()*W.transpose()*svd.matrixU().transpose();
        if (R1.determinant()<0){
            R1=svd_m.matrixV()*W*svd_m.matrixU().transpose();
            R2=svd_m.matrixV()*W.transpose()*svd_m.matrixU().transpose();
        }
        skew_t1=R1*E;
        skew_t2=R2*E;
        std::cout << skew_t1 << std::endl;
        std::cout << skew_t2 << std::endl;
        X1.linear()=R1;
        X1.translation() << skew_t1(2,1)-skew_t1(1,2),
                            skew_t1(0,2)-skew_t1(2,0),
                            skew_t1(1,0)-skew_t1(0,1);
        
        X2.linear()=R2;
        X2.translation() << skew_t2(2,1)-skew_t2(1,2),
                            skew_t2(0,2)-skew_t2(2,0),
                            skew_t2(1,0)-skew_t2(0,1);
    }

    const Eigen::Matrix3f transform2essential(const Eigen::Isometry3f& X){
        Eigen::Matrix3f R;
        Eigen::Matrix3f skew_t;
        Eigen::Vector3f t;
        R=X.linear();
        t=X.translation();
        skew_t << 0, -t(2), t(1),
                  t(2), 0, -t(0),
                  -t(1), t(0), 0;
        return R.transpose()*skew_t;
    }


}