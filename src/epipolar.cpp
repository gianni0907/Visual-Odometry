#include "epipolar.h"

namespace pr{
    using namespace std;

    int pruneUnmatchingProjectedPoints(const Points2dVector& img1_points,
                                       const Points2dVector& img2_points,
                                       Points2dVector& img1_matching_points,
                                       Points2dVector& img2_matching_points){
        if (img1_points.size() != img2_points.size()){
            cout << "Different number of points in the two images" << endl;
            return -1;
        }
        int num_matching_points=0;
        img1_matching_points.resize(img1_points.size());
        img2_matching_points.resize(img2_points.size());
        const Eigen::Vector2f point_outside(-1,-1);
        for(size_t i=0; i<img1_points.size(); i++){
            if(img1_points[i].p!=point_outside && img2_points[i].p!=point_outside){
                img1_matching_points[num_matching_points]=img1_points[i];
                img2_matching_points[num_matching_points]=img2_points[i];
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
            // cerr << "Point behind one of the two cameras" << endl;
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
                           const Points2dVector& img1_points,
                           const Points2dVector& img2_points,
                           Points3dVector& points,
                           std::vector<float>& errors){
        Eigen::Isometry3f iX=X.inverse();
        Eigen::Matrix3f iK=K.inverse();
        Eigen::Matrix3f iRiK=iX.linear()*iK;
        Eigen::Vector3f o2=iX.translation();
        Points2dVector img1_matching_points,img2_matching_points;
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
            img1_3dpoint << img1_matching_points[i].p.x(),
                            img1_matching_points[i].p.y(),
                            1;
            img2_3dpoint << img2_matching_points[i].p.x(),
                            img2_matching_points[i].p.y(),
                            1;
            p1_cam=iK*img1_3dpoint;
            p2_cam=iRiK*img2_3dpoint;
            success=triangulatePoint(o2,p1_cam,p2_cam,points[num_success].p,errors[num_success]);
            if (success){
                points[num_success].id=img1_matching_points[i].id;
                points[num_success].appearance=img1_matching_points[i].appearance;
                num_success++;
            }
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
        skew_t=skew(t);
        return R.transpose()*skew_t;
    }

    void fundamental2transform(const Eigen::Matrix3f& K,
                               const Eigen::Matrix3f& F,
                               Eigen::Isometry3f& X1,
                               Eigen::Isometry3f& X2){
        Eigen::Matrix3f E;
        E=K.transpose()*F*K;
        essential2transform(E,X1,X2);
    }

    const Eigen::Matrix3f transform2fundamental(const Eigen::Matrix3f& K,
                                                const Eigen::Isometry3f& X){
        Eigen::Matrix3f iK=K.inverse();
        return iK.transpose()*transform2essential(X)*iK;
    }

    const Eigen::Matrix3f normTransform(const Points2dVector& img_points){
        Eigen::Matrix3f T;
        T.setZero();
        float x_max,y_max;
        x_max=img_points[0].p.x();
        y_max=img_points[0].p.y();
        for (size_t i=1; i<img_points.size();i++){
            if (img_points[i].p.x()>x_max)
                x_max=img_points[i].p.x();
            if (img_points[i].p.y()>y_max)
                y_max=img_points[i].p.y();
        }
        T << 2/x_max, 0, -1,
             0, -2/y_max, 1,
             0, 0, 1;
        return T;
    }

    const Eigen::Matrix3f estimateFundamental(const Points2dVector& img1_points,
                                              const Points2dVector& img2_points){
        size_t n_points=img1_points.size();
        Eigen::Matrix<float, 9, 9> H;
        H.setZero();
        Eigen::Matrix3f tmp_mat,Fa,F,sing_values,T1,T2;
        Vector9f A,eig_vec;
        Eigen::Vector3f img1_3dpoint,img2_3dpoint;
        T1=normTransform(img1_points);
        T2=normTransform(img2_points);
        for (size_t i=0; i<n_points; i++){
            img1_3dpoint << img1_points[i].p.x(), img1_points[i].p.y(), 1;
            img2_3dpoint << img2_points[i].p.x(), img2_points[i].p.y(), 1;
            tmp_mat=(T1*img1_3dpoint)*(T2*img2_3dpoint).transpose();
            A << tmp_mat(0,0), tmp_mat(1,0), tmp_mat(2,0),
                 tmp_mat(0,1), tmp_mat(1,1), tmp_mat(2,1),
                 tmp_mat(0,2), tmp_mat(1,2), tmp_mat(2,2);
            H+=A*A.transpose();
        }
        eig_vec=smallestEigenVector(H);
        Fa << eig_vec(0), eig_vec(3), eig_vec(6),
              eig_vec(1), eig_vec(4), eig_vec(7),
              eig_vec(2), eig_vec(5), eig_vec(8);
        Fa=T1.transpose()*Fa*T2;
        Eigen::JacobiSVD<Eigen::Matrix3f> svd(Fa, Eigen::ComputeFullU | Eigen::ComputeFullV);
        sing_values=svd.singularValues().asDiagonal();
        sing_values(2,2)=0;
        F=svd.matrixU()*sing_values*svd.matrixV().transpose();

        return F;
    }

    const Eigen::Isometry3f estimateTransform(const Eigen::Matrix3f& K,
                                              const Points2dVector& img1_points,
                                              const Points2dVector& img2_points){
        Eigen::Matrix3f F;
        Eigen::Isometry3f X1,X2,X_tmp,X;
        Points3dVector points;
        std::vector<float> errors;
        int num_points=0;
        int num_points_tmp=0;
        F=estimateFundamental(img1_points,img2_points);
        fundamental2transform(K,F,X1,X2);
        X=X1;
        X_tmp=X1;
        num_points_tmp=triangulatePoints(K,X_tmp,
                                        img1_points,img2_points,
                                        points,errors);
        if (num_points_tmp>num_points){
            num_points=num_points_tmp;
            X=X_tmp;
        }
        X_tmp.translation()=-X_tmp.translation();
        num_points_tmp=triangulatePoints(K,X_tmp,
                                        img1_points,img2_points,
                                        points,errors);
        if (num_points_tmp>num_points){
            num_points=num_points_tmp;
            X=X_tmp;
        }
        X_tmp=X2;
        num_points_tmp=triangulatePoints(K,X_tmp,
                                        img1_points,img2_points,
                                        points,errors);
        if (num_points_tmp>num_points){
            num_points=num_points_tmp;
            X=X_tmp;
        }
        X_tmp.translation()=-X_tmp.translation();
        num_points_tmp=triangulatePoints(K,X_tmp,
                                        img1_points,img2_points,
                                        points,errors);
        if (num_points_tmp>num_points){
            num_points=num_points_tmp;
            X=X_tmp;
        }
        return X;
    }
}