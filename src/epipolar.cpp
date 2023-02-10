#include "epipolar.h"

namespace pr{
    using namespace std;

    void computeImg2ImgCorrespondences(IntPairVector& correspondences,
				                       const Points2dVector& img1_points,
				                       const Points2dVector& img2_points){
        correspondences.resize(min(img1_points.size(),img2_points.size()));
        int num_correspondences=0;
        Point2d reference_point,current_point;
        for (size_t i=0; i<img1_points.size(); i++){
            reference_point=img1_points[i];
            for (size_t j=0; j<img2_points.size();j++){
                current_point=img2_points[j];
                if (reference_point.appearance==current_point.appearance && !(reference_point.p.x()<0 || current_point.p.x()<0)){
                    correspondences[num_correspondences].first=i;
                    correspondences[num_correspondences].second=j;
                    num_correspondences++;
                    break;
                }
            }
        }
        correspondences.resize(num_correspondences);
    }

    void computeWrld2ImgCorrespondences(IntPairVector& correspondences,
				                        const Points3dVector& world_points,
				                        const Points2dVector& img_points){
        correspondences.resize(min(img_points.size(),world_points.size()));
        int num_correspondences=0;
        Point2d img_point;
        Point3d world_point;
        for (size_t i=0; i<img_points.size(); i++){
            img_point=img_points[i];
            for (size_t j=0; j<world_points.size();j++){
                world_point=world_points[j];
                if (img_point.appearance==world_point.appearance && !(img_point.p.x()<0)){
                    correspondences[num_correspondences].first=j;
                    correspondences[num_correspondences].second=i;
                    num_correspondences++;
                    break;
                }
            }
        }
        correspondences.resize(num_correspondences);
    }

    void computeWrld2WrldCorrespondences(IntPairVector& correspondences,
				                         const Points3dVector& world1_points,
				                         const Points3dVector& world2_points){
        correspondences.resize(min(world1_points.size(),world2_points.size()));
        int num_correspondences=0;
        Point3d world1_point,world2_point;
        for (size_t i=0; i<world1_points.size(); i++){
            world1_point=world1_points[i];
            for (size_t j=0; j<world2_points.size();j++){
                world2_point=world2_points[j];
                if (world1_point.appearance==world2_point.appearance){
                    correspondences[num_correspondences].first=i;
                    correspondences[num_correspondences].second=j;
                    num_correspondences++;
                    break;
                }
            }
        }
        correspondences.resize(num_correspondences);
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
        Eigen::Vector3f p1_cam,p2_cam,img1_3dpoint,img2_3dpoint;
        int num_success=0;
        bool success=false;
        IntPairVector correspondences;

        //compute correspondences between projected points
        computeImg2ImgCorrespondences(correspondences, img1_points, img2_points);
        points.resize(correspondences.size());
        errors.resize(correspondences.size());

        //express the points in the 1st camera coordinates (i.e. the world)
        //and apply the triangulation
        for (size_t i=0; i<correspondences.size(); i++){
            int idx1=correspondences[i].first;
            int idx2=correspondences[i].second;
            img1_3dpoint << img1_points[idx1].p.x(),
                            img1_points[idx1].p.y(),
                            1;
            img2_3dpoint << img2_points[idx2].p.x(),
                            img2_points[idx2].p.y(),
                            1;
            p1_cam=iK*img1_3dpoint;
            p2_cam=iRiK*img2_3dpoint;
            success=triangulatePoint(o2,p1_cam,p2_cam,points[num_success].p,errors[num_success]);
            if (success){
                points[num_success].id=img1_points[idx1].id;
                points[num_success].appearance=0.5*(img1_points[idx1].appearance+img2_points[idx2].appearance);
                num_success++;
            }
        }
        points.resize(num_success);
        errors.resize(num_success);
        return num_success;
    }

    Points3dVector mergePoints(const Eigen::Isometry3f& X,
                               const Points3dVector& new_points,
                               const Points3dVector& points){
        Points3dVector merged_points=new_points;
        size_t num_points=merged_points.size();
        merged_points.resize(new_points.size()+points.size());
        IntPairVector correspondences;
        computeWrld2WrldCorrespondences(correspondences,new_points,points);
        bool inside=false;
        for (size_t i=0;i<points.size();i++){ //TODO: for on old points and see if they are in correspondences,otherwise you add them in merged_points
            inside=false;
            for (const IntPair& corr: correspondences){
                if (static_cast<int>(i)==corr.second){
                    inside=true;
                    break;
                }
            }
            if (!inside){
                merged_points[num_points].p=X*points[i].p;
                merged_points[num_points].id=points[i].id;
                merged_points[num_points].appearance=points[i].appearance;
                num_points++;
            }
        }
        merged_points.resize(num_points);
        return merged_points;    
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
        X1.translation() << skew_t1(2,1),//-skew_t1(1,2),
                            skew_t1(0,2),//-skew_t1(2,0),
                            skew_t1(1,0);//-skew_t1(0,1);
        
        X2.linear()=R2;
        X2.translation() << skew_t2(2,1),//-skew_t2(1,2),
                            skew_t2(0,2),//-skew_t2(2,0),
                            skew_t2(1,0);//-skew_t2(0,1);
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

    const Matrix3fPair normTransform(const IntPairVector& correspondences,
                                     const Points2dVector& img1_points,
                                     const Points2dVector& img2_points){
        Matrix3fPair T;
        T.first.setZero();
        T.second.setZero();
        float x1_max,x2_max,y1_max,y2_max;
        x1_max=img1_points[correspondences[0].first].p.x();
        y1_max=img1_points[correspondences[0].first].p.y();
        x2_max=img2_points[correspondences[0].second].p.x();
        y2_max=img2_points[correspondences[0].second].p.y();
        for (size_t i=1; i<correspondences.size();i++){
            int idx1=correspondences[i].first;
            int idx2=correspondences[i].second;
            if (img1_points[idx1].p.x()>x1_max)
                x1_max=img1_points[idx1].p.x();
            if (img1_points[idx1].p.y()>y1_max)
                y1_max=img1_points[idx1].p.y();
            if (img2_points[idx2].p.x()>x2_max)
                x2_max=img2_points[idx2].p.x();
            if (img2_points[idx2].p.y()>y2_max)
                y2_max=img2_points[idx2].p.y();
        }
        T.first << 2/x1_max, 0, -1,
                0, -2/y1_max, 1,
                0, 0, 1;
        T.second << 2/x2_max, 0, -1,
                0, -2/y2_max, 1,
                0, 0, 1;
        return T;
    }

    const Eigen::Matrix3f estimateFundamental(const Points2dVector& img1_points,
                                              const Points2dVector& img2_points){
        Eigen::Matrix<float, 9, 9> H;
        H.setZero();
        Eigen::Matrix3f tmp_mat,Fa,F,sing_values;
        Matrix3fPair T;
        Vector9f A,eig_vec;
        Eigen::Vector3f img1_3dpoint,img2_3dpoint;
        IntPairVector correspondences;

        //compute correspondences between projected points
        computeImg2ImgCorrespondences(correspondences, img1_points, img2_points);
        size_t n_points=correspondences.size();
        //compute the normalization transform, one for each set of points
        T=normTransform(correspondences,img1_points,img2_points);
        for (size_t i=0; i<n_points; i++){
            img1_3dpoint << img1_points[correspondences[i].first].p.x(),
                            img1_points[correspondences[i].first].p.y(),
                            1;
            img2_3dpoint << img2_points[correspondences[i].second].p.x(),
                            img2_points[correspondences[i].second].p.y(),
                            1;
            tmp_mat=(T.first*img1_3dpoint)*(T.second*img2_3dpoint).transpose();
            A << tmp_mat(0,0), tmp_mat(1,0), tmp_mat(2,0),
                 tmp_mat(0,1), tmp_mat(1,1), tmp_mat(2,1),
                 tmp_mat(0,2), tmp_mat(1,2), tmp_mat(2,2);
            H+=A*A.transpose();
        }
        eig_vec=smallestEigenVector(H);
        Fa << eig_vec(0), eig_vec(3), eig_vec(6),
              eig_vec(1), eig_vec(4), eig_vec(7),
              eig_vec(2), eig_vec(5), eig_vec(8);
        Fa=T.first.transpose()*Fa*T.second;
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
        X1=Eigen::Isometry3f::Identity();
        X2=Eigen::Isometry3f::Identity();
        X_tmp=Eigen::Isometry3f::Identity();
        X=Eigen::Isometry3f::Identity();
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