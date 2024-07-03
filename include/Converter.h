


#ifndef CONVERTER_H
#define CONVERTER_H

#include<opencv2/core/core.hpp>

#include<Eigen/Dense>
#include"Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include"Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace ORB_SLAM3
{

class Converter
{
public:
    // NOTE [EAO-SLAM]
    //myplan
    static Eigen::MatrixXd toEigenMatrixXd(const cv::Mat &cvMat);
    static float bboxOverlapratioLatter(const cv::Rect& rect1, const cv::Rect& rect2);
    static float bboxOverlapratio(const cv::Rect& rect1, const cv::Rect& rect2);
    static float bboxOverlapratioFormer(const cv::Rect& rect1, const cv::Rect& rect2);


    static std::vector<cv::Mat> toDescriptorVector(const cv::Mat &Descriptors);
    static g2o::SE3Quat toSE3Quat(const cv::Mat &cvT);
    static g2o::SE3Quat toSE3Quat(const g2o::Sim3 &gSim3);
    static cv::Mat toCvMat(const g2o::SE3Quat &SE3);
    static cv::Mat toCvMat(const g2o::Sim3 &Sim3);
    static cv::Mat toCvMat(const Eigen::Matrix<double,4,4> &m);
    static cv::Mat toCvMat(const Eigen::Matrix3d &m);
    static cv::Mat toCvMat(const Eigen::Matrix<double,3,1> &m);
    static cv::Mat toCvMat(const Eigen::MatrixXd &m);

    static cv::Mat toCvSE3(const Eigen::Matrix<double,3,3> &R, const Eigen::Matrix<double,3,1> &t);
    static cv::Mat tocvSkewMatrix(const cv::Mat &v);

    static cv::Mat toInvCvMat(const cv::Mat &T_cvMat);
    static cv::Mat toInvCvMat(const Eigen::Matrix<double,4,4> &T_eigenMtrx);

    static Eigen::Matrix<double,3,1> toVector3d(const cv::Mat &cvVector);
    static Eigen::Matrix<double,3,1> toVector3d(const cv::Point3f &cvPoint);
    static Eigen::Matrix<double,3,3> toMatrix3d(const cv::Mat &cvMat3);
    static Eigen::Matrix<double,4,4> toMatrix4d(const cv::Mat &cvMat4);
    static Eigen::Matrix<double,4,4> toInvMatrix4d(const cv::Mat &T_cvMat);
    static Eigen::Matrix<double,4,4> toInvMatrix4d(const Eigen::Matrix<double,4,4> &T_eigenMtrx);
    static std::vector<float> toQuaternion(const cv::Mat &M);

    static bool isRotationMatrix(const cv::Mat &R);
    static std::vector<float> toEuler(const cv::Mat &R);
    static void RmatOfQuat(cv::Mat &M, const cv::Mat &q);

};

}// namespace ORB_SLAM

#endif // CONVERTER_H
