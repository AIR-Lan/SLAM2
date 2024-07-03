//
// Created by lzh on 10/07/23.
//

#ifndef ORB_SLAM3_PROBABILITYMAPPING_H
#define ORB_SLAM3_PROBABILITYMAPPING_H

#include <cstdlib>
#include <stdio.h>
#include <string.h>
#include <vector>
#include <numeric>
#include <Eigen/Core>
#include <mutex>

#include "opencv2/core/core.hpp"

#include"LineExtractor.h"
#include "Atlas.h"
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include "LineDetector.h"
#include "Thirdparty/EDTest/EDLib.h"




// parameters for semidense mapping
#define covisN 7
#define sigmaI 20
#define lambdaG 8//8
#define lambdaL 80
#define lambdaTheta 45  // 45
#define lambdaN 3
#define histo_length 30
#define th_high 100
#define th_low 50
#define THETA 0.23
#define NNRATIO 0.6
#define NULL_DEPTH 999


namespace ORB_SLAM3 {
    class KeyFrame;
    class Map;
    class Atlas;
}

namespace cv {
    class Mat;
}

namespace ORB_SLAM3 {

    class Lineextractor;
    class Modeler;
// thread for semi-dense mapping: compute depth for pixels on keyframes
    class ProbabilityMapping {
    public:

        struct depthHo {
            depthHo() : depth(0.0), sigma(0.0), supported(false), Pw(0.0, 0.0, 0.0) {};
            float depth;
            float sigma;
            bool supported;
            Eigen::Vector3f Pw; // point pose in world frame
        };

        ProbabilityMapping(ORB_SLAM3::Atlas *pAtlas);

        // main function for the thread
        void Run();

        // loop for each keyframe
        void SemiDenseLoop();

        void TestSemiDenseViewer();

        void StereoSearchConstraints(ORB_SLAM3::KeyFrame *kf, float *min_depth, float *max_depth);

        void EpipolarSearch(ORB_SLAM3::KeyFrame *kf1, ORB_SLAM3::KeyFrame *kf2, const int x, const int y, float pixel,
                            float min_depth, float max_depth, depthHo *dh, cv::Mat F12, float &best_u, float &best_v,
                            float th_pi, float rot);

        void GetSearchRange(float &umin, float &umax, int px, int py, float mind, float maxd, ORB_SLAM3::KeyFrame *kf,
                            ORB_SLAM3::KeyFrame *kf2);

        void InverseDepthHypothesisFusion(const std::vector<depthHo> &h, depthHo &dist);

        void IntraKeyFrameDepthChecking(std::vector<std::vector<depthHo> > &ho, int imrows, int imcols);

        void IntraKeyFrameDepthChecking(cv::Mat &depth_map, cv::Mat &depth_sigma, const cv::Mat gradimg);

        void IntraKeyFrameDepthGrowing(cv::Mat &depth_map, cv::Mat &depth_sigma, const cv::Mat gradimg);

        void UpdateSemiDensePointSet(ORB_SLAM3::KeyFrame *kf);

        void UpdateAllSemiDensePointSet();

        void ApplySigmaThreshold(ORB_SLAM3::KeyFrame *kf);

        void SaveSemiDensePoints();

        void WriteModel();

        void InterKeyFrameDepthChecking(const cv::Mat &im, ORB_SLAM3::KeyFrame *currentKF,
                                        std::vector<std::vector<depthHo> > &h);

        void InterKeyFrameDepthChecking(ORB_SLAM3::KeyFrame *currentKf, std::vector<ORB_SLAM3::KeyFrame *> neighbors);

        void RequestFinish();

        bool CheckFinish();

        void SetFinish();

        bool isFinished();

        void RequestReset();

        void ResetIfRequested();

        Modeler *GetModeler();

        std::mutex mMutexSemiDense;

    private:
        bool mbFinishRequested;
        bool mbFinished;
        std::mutex mMutexFinish;

        bool mbResetRequested;
        std::mutex mMutexReset;

        ORB_SLAM3::Map *mpMap;
        ORB_SLAM3::Atlas *mpAtlas;

        ORB_SLAM3::Modeler *mpModeler;

//    LineDetector mLineDetector; Lineextractor* Lineextractor
//mynote 这样值是创建一个指向类的指针没有被调用,需要用new来调用，
// 但是它们的区别在于内存管理和生命周期。
        LineDetector mLineDetector;

        // helper functions, some are not used
        void GetTR(ORB_SLAM3::KeyFrame *kf, cv::Mat *t, cv::Mat *r);

        void GetXp(const cv::Mat &K, int x, int y, cv::Mat *Xp);

        void GetParameterization(const cv::Mat &F12, const int x, const int y, float &a, float &b, float &c);

        void ComputeInvDepthHypothesis(ORB_SLAM3::KeyFrame *kf, ORB_SLAM3::KeyFrame *kf2, float ustar, float ustar_var,
                                       float a, float b, float c, depthHo *dh, int x, int y);

        void GetGradientMagAndOri(const cv::Mat &image, cv::Mat *gradx, cv::Mat *grady, cv::Mat *mag, cv::Mat *ori);

        void GetInPlaneRotation(ORB_SLAM3::KeyFrame *k1, ORB_SLAM3::KeyFrame *k2, float *th);

        void PixelNeighborSupport(std::vector<std::vector<depthHo> > H, int x, int y, std::vector<depthHo> &support);

        void PixelNeighborNeighborSupport(std::vector<std::vector<depthHo> > H, int px, int py,
                                          std::vector<std::vector<depthHo> > &support);

        void GetIntensityGradient_D(const cv::Mat &ImGrad, float a, float b, float c, int px, float *q);

        void GetPixelDepth(float uj, int px, int py, ORB_SLAM3::KeyFrame *kf, ORB_SLAM3::KeyFrame *kf2, float &p);

        void
        GetPixelDepth(float uj, float vj, int px, int py, ORB_SLAM3::KeyFrame *kf, ORB_SLAM3::KeyFrame *kf2, float &p,
                      depthHo *dh);

        bool ChiTest(const depthHo &ha, const depthHo &hb, float *chi_val);

        bool ChiTest(const float &a, const float &b, const float sigma_a, float sigma_b);

        void GetFusion(const std::vector<std::pair<float, float> > supported, float &depth, float &sigma);

        void GetFusion(const std::vector<depthHo> &best_compatible_ho, depthHo &hypothesis, float *min_sigma);

        void Equation14(depthHo &dHjn, float &depthp, cv::Mat &xp, cv::Mat &rji, cv::Mat &tji, float *res);

        cv::Mat ComputeFundamental(ORB_SLAM3::KeyFrame *&pKF1, ORB_SLAM3::KeyFrame *&pKF2);

        cv::Mat GetSkewSymmetricMatrix(const cv::Mat &v);

        std::vector<float> GetRotInPlane(ORB_SLAM3::KeyFrame *kf1, ORB_SLAM3::KeyFrame *kf2);


    };
}
#endif //ORB_SLAM3_PROBABILITYMAPPING_H
