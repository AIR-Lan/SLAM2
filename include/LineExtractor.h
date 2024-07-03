

#ifndef LINEEXTRACTOR_H
#define LINEEXTRACTOR_H

#include <vector>
#include <list>
#include <opencv/cv.h>
#include <mutex>
#include <cstdlib>
#include <stdio.h>
#include <string.h>
#include <numeric>
#include "opencv2/core/core.hpp"

#include <line_descriptor_custom.hpp>
#include <line_descriptor/descriptor_custom.hpp>
#include "KeyFrame.h"
//myplan
#include "Thirdparty/EDLines/LS.h"
#include <Eigen/Core>
#include "Thirdparty/EDTest/EDLib.h"
#include "Thirdparty/EDTest/EdgeMap.h"
#include "line3D.h"
#include "Modeler.h"



using namespace cv;
using namespace line_descriptor;

//class Cluster {
//public:
//    Cluster() {
//        segments = cv::Mat::zeros(0, 6, CV_32F);
//        centerSegment = cv::Mat::zeros(1, 6, CV_32F);
//        index = 0;
//    }
//    unsigned long index;
//    cv::Mat segments;
//    cv::Mat centerSegment;
//};

namespace ORB_SLAM3
{

struct sort_lines_by_response
{
    inline bool operator()(const KeyLine& a, const KeyLine& b){
        return ( a.response > b.response );
    }
};

struct sort_lines_by_length
{
    inline bool operator()(const KeyLine& a, const KeyLine& b){
        return ( a.lineLength > b.lineLength );
    }
};
class KeyFrame;
class Viewer;
class Modeler;
class Lineextractor
{
public:
    Lineextractor(int _lsd_nfeatures, int _lsd_refine, float _lsd_scale, int _nlevels, float _scale, int _extractor);

    Lineextractor();

    ~Lineextractor(){}

    void operator()( const cv::Mat& image, const cv::Mat& mask,
      std::vector<cv::line_descriptor::KeyLine>& keylines,
      cv::Mat& descriptors_line);

    // Images on the pyramid
    std::vector<cv::Mat> mvImagePyramid_l;
    std::vector<float> mvScaleFactor_l;//尺度图像金字塔的缩放因子
    std::vector<float> mvInvScaleFactor_l;
    std::vector<float> mvLevelSigma2_l;
    std::vector<float> mvInvLevelSigma2_l;//逆级别方差的倒数
    int nlevels_l;

    std::vector<cv::Rect2i> mvDynamicArea;//change 添加这一行
    //myplan
    std::string GetStringDateTime();
    void RunLine3Dpp(std::vector<KeyFrame*> vpKFs);
    void LineFittingEDLinesOffline(std::vector<ORB_SLAM3::KeyFrame*> vpKFs);
    // run edge-aided line segment extraction (offline)
    void LineFittingOffline(std::vector<ORB_SLAM3::KeyFrame*> vpKFs, Modeler* pModeler);
    // save line segments as obj files
    void SaveAllLineSegments();
    void SaveClusteredSegments();
    void Summary();// save time usage of methods in log file
    std::vector<double> time_modeling;
//    void DetectEdgeMap(ORB_SLAM3::KeyFrame* kf); // edge detection by EdgeDrawing
    void DetectLineSegments(ORB_SLAM3::KeyFrame* kf);//// 2D line segment detection by EDLines
    void Reset();
    void ClosestPointOnLine(float a, float b, float c, int x, int y, float& cx, float& cy);
    void LineFitting(ORB_SLAM3::KeyFrame* kf);
    void MergeLines(ORB_SLAM3::KeyFrame* kf, Modeler* pModeler);
    std::string CurrentDateTime();
    void LineFit(Pixel *pixelChain, int noPixels, ORB_SLAM3::KeyFrame* kf);
    bool CLoseEnough(Cluster c, cv::Mat l);
    void UpdateCluster(Cluster& c);
    int CountDepth(Pixel *pixelChain, int length, ORB_SLAM3::KeyFrame* kf);
    void LeastSquaresLineFit(Pixel* pixelChain, int initLength, float& u1, float& u2, float& u3, float& lineFitError);
    void LeastSquaresDepthFit(Pixel* pixelChain, int initLength, float la, float lb, float lc, float& u1, float& u2, float& depthFitError, ORB_SLAM3::KeyFrame* kf);
    float ComputePointDistance2Line(float a, float b, float c, Pixel pixel);
    float ComputePointDepth2Line(float a, float b, float c, float alpha, float beta, Pixel *pixelChain, Pixel pixel, ORB_SLAM3::KeyFrame* kf);

protected:
    //myplan
    std::string mStrDateTime;
    std::vector<Cluster> clusters;
    std::vector<double> time_decoupled;
    cv::Mat mAllLines;
    std::vector<double> time_line3d;
    std::vector<double> time_edge;
    std::vector<double> time_edlines;
    std::vector<ORB_SLAM3::KeyFrame *> vpKFs_global;
    std::vector<double> time_fitting;
    std::vector<double> time_merging;


    // filtering after extraction
    int    lsd_nfeatures;
    double min_line_length;

    // lines detection
    int    lsd_refine;
    float  lsd_scale;
    double lsd_sigma_scale;
    double lsd_quant;
    double lsd_ang_th;
    double lsd_log_eps;
    double lsd_density_th;
    int    lsd_n_bins;
    int    nlevels;
    float  scale;
    int    extractor;
    Viewer* mpViewer;
};

} //namespace ORB_SLAM

#endif

