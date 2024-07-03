

#ifndef FRAMEDRAWER_H
#define FRAMEDRAWER_H

#include "Tracking.h"
#include "MapPoint.h"
#include "MapLine.h"
#include "Atlas.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include<mutex>
#include <unordered_set>


namespace ORB_SLAM3
{

class Tracking;
class Viewer;

class FrameDrawer
{
public:
    FrameDrawer(Atlas* pAtlas);

    // Update info from the last processed frame.
    void Update(Tracking *pTracker);

    // Draw last processed frame.
    cv::Mat DrawFrame(bool bOldFeatures=true);
    // myline 画线函数
    cv::Mat DrawFrameWithLines(bool bOldFeatures=true);
    cv::Mat DrawRightFrame();

    //mylk  显示光流场函数
    void visualizeOpticalFlow( cv::Mat& image, const std::vector<cv::Point2f>& points1, const std::vector<cv::Point2f>& points2, const std::vector<uchar>& status);
    void visualizeDynaOpticalFlow(cv::Mat& image, const std::vector<cv::Point2f>& points);
    std::vector<cv::Rect2i> mvDynamicArea;//IMP 第四次修改

    bool both;
    Viewer* mpViewer;

protected:

    void DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText);
    //myline 画线函数
    void DrawTextInfoWithLines(cv::Mat &im, int nState, cv::Mat &imText);

    // Info of the frame to be drawn
    cv::Mat mIm, mImRight;
    int N, N_l;
    vector<cv::KeyPoint> mvCurrentKeys,mvCurrentKeysRight;
    vector<cv::line_descriptor::KeyLine> mvCurrentKeys_l;
    vector<bool> mvbMap, mvbVO, mvbMap_l, mvbVO_l;
    bool mbOnlyTracking;
    int mnTracked, mnTrackedVO, mnTracked_l, mnTrackedVO_l;
    vector<cv::KeyPoint> mvIniKeys;
    vector<int> mvIniMatches;
    int mState;

    Atlas* mpAtlas;

    std::mutex mMutex;
    vector<pair<cv::Point2f, cv::Point2f> > mvTracks;

    Frame mCurrentFrame;
    vector<MapPoint*> mvpLocalMap;
    vector<cv::KeyPoint> mvMatchedKeys;
    vector<MapPoint*> mvpMatchedMPs;
    vector<cv::KeyPoint> mvOutlierKeys;
    vector<MapPoint*> mvpOutlierMPs;

    map<long unsigned int, cv::Point2f> mmProjectPoints;
    map<long unsigned int, cv::Point2f> mmMatchedInImage;

    ///mylk 新增定义
    std::vector<cv::Point2f> F_prepoint, F_nextpoint;
    std::vector<uchar> State;
    std::vector<cv::Point2f> T_M_dyna;
    ///


};

} //namespace ORB_SLAM

#endif // FRAMEDRAWER_H
