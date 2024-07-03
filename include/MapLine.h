
#ifndef MAPLINE_H
#define MAPLINE_H

#include"KeyFrame.h"
#include"Frame.h"
#include"Map.h"

#include<opencv2/core/core.hpp>
#include<mutex>
#include <map>

#include <eigen3/Eigen/Core>
using namespace Eigen;

typedef Eigen::Matrix<double,6,6> Matrix6d;
typedef Eigen::Matrix<double,6,1> Vector6d;

namespace ORB_SLAM3
{

class KeyFrame;
class Map;
class Frame;

class MapLine
{
public:
    MapLine(const Eigen::Vector3d &sP, const Eigen::Vector3d &eP, Map* pMap);
    MapLine(const Eigen::Vector3d &sP, const Eigen::Vector3d &eP, KeyFrame* pRefKF, Map* pMap);
    MapLine(const Eigen::Vector3d &sP, const Eigen::Vector3d &eP,  Map* pMap, Frame* pFrame, const int &idxF);

    void SetWorldPos(const Eigen::Vector3d &sP, const Eigen::Vector3d &eP);
    Vector6d GetWorldPos();

    cv::Mat GetNormal();

    KeyFrame* GetReferenceKeyFrame();

    std::map<KeyFrame*,size_t> GetObservations();
    int Observations();

    void AddObservation(KeyFrame* pKF,size_t idx);
    void EraseObservation(KeyFrame* pKF);

    int GetIndexInKeyFrame(KeyFrame* pKF);
    bool IsInKeyFrame(KeyFrame* pKF);

    void SetBadFlag();
    bool isBad();

    void Replace(MapLine* pML);    
    MapLine* GetReplaced();

    void IncreaseVisible(int n=1);
    void IncreaseFound(int n=1);
    float GetFoundRatio();
    inline int GetFound(){
        return mnFound;
    }
    void ComputeDistinctiveDescriptors();

    cv::Mat GetDescriptor();

    void UpdateNormalAndDepth();
    void SetNormalVector(cv::Mat& normal);

    void UpdateMap(Map* pMap);

    float GetMinDistanceInvariance();
    float GetMaxDistanceInvariance();

    KeyFrame* SetReferenceKeyFrame(KeyFrame* RFKF);

    Map* GetMap();

public:
    long unsigned int mnId;
    static long unsigned int nNextId;
    long int mnFirstKFid;
    long int mnFirstFrame;
    int nObs;

    // Variables used by loop closing
    long unsigned int mnCorrectedByKF;
    long unsigned int mnCorrectedReference;    

    static std::mutex mGlobalMutex;

     // Position in absolute coordinates
     Eigen::Vector3d mWorldPos_sP;
     Eigen::Vector3d mWorldPos_eP;

     // Tracking
     bool mbTrackInView;
     float mTrackProjsX;
     float mTrackProjsY;
     float mTrackProjeX;
     float mTrackProjeY;
     double mnTrackangle;
     long unsigned int mnTrackReferenceForFrame;
     long unsigned int mnLastFrameSeen;

    // Variables used by local mapping
    long unsigned int mnBALocalForKF;

    // Used for Loop Closing
    Vector6d mPosGBA;
    long unsigned int mnBAGlobalForKF;
    long unsigned int mnBALocalForMerge;

    // Variable used by merging
    Vector6d mPosMerge;
    cv::Mat mNormalVectorMerge;
    
protected:    

     // Keyframes observing the line and associated index in keyframe
     std::map<KeyFrame*,size_t> mObservations;

    // Mean viewing direction
     cv::Mat mNormalVector;

     // Best descriptor to fast matching
     cv::Mat mDescriptor;

     // Reference KeyFrame
     KeyFrame* mpRefKF;

     // Tracking counters
     int mnVisible;
     int mnFound;

     // Bad flag (we do not currently erase MapPoint from memory)
     bool mbBad;
     MapLine* mpReplaced;

     // Scale invariance distances
     float mfMinDistance;
     float mfMaxDistance;

     Map* mpMap;

     std::mutex mMutexMap;
     std::mutex mMutexPos;
     std::mutex mMutexFeatures;
};

} //namespace ORB_SLAM

#endif // MAPLINE_H
