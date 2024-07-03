


#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include "KeyFrame.h"
#include "Atlas.h"
#include "LoopClosing.h"
#include "Tracking.h"
#include "KeyFrameDatabase.h"
#include "Initializer.h"

#include <mutex>
#include "planar_mapping_module.h"


namespace ORB_SLAM3
{

class System;
class Tracking;
class LoopClosing;
class Atlas;
//myplan
class Planar_Mapping_module;

class LocalMapping
{
public:
    LocalMapping(System* pSys, Atlas* pAtlas, const float bMonocular, bool bInertial, const string &_strSeqName=std::string());

    void SetLoopCloser(LoopClosing* pLoopCloser);

    void SetTracker(Tracking* pTracker);

    //myplan
//FW: Set the planar mapping module
    void set_planar_mapping_module(Planar_Mapping_module *planar_mapper);
    // FW: planar mapping module
    Planar_Mapping_module *_planar_mapper = nullptr;


    // Main function
    void Run();

    // Main function with lines
    //myline 局部建图主函数
    void Run_Lines();

    //MYPLAN
    // NOTE: [EAO-SLAM]
    void UpdateObject();            // update object.
    void MergePotentialAssObjs();   // merge potentially associated objects.
    void WhetherOverlapObject();    // determine whether two objects overlap.

    string mflag = "Full";

    void InsertKeyFrame(KeyFrame* pKF);
    void EmptyQueue();

    // Thread Synch
    void RequestStop();
    void RequestReset();
    void RequestResetActiveMap(Map* pMap);
    bool Stop();
    void Release();
    bool isStopped();
    bool stopRequested();
    bool AcceptKeyFrames();
    void SetAcceptKeyFrames(bool flag);
    bool SetNotStop(bool flag);

    void InterruptBA();

    void RequestFinish();
    bool isFinished();

    int KeyframesInQueue(){
        unique_lock<std::mutex> lock(mMutexNewKFs);
        return mlNewKeyFrames.size();
    }

    bool IsInitializing();
    double GetCurrKFTime();
    KeyFrame* GetCurrKF();

    std::mutex mMutexImuInit;

    Eigen::MatrixXd mcovInertial;
    Eigen::Matrix3d mRwg;
    Eigen::Vector3d mbg;
    Eigen::Vector3d mba;
    double mScale;
    double mInitTime;
    double mCostTime;
    bool mbNewInit;
    unsigned int mInitSect;
    unsigned int mIdxInit;
    unsigned int mnKFs;
    double mFirstTs;
    int mnMatchesInliers;

    // For debugging (erase in normal mode)
    int mInitFr;
    int mIdxIteration;
    string strSequence;

    bool mbNotBA1;
    bool mbNotBA2;
    bool mbBadImu;

    bool mbWriteStats;

    // not consider far points (clouds)
    //远点近点
    bool mbFarPoints;//标志
    float mThFarPoints;
protected:

    bool CheckNewKeyFrames();
    void ProcessNewKeyFrame();
    //myline 局部地图处理线的函数
    void ProcessNewKeyFrameWithLines();
    void CreateNewMapPoints();

    //myplan
    std::mutex mMutexObject;

    void MapPointCulling();
    void MapLineCulling();
    void SearchInNeighbors();
    void KeyFrameCulling();
    void KeyFrameCullingWithLines();

    cv::Mat ComputeF12(KeyFrame* &pKF1, KeyFrame* &pKF2);

    cv::Mat SkewSymmetricMatrix(const cv::Mat &v);

    System *mpSystem;

    bool mbMonocular;
    bool mbInertial;

    void ResetIfRequested();
    //myline
    void ResetIfRequestedWithLines();
    bool mbResetRequested;
    bool mbResetRequestedActiveMap;
    Map* mpMapToReset;
    std::mutex mMutexReset;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    Atlas* mpAtlas;

    LoopClosing* mpLoopCloser;
    Tracking* mpTracker;

    std::list<KeyFrame*> mlNewKeyFrames;

    KeyFrame* mpCurrentKeyFrame;

    std::list<MapPoint*> mlpRecentAddedMapPoints;
    //myline 线描述子
    std::list<MapLine*> mlpRecentAddedMapLines;

    std::mutex mMutexNewKFs;

    bool mbAbortBA;

    bool mbStopped;
    bool mbStopRequested;
    bool mbNotStop;
    std::mutex mMutexStop;

    bool mbAcceptKeyFrames;
    std::mutex mMutexAccept;

    void InitializeIMU(float priorG = 1e2, float priorA = 1e6, bool bFirst = false);
    void ScaleRefinement();

    bool bInitializing;

    Eigen::MatrixXd infoInertial;
    int mNumLM;
    int mNumKFCulling;

    float mTinit;

    int countRefinement;

    //DEBUG
    ofstream f_lm;
};

} //namespace ORB_SLAM

#endif // LOCALMAPPING_H
