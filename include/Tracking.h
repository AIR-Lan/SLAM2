


#ifndef TRACKING_H
#define TRACKING_H

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>

#include"Viewer.h"
#include"FrameDrawer.h"
#include"Atlas.h"
#include"LocalMapping.h"
#include"LoopClosing.h"
#include"Frame.h"
#include "ORBVocabulary.h"
#include"KeyFrameDatabase.h"
#include"ORBextractor.h"
#include"LineExtractor.h"
#include "Initializer.h"
#include "MapDrawer.h"
#include "System.h"
#include "ImuTypes.h"

#include "GeometricCamera.h"

#include <mutex>
#include <unordered_set>
#include "YOLOV5.h" //change 添加这个

#include "utils.h"

// cube slam.
#include "detect_3d_cuboid/matrix_utils.h"
//#include <line_lbd/line_descriptor.hpp>
//#include <line_lbd/line_lbd_allclass.h>
#include "landmark_plane.h"

namespace ORB_SLAM3
{

class Viewer;
class FrameDrawer;
class Atlas;
class LocalMapping;
class LoopClosing;
class System;
class ProbabilityMapping;
//myplan
class Planar_Mapping_module;
class MapDrawer;
class Plane;

class Tracking
{  

public:

    //myline 添加线辞典参数的tracking构造类
    Tracking(System* pSys, ORBVocabulary* pVoc, LineVocabulary* lVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Atlas* pAtlas,
             KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor, const string &_nameSeq=std::string());

    Tracking(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Atlas* pAtlas,
             KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor, const string &_nameSeq=std::string());

    ~Tracking();
    bool isYoloObject = false;//change 添加这一行
    // Parse the config file
    bool ParseCamParamFile(cv::FileStorage &fSettings);
    bool ParseORBParamFile(cv::FileStorage &fSettings);
    bool ParseLineParamFile(cv::FileStorage &fSettings);
    bool ParseIMUParamFile(cv::FileStorage &fSettings);

    // Preprocess the input and call Track(). Extract features and performs stereo matching.
    cv::Mat GrabImageStereo(const cv::Mat &imRectLeft,const cv::Mat &imRectRight, const double &timestamp, string filename);
    cv::Mat GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const cv::Mat &img_seg_mask,const double &timestamp, string filename);
    cv::Mat GrabImageMonocular(const cv::Mat &im, const double &timestamp, string filename);
    // cv::Mat GrabImageImuMonocular(const cv::Mat &im, const double &timestamp);

    void GrabImuData(const IMU::Point &imuMeasurement);

    //myplan
    void SetSemiDenseMapping(ProbabilityMapping* pSemiDenseMapping);
    void AssociateObjAndPoints(vector<Object_2D *> objs_2d);//为什么这个link到keyframe不明白
    // [EAO] associate objects with lines.
    void AssociateObjAndLines(vector<Object_2D *> objs_2d);
    void InitObjMap(vector<Object_2D *> objs_2d);
    void SampleObjYaw(Object_Map* objMap);// [EAO] sample object yaw.
    // [EAO] project quadrics to the image.
    cv::Mat DrawQuadricProject( cv::Mat &im,
                                const cv::Mat &P,
                                const cv::Mat &axe,
                                const cv::Mat &Twq,
                                int nClassid,
                                bool isGT=true,
                                int nLatitudeNum = 7,
                                int nLongitudeNum = 6);
    // [EAO] project points from world to image.
    cv::Point2f WorldToImg(cv::Mat &PointPosWorld);
    void printMatrix(const std::vector<std::vector<double>>& matrix);
    void get_plane_norm();


    bool mbObjectIni = false;
    int mnObjectIniFrameID;



    void SetLocalMapper(LocalMapping* pLocalMapper);
    void SetLoopClosing(LoopClosing* pLoopClosing);
    void SetViewer(Viewer* pViewer);
    void SetDetector(YOLOV5* pDetector);///change 添加这个
    void SetStepByStep(bool bSet);
    //myplan
    void set_planar_mapping_module(Planar_Mapping_module *planar_mapper);

    // Load new settings
    // The focal lenght should be similar or scale prediction will fail when projecting points
    void ChangeCalibration(const string &strSettingPath);

    // Use this function if you have deactivated local mapping and you only want to localize the camera.
    void InformOnlyTracking(const bool &flag);

    void UpdateFrameIMU(const float s, const IMU::Bias &b, KeyFrame* pCurrentKeyFrame);
    KeyFrame* GetLastKeyFrame()
    {
        return mpLastKeyFrame;
    }

    void CreateMapInAtlas();
    std::mutex mMutexTracks;

    //--
    void NewDataset();
    int GetNumberDataset();
    int GetMatchesInliers();
public:

    // Tracking states
    enum eTrackingState{
        SYSTEM_NOT_READY=-1,
        NO_IMAGES_YET=0,
        NOT_INITIALIZED=1,
        OK=2,
        RECENTLY_LOST=3,
        LOST=4,
        OK_KLT=5
    };

    eTrackingState mState;
    eTrackingState mLastProcessedState;

    // Input sensor
    int mSensor;

    // Current Frame
    Frame mCurrentFrame;
    Frame mLastFrame;//myplan  相当于mInitialSecendFrame;

    cv::Mat mImGray;
    ///mylk 新增变量
    cv::Mat mImS;
    cv::Mat mImS_C;
    cv::Mat mImRGB;// adding for color point map  by zoe
    cv::Mat mImDepth; // adding mImDepth member to realize pointcloud view
    ///
    //myplan
    cv::Mat _img_seg_mask;

    // Initialization Variables (Monocular)
    std::vector<int> mvIniLastMatches;
    std::vector<int> mvIniMatches;
    std::vector<cv::Point2f> mvbPrevMatched;
    std::vector<cv::Point3f> mvIniP3D;
    Frame mInitialFrame;

    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    list<cv::Mat> mlRelativeFramePoses;
    list<KeyFrame*> mlpReferences;
    list<double> mlFrameTimes;
    list<bool> mlbLost;


    // frames with estimated pose
    int mTrackedFr;
    bool mbStep;

    // True if local mapping is deactivated and we are performing only localization
    bool mbOnlyTracking;

    void Reset(bool bLocMap = false);
    void ResetActiveMap(bool bLocMap = false);

    float mMeanTrack;
    bool mbInitWith3KFs;
    double t0; // time-stamp of first read frame
    double t0vis; // time-stamp of first inserted keyframe
    double t0IMU; // time-stamp of IMU initialization


    vector<MapPoint*> GetLocalMapMPS();


    //TEST--
    bool mbNeedRectify;
    //cv::Mat M1l, M2l;
    //cv::Mat M1r, M2r;

    bool mbWriteStats;

    // Parameter for Choosing Pose Optimization and Local BA
    //myline 位姿优化传递的参数进行不同优化
    int SLAM;
    //MYPLAN
    //groundtruth.
    static bool mbReadedGroundtruth;
    string mflag;// demo.

protected:

    // Main tracking function. It is independent of the input sensor.
    void Track();

    // Main tracking function for stereo with Lines. it is independent of the input sensor. 
    void TrackWithLines(const double &timestamp);

    //myplan
    ProbabilityMapping* mpSemiDenseMapping;

    // Map initialization for stereo and RGB-D
    void StereoInitialization();
    void StereoInitializationWithLines();

    // Map initialization for monocular
    void MonocularInitialization();
    void CreateNewMapPoints();
    cv::Mat ComputeF12(KeyFrame *&pKF1, KeyFrame *&pKF2);
    void CreateInitialMapMonocular();

    void CheckReplacedInLastFrame();
    void CheckReplacedInLastFrameWithLines();
    bool TrackReferenceKeyFrame();
    bool TrackReferenceKeyFrameWithLines();
    void UpdateLastFrame();
    void UpdateLastFrameWithLines();
    bool TrackWithMotionModel();
    bool TrackWithMotionModelWithLines(const double &timestamp);
    bool PredictStateIMU();

    bool Relocalization();

    void UpdateLocalMap();
    //myline 更新线和关键帧函数
    void UpdateLocalMapWithLines();
    void UpdateLocalPoints();
    void UpdateLocalPointsAndLines();
    void UpdateLocalKeyFrames();
    void UpdateLocalKeyFramesWithLines();

    bool TrackLocalMap();
    bool TrackLocalMapWithLines();
    bool TrackLocalMap_old();
    void SearchLocalPoints();
    void SearchLocalPointsAndLines();

    bool NeedNewKeyFrame();
    int NeedNewKeyFrameWithLines();
    bool NeedNewKeyFrameWithRGBD();
    void CreateNewKeyFrame();
    void CreateNewKeyFrameWithLines(bool CreateByObjs);

    // Perform preintegration from last frame
    void PreintegrateIMU();

    // Reset IMU biases and compute frame velocity
    void ResetFrameIMU();
    void ComputeGyroBias(const vector<Frame*> &vpFs, float &bwx,  float &bwy, float &bwz);
    void ComputeVelocitiesAccBias(const vector<Frame*> &vpFs, float &bax,  float &bay, float &baz);


    bool mbMapUpdated;
    //MYPLAN
    vector<vector<double>> mGroundtruth_mat;    // camera groundtruth.


    // Imu preintegration from last frame
    IMU::Preintegrated *mpImuPreintegratedFromLastKF;

    // Queue of IMU measurements between frames
    std::list<IMU::Point> mlQueueImuData;

    // Vector of IMU measurements from previous to current frame (to be filled by PreintegrateIMU)
    std::vector<IMU::Point> mvImuFromLastFrame;
    std::mutex mMutexImuQueue;

    // Imu calibration parameters
    IMU::Calib *mpImuCalib;

    // Last Bias Estimation (at keyframe creation)
    IMU::Bias mLastBias;

    // In case of performing only localization, this flag is true when there are no matches to
    // points in the map. Still tracking will continue if there are enough matches with temporal points.
    // In that case we are doing visual odometry. The system will try to do relocalization to recover
    // "zero-drift" localization to the map.
    bool mbVO;

    //Other Thread Pointers
    LocalMapping* mpLocalMapper;
    LoopClosing* mpLoopClosing;

    //ORB
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;
    ORBextractor* mpIniORBextractor;

    //Line Feature myline 左右目线特征提取器
    Lineextractor* mpLineextractorLeft, *mpLineextractorRight;

    //BoW
    ORBVocabulary* mpORBVocabulary;
    LineVocabulary* mpLineVocabulary;//myline 传递tracking中的线辞典
    KeyFrameDatabase* mpKeyFrameDB;

    // Initalization (only for monocular)
    Initializer* mpInitializer;
    bool mbSetInit;

    //Local Map
    KeyFrame* mpReferenceKF;
    std::vector<KeyFrame*> mvpLocalKeyFrames;
    std::vector<MapPoint*> mvpLocalMapPoints;
    std::vector<MapLine*> mvpLocalMapLines;
    
    // System
    System* mpSystem;
    
    //Drawers
    Viewer* mpViewer;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;
    bool bStepByStep;

    //Atlas
    Atlas* mpAtlas;

    //Calibration matrix
    cv::Mat mK;
    cv::Mat mDistCoef;
    float mbf;

    //New KeyFrame rules (according to fps)
    int mMinFrames;
    int mMaxFrames;

    int mnFirstImuFrameId;
    int mnFramesToResetIMU;

    // Threshold close/far points
    // Points seen as close by the stereo/RGBD sensor are considered reliable
    // and inserted from just one frame. Far points requiere a match in two keyframes.
    float mThDepth;

    // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
    float mDepthMapFactor;

    //Current matches in frame
    //myline 匹配线数量
    int mnMatchesInliers;
    int mnMatchesInliers_l;
    YOLOV5* mpDetector;///change 添加这个

    //myplan
    Planar_Mapping_module *_planar_mapper = nullptr;

    std::vector<Vector3d> t_all_plane_normals;

    //Last Frame, KeyFrame and Relocalisation Info
    KeyFrame* mpLastKeyFrame;
    unsigned int mnLastKeyFrameId;
    unsigned int mnLastRelocFrameId;
    double mTimeStampLost;
    double time_recently_lost;

    unsigned int mnFirstFrameId;
    unsigned int mnInitialFrameId;
    unsigned int mnLastInitFrameId;

    bool mbCreatedMap;


    //Motion Model
    cv::Mat mVelocity;

    //Color order (true RGB, false BGR, ignored if grayscale)
    bool mbRGB;

    list<MapPoint*> mlpTemporalPoints;
    list<MapLine*> mlpTemporalLines;

    //int nMapChangeIndex;

    int mnNumDataset;

    ofstream f_track_stats;

    ofstream f_track_times;
    double mTime_PreIntIMU;
    double mTime_PosePred;
    double mTime_LocalMapTrack;
    double mTime_NewKF_Dec;

    GeometricCamera* mpCamera, *mpCamera2;

    int initID, lastID;

    cv::Mat mTlr;

public:
    cv::Mat mImRight;
    vector<Detection> mmDetectMap; //mylk 目前不知道干啥的
};

} //namespace ORB_SLAM

#endif // TRACKING_H
