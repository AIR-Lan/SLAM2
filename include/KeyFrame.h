


#ifndef KEYFRAME_H
#define KEYFRAME_H

#include "MapPoint.h"
#include "MapLine.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "Frame.h"
#include "KeyFrameDatabase.h"
#include "ImuTypes.h"

#include "GeometricCamera.h"

#include <mutex>

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/map.hpp>

#include "ProbabilityMapping.h"

class EdgeMap;
namespace ORB_SLAM3
{

class Map;
class MapPoint;
///myline 添加线继承类
class MapLine;
class Frame;
class KeyFrameDatabase;

class Object_2D;

class GeometricCamera;


class KeyFrame
{


    template<class Archive>
    void serializeMatrix(Archive& ar, cv::Mat& mat, const unsigned int version)
    {
        int cols, rows, type;
        bool continuous;

        if (Archive::is_saving::value) {
            cols = mat.cols; rows = mat.rows; type = mat.type();
            continuous = mat.isContinuous();
        }

        ar & cols & rows & type & continuous;

        if (Archive::is_loading::value)
            mat.create(rows, cols, type);

        if (continuous) {
            const unsigned int data_size = rows * cols * mat.elemSize();
            ar & boost::serialization::make_array(mat.ptr(), data_size);
        } else {
            const unsigned int row_size = cols*mat.elemSize();
            for (int i = 0; i < rows; i++) {
                ar & boost::serialization::make_array(mat.ptr(i), row_size);
            }
        }
    }


    template<class Archive>
    void serializeMatrix(Archive& ar, const cv::Mat& mat, const unsigned int version)
    {
        cv::Mat matAux = mat;

        serializeMatrix(ar, matAux,version);

        if (Archive::is_loading::value)
        {
            cv::Mat* ptr;
            ptr = (cv::Mat*)( &mat );
            *ptr = matAux;
        }
    }

    friend class boost::serialization::access;
    template<class Archive>
    void serializeVectorKeyPoints(Archive& ar, const vector<cv::KeyPoint>& vKP, const unsigned int version)
    {
        int NumEl;

        if (Archive::is_saving::value) {
            NumEl = vKP.size();
        }

        ar & NumEl;

        vector<cv::KeyPoint> vKPaux = vKP;
        if (Archive::is_loading::value)
            vKPaux.reserve(NumEl);

        for(int i=0; i < NumEl; ++i)
        {
            cv::KeyPoint KPi;

            if (Archive::is_loading::value)
                KPi = cv::KeyPoint();

            if (Archive::is_saving::value)
                KPi = vKPaux[i];

            ar & KPi.angle;
            ar & KPi.response;
            ar & KPi.size;
            ar & KPi.pt.x;
            ar & KPi.pt.y;
            ar & KPi.class_id;
            ar & KPi.octave;

            if (Archive::is_loading::value)
                vKPaux.push_back(KPi);
        }


        if (Archive::is_loading::value)
        {
            vector<cv::KeyPoint> *ptr;
            ptr = (vector<cv::KeyPoint>*)( &vKP );
            *ptr = vKPaux;
        }
    }

    template<class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar & mnId;
        ar & const_cast<long unsigned int&>(mnFrameId);
        ar & const_cast<double&>(mTimeStamp);
        // Grid
        ar & const_cast<int&>(mnGridCols);
        ar & const_cast<int&>(mnGridRows);
        ar & const_cast<float&>(mfGridElementWidthInv);
        ar & const_cast<float&>(mfGridElementHeightInv);
        // Variables of tracking
        ar & mnTrackReferenceForFrame;
        ar & mnFuseTargetForKF;
        // Variables of local mapping
        ar & mnBALocalForKF;
        ar & mnBAFixedForKF;
        ar & mnNumberOfOpt;
        // Variables used by KeyFrameDatabase
        ar & mnLoopQuery;
        ar & mnLoopWords;
        ar & mLoopScore;
        ar & mnRelocQuery;
        ar & mnRelocWords;
        ar & mRelocScore;
        ar & mnMergeQuery;
        ar & mnMergeWords;
        ar & mMergeScore;
        ar & mnPlaceRecognitionQuery;
        ar & mnPlaceRecognitionWords;
        ar & mPlaceRecognitionScore;
        ar & mbCurrentPlaceRecognition;
        // Variables of loop closing
        serializeMatrix(ar,mTcwGBA,version);
        serializeMatrix(ar,mTcwBefGBA,version);
        serializeMatrix(ar,mVwbGBA,version);
        serializeMatrix(ar,mVwbBefGBA,version);
        ar & mBiasGBA;
        ar & mnBAGlobalForKF;
        // Variables of Merging
        serializeMatrix(ar,mTcwMerge,version);
        serializeMatrix(ar,mTcwBefMerge,version);
        serializeMatrix(ar,mTwcBefMerge,version);
        serializeMatrix(ar,mVwbMerge,version);
        serializeMatrix(ar,mVwbBefMerge,version);
        ar & mBiasMerge;
        ar & mnMergeCorrectedForKF;
        ar & mnMergeForKF;
        ar & mfScaleMerge;
        ar & mnBALocalForMerge;
        // Scale
        ar & mfScale;
        // Calibration parameters
        ar & const_cast<float&>(fx);
        ar & const_cast<float&>(fy);
        ar & const_cast<float&>(invfx);
        ar & const_cast<float&>(invfy);
        ar & const_cast<float&>(cx);
        ar & const_cast<float&>(cy);
        ar & const_cast<float&>(mbf);
        ar & const_cast<float&>(mb);
        ar & const_cast<float&>(mThDepth);
        serializeMatrix(ar,mDistCoef,version);
        // Number of Keypoints
        ar & const_cast<int&>(N);
        // KeyPoints
        serializeVectorKeyPoints(ar,mvKeys,version);
        serializeVectorKeyPoints(ar,mvKeysUn,version);
        ar & const_cast<vector<float>& >(mvuRight);
        ar & const_cast<vector<float>& >(mvDepth);
        serializeMatrix(ar,mDescriptors,version);
        // BOW
        ar & mBowVec;
        ar & mFeatVec;
        // Pose relative to parent
        serializeMatrix(ar,mTcp,version);
        // Scale
        ar & const_cast<int&>(mnScaleLevels);
        ar & const_cast<float&>(mfScaleFactor);
        ar & const_cast<float&>(mfLogScaleFactor);
        ar & const_cast<vector<float>& >(mvScaleFactors);
        ar & const_cast<vector<float>& >(mvLevelSigma2);
        ar & const_cast<vector<float>& >(mvInvLevelSigma2);
        // Image bounds and calibration
        ar & const_cast<int&>(mnMinX);
        ar & const_cast<int&>(mnMinY);
        ar & const_cast<int&>(mnMaxX);
        ar & const_cast<int&>(mnMaxY);
        serializeMatrix(ar,mK,version);
        // Pose
        serializeMatrix(ar,Tcw,version);
        // MapPointsId associated to keypoints
        ar & mvBackupMapPointsId;
        // Grid
        ar & mGrid;
        // Connected KeyFrameWeight
        ar & mBackupConnectedKeyFrameIdWeights;
        // Spanning Tree and Loop Edges
        ar & mbFirstConnection;
        ar & mBackupParentId;
        ar & mvBackupChildrensId;
        ar & mvBackupLoopEdgesId;
        ar & mvBackupMergeEdgesId;
        // Bad flags
        ar & mbNotErase;
        ar & mbToBeErased;
        ar & mbBad;

        ar & mHalfBaseline;

        // Camera variables
        ar & mnBackupIdCamera;
        ar & mnBackupIdCamera2;

        // Fisheye variables
        /*ar & mvLeftToRightMatch;
        ar & mvRightToLeftMatch;
        ar & NLeft;
        ar & NRight;
        serializeMatrix(ar, mTlr, version);
        //serializeMatrix(ar, mTrl, version);
        serializeVectorKeyPoints(ar, mvKeysRight, version);
        ar & mGridRight;

        // Inertial variables
        ar & mImuBias;
        ar & mBackupImuPreintegrated;
        ar & mImuCalib;
        ar & mBackupPrevKFId;
        ar & mBackupNextKFId;
        ar & bImu;
        serializeMatrix(ar, Vw, version);
        serializeMatrix(ar, Owb, version);*/

    }

public:
    //拷贝构造类，加了线的参数
    KeyFrame();
    KeyFrame(Frame &F, Map* pMap, KeyFrameDatabase* pKFDB);

    //myplan
    ~KeyFrame();

    // Pose functions
    void SetPose(const cv::Mat &Tcw);
    void SetVelocity(const cv::Mat &Vw_);

    cv::Mat GetPose();
    cv::Mat GetPoseInverse();
    cv::Mat GetCameraCenter();
    cv::Mat GetImuPosition();
    cv::Mat GetImuRotation();
    cv::Mat GetImuPose();
    cv::Mat GetStereoCenter();
    cv::Mat GetRotation();
    cv::Mat GetTranslation();
    cv::Mat GetVelocity();

    // Bag of Words Representation
    void ComputeBoW();

    // Covisibility graph functions
    void AddConnection(KeyFrame* pKF, const int &weight);
    void EraseConnection(KeyFrame* pKF);

    void UpdateConnections(bool upParent=true);
    // myline 对线共识图更新
    void UpdateConnectionsWithLines(bool upParent=true);
    void UpdateBestCovisibles();
    std::set<KeyFrame *> GetConnectedKeyFrames();
    std::vector<KeyFrame* > GetVectorCovisibleKeyFrames();
    std::vector<KeyFrame*> GetBestCovisibilityKeyFrames(const int &N);
    std::vector<KeyFrame*> GetCovisiblesByWeight(const int &w);
    int GetWeight(KeyFrame* pKF);

    // Spanning tree functions
    void AddChild(KeyFrame* pKF);
    void EraseChild(KeyFrame* pKF);
    void ChangeParent(KeyFrame* pKF);
    std::set<KeyFrame*> GetChilds();
    KeyFrame* GetParent();
    bool hasChild(KeyFrame* pKF);
    void SetFirstConnection(bool bFirst);

    // Loop Edges
    void AddLoopEdge(KeyFrame* pKF);
    std::set<KeyFrame*> GetLoopEdges();

    // Merge Edges
    void AddMergeEdge(KeyFrame* pKF);
    set<KeyFrame*> GetMergeEdges();

    // MapPoint observation functions
    int GetNumberMPs();
    void AddMapPoint(MapPoint* pMP, const size_t &idx);
    void EraseMapPointMatch(const int &idx);
    void EraseMapPointMatch(MapPoint* pMP);
    void ReplaceMapPointMatch(const int &idx, MapPoint* pMP);
    std::set<MapPoint*> GetMapPoints();
    std::vector<MapPoint*> GetMapPointMatches();
    int TrackedMapPoints(const int &minObs);
    MapPoint* GetMapPoint(const size_t &idx);

    // MapLine observation functions myline 关于mapline 的函数
    void AddMapLine(MapLine* pML, const size_t &idx);
    void EraseMapLineMatch(const size_t &idx);
    void EraseMapLineMatch(MapLine* pML);
    void ReplaceMapLineMatch(const size_t &idx, MapLine* pML);
    std::set<MapLine*> GetMapLines();
    std::vector<MapLine*> GetMapLineMatches();
    int TrackedMapLines(const int &minObs);
    MapLine* GetMapLine(const size_t &idx);

    // KeyPoint functions
    std::vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const bool bRight = false) const;
    cv::Mat UnprojectStereo(int i);

    // Image
    bool IsInImage(const float &x, const float &y) const;

    // Enable/Disable bad flag changes
    void SetNotErase();
    void SetErase();

    // Set/check bad flag
    void SetBadFlag();
    void SetBadFlagWithLines();
    bool isBad();

    // Compute Scene Depth (q=2 median). Used in monocular.
    float ComputeSceneMedianDepth(const int q);

    static bool weightComp( int a, int b){
        return a>b;
    }

    static bool lId(KeyFrame* pKF1, KeyFrame* pKF2){
        return pKF1->mnId<pKF2->mnId;
    }

    Map* GetMap();
    void UpdateMap(Map* pMap);

    void SetNewBias(const IMU::Bias &b);
    cv::Mat GetGyroBias();
    cv::Mat GetAccBias();
    IMU::Bias GetImuBias();

    bool ProjectPointDistort(MapPoint* pMP, cv::Point2f &kp, float &u, float &v);
    bool ProjectPointUnDistort(MapPoint* pMP, cv::Point2f &kp, float &u, float &v);

    void PreSave(set<KeyFrame*>& spKF,set<MapPoint*>& spMP, set<GeometricCamera*>& spCam);
    void PostLoad(map<long unsigned int, KeyFrame*>& mpKFid, map<long unsigned int, MapPoint*>& mpMPid, map<unsigned int, GeometricCamera*>& mpCamId);


    void SetORBVocabulary(ORBVocabulary* pORBVoc);
    void SetKeyFrameDatabase(KeyFrameDatabase* pKFDB);

    bool bImu;

    //myplan

    cv::Mat get_segmentation_mask() const;
    void set_segmentation_mask(const cv::Mat &img_seg_mask);
    void set_depth_map(const cv::Mat &depth_img);

    void SetNotEraseDrawer();
    void SetEraseDrawer();
    vector<float> GetTexCoordinate(float x, float y, float z);// for drawing model
    // used in semidense mapping
    cv::Mat GetImage();
    bool MappingIdDelay();

    cv::Mat GetDescriptors();
    vector< cv::KeyPoint > GetKeyPointsUn() const;
    cv::Mat GetCalibrationMatrix() const;
    DBoW2::FeatureVector GetFeatureVector();
    std::vector<float> GetAllPointDepths();
    void IncreaseMappingId();
    bool Mapped();
    bool PoseChanged();
    void SetPoseChanged(bool bChanged);
    void Release();
    void SetNotEraseSemiDense();
    void SetEraseSemiDense();

private:
    // FW: for planar mapping module
    cv::Mat _img_seg_mask; // for planar mapping
    cv::Mat _depth_img;    // for dense reconstruction (demo)
    cv::Mat _img_rgb;      // for assigning color to the dense point cloud (demo)



    // The following variables are accesed from only 1 thread or never change (no mutex needed).
public:

    /***********semi dense*******************************///MYPLAN 这些变量需要清查
    // img used to semidense
    cv::Mat im_;
    cv::Mat rgb_;
    bool semidense_flag_;  // whether this frame have build dense map or not?
    bool interKF_depth_flag_; // for inter kf depth check
    cv::Mat GradImg,GradTheta;
    float I_stddev;
    cv::Mat depth_map_;
    cv::Mat depth_sigma_;
    cv::Mat depth_map_checked_;
    bool poseChanged;
    std::mutex mMutexSemiDensePoints;
    cv::Mat SemiDensePointSets_;
    static long unsigned int nNextMappingId;
    long unsigned int mnMappingId;
    std::mutex mMutexMappingId;
    cv::Mat mLines; // line segments, each row: (sx, sy, ex, ey)
    cv::Mat mLineIndex; // segment correspondence index for each pixel (integers, init: -1)
    cv::Mat mLinesSeg; // line segments given by depth and pixel: (sx, sy, ex, ey)
    cv::Mat mLines3D; // 3D line segments, each row: (sx, sy, sz, ex, ey, ez)
    cv::Mat mEdgeIndex; // edge chains correspondence index for each pixel (integers, init: -1)
    EdgeMap* mEdgeMap;

/******************************************/
    std::vector<Object_2D*> objects_kf;     // 2d objects.
    bool mbCreatedByObjs = false;           // keyframe created by keyframe.



    static long unsigned int nNextId;
    long unsigned int mnId;
    const long unsigned int mnFrameId;

    const double mTimeStamp;

    // Grid (to speed up feature matching)
    const int mnGridCols;
    const int mnGridRows;
    const float mfGridElementWidthInv;
    const float mfGridElementHeightInv;

    // Variables used by the tracking
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnFuseTargetForKF;

    // Variables used by the local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnBAFixedForKF;

    //Number of optimizations by BA(amount of iterations in BA)
    long unsigned int mnNumberOfOpt;

    // Variables used by the keyframe database
    long unsigned int mnLoopQuery;
    int mnLoopWords;
    float mLoopScore;
    long unsigned int mnRelocQuery;
    int mnRelocWords;
    float mRelocScore;
    long unsigned int mnMergeQuery;
    int mnMergeWords;
    float mMergeScore;
    long unsigned int mnPlaceRecognitionQuery;
    int mnPlaceRecognitionWords;
    float mPlaceRecognitionScore;

    bool mbCurrentPlaceRecognition;


    // Variables used by loop closing
    cv::Mat mTcwGBA;
    cv::Mat mTcwBefGBA;
    cv::Mat mVwbGBA;
    cv::Mat mVwbBefGBA;
    IMU::Bias mBiasGBA;
    long unsigned int mnBAGlobalForKF;

    // Variables used by merging
    cv::Mat mTcwMerge;
    cv::Mat mTcwBefMerge;
    cv::Mat mTwcBefMerge;
    cv::Mat mVwbMerge;
    cv::Mat mVwbBefMerge;
    IMU::Bias mBiasMerge;
    long unsigned int mnMergeCorrectedForKF;
    long unsigned int mnMergeForKF;
    float mfScaleMerge;
    long unsigned int mnBALocalForMerge;

    float mfScale;

    // Calibration parameters
    const float fx, fy, cx, cy, invfx, invfy, mbf, mb, mThDepth;
    cv::Mat mDistCoef;

    // Number of KeyPoints
    const int N;

    // Number of Keylines  myline 线数量
    const int N_l;

    // KeyPoints, stereo coordinate and descriptors (all associated by an index)
    const std::vector<cv::KeyPoint> mvKeys;
    const std::vector<cv::KeyPoint> mvKeysUn;
    const std::vector<float> mvuRight; // negative value for monocular points
    const std::vector<float> mvDepth; // negative value for monocular points
    const cv::Mat mDescriptors;

    // KeyLines
    const std::vector<cv::line_descriptor::KeyLine> mvKeys_Line;
    const std::vector<cv::line_descriptor::KeyLine> mvKeysUn_Line;
    std::vector<pair<float,float>> mvDisparity_l;
    std::vector<Vector3d> mvle_l;
    std::vector<pair<float,float>> mvDepth_l;
    const cv::Mat mDescriptors_l;

    //BoW
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    // Pose relative to parent (this is computed when bad flag is activated)
    cv::Mat mTcp;

    // Scale
    const int mnScaleLevels;
    const float mfScaleFactor;
    const float mfLogScaleFactor;
    const std::vector<float> mvScaleFactors;
    const std::vector<float> mvLevelSigma2;
    const std::vector<float> mvInvLevelSigma2;
    const std::vector<float> mvInvLevelSigma2_l;

    // Scale Lines myline 的缩放尺度
    const int mnScaleLevels_l;
    const std::vector<float> mvScaleFactors_l;

    // Image bounds and calibration
    const int mnMinX;
    const int mnMinY;
    const int mnMaxX;
    const int mnMaxY;
    const cv::Mat mK;

    // Preintegrated IMU measurements from previous keyframe
    KeyFrame* mPrevKF;
    KeyFrame* mNextKF;

    IMU::Preintegrated* mpImuPreintegrated;
    IMU::Calib mImuCalib;


    unsigned int mnOriginMapId;

    string mNameFile;

    int mnDataset;

    std::vector <KeyFrame*> mvpLoopCandKFs;
    std::vector <KeyFrame*> mvpMergeCandKFs;

    bool mbHasHessian;
    cv::Mat mHessianPose;

    // The following variables need to be accessed trough a mutex to be thread safe.
protected:
//myplan

    // semi dense: erase flags for different threads
    bool mbNotEraseSemiDense;
    bool mbNotEraseDrawer;


    // SE3 Pose and camera center
    cv::Mat Tcw;
    cv::Mat Twc;
    cv::Mat Ow;
    cv::Mat Cw; // Stereo middel point. Only for visualization

    // IMU position
    cv::Mat Owb;

    // Velocity (Only used for inertial SLAM)
    cv::Mat Vw;

    // Imu bias
    IMU::Bias mImuBias;

    // MapPoints associated to keypoints
    std::vector<MapPoint*> mvpMapPoints;
    // MapLines associated to keylines
    //myline 地图线容器
    std::vector<MapLine*> mvpMapLines;
    // For save relation without pointer, this is necessary for save/load function
    std::vector<long long int> mvBackupMapPointsId;

    // BoW
    KeyFrameDatabase* mpKeyFrameDB;
    ORBVocabulary* mpORBvocabulary;
    ORBVocabulary* mpLinevocabulary;

    // Grid over the image to speed up feature matching
    std::vector< std::vector <std::vector<size_t> > > mGrid;

    std::map<KeyFrame*,int> mConnectedKeyFrameWeights;
    std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames;
    std::vector<int> mvOrderedWeights;
    // For save relation without pointer, this is necessary for save/load function
    std::map<long unsigned int, int> mBackupConnectedKeyFrameIdWeights;

    // Spanning Tree and Loop Edges
    bool mbFirstConnection;
    KeyFrame* mpParent;
    std::set<KeyFrame*> mspChildrens;
    std::set<KeyFrame*> mspLoopEdges;
    std::set<KeyFrame*> mspMergeEdges;
    // For save relation without pointer, this is necessary for save/load function
    long long int mBackupParentId;
    std::vector<long unsigned int> mvBackupChildrensId;
    std::vector<long unsigned int> mvBackupLoopEdgesId;
    std::vector<long unsigned int> mvBackupMergeEdgesId;

    // Bad flags
    bool mbNotErase;
    bool mbToBeErased;
    bool mbBad;    

    float mHalfBaseline; // Only for visualization

    Map* mpMap;

    std::mutex mMutexPose; // for pose, velocity and biases
    std::mutex mMutexConnections;
    std::mutex mMutexFeatures;
    std::mutex mMutexMap;

    // Backup variables for inertial
    long long int mBackupPrevKFId;
    long long int mBackupNextKFId;
    IMU::Preintegrated mBackupImuPreintegrated;

    // Backup for Cameras
    unsigned int mnBackupIdCamera, mnBackupIdCamera2;

public:
    GeometricCamera* mpCamera, *mpCamera2;

    //Indexes of stereo observations correspondences
    std::vector<int> mvLeftToRightMatch, mvRightToLeftMatch;

    //Transformation matrix between cameras in stereo fisheye
    cv::Mat mTlr;
    cv::Mat mTrl;

    //KeyPoints in the right image (for stereo fisheye, coordinates are needed)
    const std::vector<cv::KeyPoint> mvKeysRight;

    const int NLeft, NRight;

    std::vector< std::vector <std::vector<size_t> > > mGridRight;

    cv::Mat GetRightPose();
    cv::Mat GetRightPoseInverse();
    cv::Mat GetRightPoseInverseH();
    cv::Mat GetRightCameraCenter();
    cv::Mat GetRightRotation();
    cv::Mat GetRightTranslation();

    cv::Mat imgLeft, imgRight; //TODO Backup??

    void PrintPointDistribution(){
        int left = 0, right = 0;
        int Nlim = (NLeft != -1) ? NLeft : N;
        for(int i = 0; i < N; i++){
            if(mvpMapPoints[i]){
                if(i < Nlim) left++;
                else right++;
            }
        }
        cout << "Point distribution in KeyFrame: left-> " << left << " --- right-> " << right << endl;
    }


};

} //namespace ORB_SLAM

#endif // KEYFRAME_H
