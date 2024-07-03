

#ifndef FRAME_H
#define FRAME_H

//#define SAVE_TIMES

#include<vector>

#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"

#include "ImuTypes.h"
#include "ORBVocabulary.h"

#include <mutex>
#include <opencv2/opencv.hpp>
#include "LineExtractor.h"

#include <line_descriptor_custom.hpp>
#include <line_descriptor/descriptor_custom.hpp>
#include "gridStructure.h"

#include <eigen3/Eigen/Core>
//mylk 新增
#include "utils.h"

using namespace Eigen;

namespace ORB_SLAM3
{
#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

typedef Matrix<double,6,1> Vector6d;

class MapPoint;
class MapLine;
class KeyFrame;
class ConstraintPoseImu;
class GeometricCamera;
class ORBextractor;
//myline 添加两个线类
class Lineextractor;
class Object_2D;

//myplan
//BoxSE 类提供了一种方便的方法来表示带有类别、
// 置信度和矩形框信息的对象。这对于物体检测和图像处理等任务非常有用。
// 类是继承自 cv::Rect 类的。cv::Rect 是 OpenCV 库中定义的一个矩形类，
// BoxSE 继承了 cv::Rect 的成员变量和成员函数，并在此基础上添加了自己的成员
class BoxSE : public cv::Rect
{
public:
    int m_class = -1;			// class id.
    float m_score = 0.0F;		// probability.
    std::string m_class_name;	// class name.

    BoxSE()
    {
        m_class_name = "Unknown";
    }
//参数构造函数。它接受类别 c、置信度 s、矩形框的坐标 _x、_y，宽度 _w 和高度 _h 作为参数。可以选择性地提供类别名称 name
    BoxSE(int c, float s, int _x, int _y, int _w, int _h, std::string name = "")
            :m_class(c), m_score(s)
    {
        this->x = _x;
        this->y = _y;
        this->width = _w;
        this->height = _h;
        char const *lb[5] = { "th","st","nd","rd","th" };

        if (name.length() == 0)
        {
            m_class_name = std::to_string(m_class) + lb[m_class < 4 ? m_class : 4] + " class";
        }
    }
};


class Frame
{
public:
    Frame();

    // Copy constructor.
    Frame(const Frame &frame);

    // Constructor for stereo cameras.
    Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth, GeometricCamera* pCamera,Frame* pPrevF = static_cast<Frame*>(NULL), const IMU::Calib &ImuCalib = IMU::Calib());

    // Constructor for stereo cameras with myline +const cv::Mat& semantic_img
    //myline 构建加左右目线提取器的Frame类在grabstereo用到
    Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, Lineextractor* LineextractorLeft, Lineextractor* LineextractorRight, ORBVocabulary* voc, LineVocabulary* voc_l, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth, GeometricCamera* pCamera,Frame* pPrevF = static_cast<Frame*>(NULL), const IMU::Calib &ImuCalib = IMU::Calib());

    // // MYLK 新增的Stereo+line+lk构造类 根据snesor不同阈值不同
    Frame(const cv::Mat &imLeft, const cv::Mat &imRight,vector<Detection> &mmDetectMap,const int &mSensor, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, Lineextractor* LineextractorLeft, Lineextractor* LineextractorRight, ORBVocabulary* voc, LineVocabulary* voc_l, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth, GeometricCamera* pCamera,Frame* pPrevF = static_cast<Frame*>(NULL), const IMU::Calib &ImuCalib = IMU::Calib());

    // Constructor for RGB-D cameras.
    Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth, GeometricCamera* pCamera,Frame* pPrevF = static_cast<Frame*>(NULL), const IMU::Calib &ImuCalib = IMU::Calib());
    //myline 构建加线的 RGB 提取器的Frame类在grabstereo用到
    Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor* extractor,Lineextractor* Lineextractor ,
          ORBVocabulary* voc,  LineVocabulary* voc_l,cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth,
          GeometricCamera* pCamera,Frame* pPrevF = static_cast<Frame*>(NULL), const IMU::Calib &ImuCalib = IMU::Calib());
    // //MYLK 新增的RGB+line+lk构造类 根据snesor不同阈值不同
    Frame( const cv::Mat &rawImage,
           const cv::Mat &imGray,
          const cv::Mat &imDepth,
           const cv::Mat &img_seg_mask,
          vector<Detection> &mmDetectMap,
          const int &mSensor,
          const double &timeStamp,
          ORBextractor* extractor,
          Lineextractor* Lineextractor ,
          ORBVocabulary* voc,
          LineVocabulary* voc_l,
          cv::Mat &K,
          cv::Mat &distCoef,
          const float &bf,
          const float &thDepth,
          GeometricCamera* pCamera,
           const cv::Mat& grayimg,
           const cv::Mat& rgbimg,
          Frame* pPrevF = static_cast<Frame*>(NULL),
          const IMU::Calib &ImuCalib = IMU::Calib());
//myplan
    cv::Mat mColorImage;
    cv::Mat mQuadricImage;
    cv::Mat im_;
    cv::Mat rgb_;
    bool finish_detected;           // whether finished object detection.
    std::vector<BoxSE> boxes;       // object box, vector<BoxSE> format.
    Eigen::MatrixXd boxes_eigen;    // object box, Eigen::MatrixXd format.
    bool have_detected;             // whether detected objects in current frame.
    cv::Mat mGroundtruthPose_mat;           // camera groundtruth.
    Eigen::Matrix4d mGroundtruthPose_eigen; // camera groundtruth.
    std::vector<Object_2D*> mvLastObjectFrame;      // last frame.
    std::vector<Object_2D*> mvLastLastObjectFrame;  // last last frame.
    std::vector<Object_2D*> mvObjectFrame;          // 2d object in current frame.
    std::vector<Eigen::MatrixXd> vObjsLines;//经过merge后的线
    bool AppearNewObject = false;

    // Constructor for Monocular cameras.
    Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, GeometricCamera* pCamera, cv::Mat &distCoef, const float &bf, const float &thDepth, Frame* pPrevF = static_cast<Frame*>(NULL), const IMU::Calib &ImuCalib = IMU::Calib());

    bool CheckSame(const cv::Vec3b& intensity, const std::unordered_map<std::string, int>& rgb);

    bool CheckDynamicPoint(const cv::Mat& semantic_img, const int col, const int row);

    // Destructor
    // ~Frame();
    void SelectObject(const cv::Mat& semantic_img);
    void SelectLineObject(const cv::Mat& semantic_img);

    // Extract ORB on the image. 0 for left image and 1 for right image.
    void ExtractORB(int flag, const cv::Mat &im, const int x0, const int x1);

    ///mylk 新增加函数
    static void InitializeClass(const cv::Mat &im);
    void ExtractORBKeyPoints(int flag, const cv::Mat &im, const int x0, const int x1);
    void ExtractORBDesp(int flag,const cv::Mat &im,const int x0, const int x1);
    void ExtractStereoORBDesp(int flag,const cv::Mat &imgray, const cv::Mat &imRight,const int x0, const int x1);
    ///mylk end
    std::vector<cv::Rect2i> mvDynamicArea;//imp 第三次
    // Extract LineFeatures on the image. 0 for left image and 1 for right image.
    //myline 线提取函数
    void ExtractLine(int flag, const cv::Mat &im);

    // Compute Bag of Words representation.
    void ComputeBoW();

    // Set the camera pose. (Imu pose is not modified!)
    void SetPose(cv::Mat Tcw);
    void GetPose(cv::Mat &Tcw);

    // Set IMU velocity
    void SetVelocity(const cv::Mat &Vwb);

    // Set IMU pose and velocity (implicitly changes camera pose)
    void SetImuPoseVelocity(const cv::Mat &Rwb, const cv::Mat &twb, const cv::Mat &Vwb);


    // Computes rotation, translation and camera center matrices from the camera pose.
    void UpdatePoseMatrices();

    // Returns the camera center.
    inline cv::Mat GetCameraCenter(){
        return mOw.clone();
    }

    // Returns inverse of rotation
    inline cv::Mat GetRotationInverse(){
        return mRwc.clone();
    }

    cv::Mat GetImuPosition();
    cv::Mat GetImuRotation();
    cv::Mat GetImuPose();

    void SetNewBias(const IMU::Bias &b);

    // Check if a MapPoint is in the frustum of the camera
    // and fill variables of the MapPoint to be used by the tracking
    bool isInFrustum(MapPoint* pMP, float viewingCosLimit);
    //myline 判断线函数
    bool isInFrustum_l(MapLine *pML, float viewingCosLimit);

    bool ProjectPointDistort(MapPoint* pMP, cv::Point2f &kp, float &u, float &v);

    cv::Mat inRefCoordinates(cv::Mat pCw);

    // Compute the cell of a keypoint (return false if outside the grid)
    bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);

    vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel=-1, const int maxLevel=-1, const bool bRight = false) const;

    // Search a match for each keypoint in the left image to a keypoint in the right image.
    // If there is a match, depth is computed and the right coordinate associated to the left keypoint is stored.
    void ComputeStereoMatches();

    // Search a match for each line in the left image to a line in the right image and computes the depth
    //myline 线匹配函数
    void ComputeStereoMatches_Lines();
    //myplan 线转为cvmat// std::vector<cv::line_descriptor::KeyLine>
    void keylines_to_mat(const std::vector<cv::line_descriptor::KeyLine>& keylines_src,
                         cv::Mat& linesmat_out,
                         float scale=1);
//    vector<float> mvScaleFactors_l
    // Associate a "right" coordinate to a keypoint if there is valid depth in the depthmap.
    void ComputeStereoFromRGBD(const cv::Mat &imDepth);
    void ComputeRGBDMatches_Lines(const cv::Mat &imDepth);//myline rgbd匹配函数

    // Backprojects a keypoint (if stereo/depth info available) into 3D world coordinates.
    cv::Mat UnprojectStereo(const int &i);

    // Backprojects a keyline's endpoint (if stereo/depth info available) into 3D world coordinates.
    Eigen::Vector3d UnprojectStereoLines(const double &u, const double &v, const double &z);

    // backproject 3D point(camera coodinate) into [u,v] image coordinate
    Eigen::Vector2d projection(const Eigen::Vector3d &P);

    // overlap between obs lineSegment and proj lineSegment
    ///myline 好像是覆盖语义图相关的函数
    double lineSegmentOverlapStereo(double spl_obs, double epl_obs, double spl_proj, double epl_proj);
    void filterLineSegmentDisparity(Vector2d spl, Vector2d epl, Vector2d spr, Vector2d epr, double &disp_s, double &disp_e);

    ConstraintPoseImu* mpcpi;

    bool imuIsPreintegrated();
    void setIntegrated();

    cv::Mat mRwc;
    cv::Mat mOw;
    //// mylk 新增
    std::vector<cv::Rect2i> person_area;
    std::vector<cv::Point2f> F_prepoint_draw, F_nextpoint_draw;
    std::vector<uchar> state_draw;
    ///mylk 新增函数
    // For semantic segmentation thread
    void CalculEverything( cv::Mat &imRGB,const cv::Mat &imGray,const cv::Mat &imDepth,cv::Mat &K,vector<Detection> mmDetectMap);

    void StereoCalculEverything( const cv::Mat &imGray, const cv::Mat &imRight,vector<Detection> mmDetectMap);
    void ProcessMovingObject(const cv::Mat &imgray,vector<Detection> &mmDetectMap );
    // Sets for abnormal points
    std::vector<cv::Point2f> T_M;
    std::vector<cv::Point2f> T_M_dyna;
    std::vector<cv::Point2f> T_M_staic;
    double limit_dis_epi =1;
    double limit_of_check = 2120;
    int limit_edge_corner = 5;
    int flag_mov ;
    std::vector<std::vector<cv::KeyPoint>> mvKeysTemp;
    std::vector<std::vector<cv::KeyPoint>> mvRigghtKeysTemp;
    std::vector<std::string> mClassnames;
    void visualizeOpticalFlow(const cv::Mat& image, const std::vector<cv::Point2f>& points1, const std::vector<cv::Point2f>& points2, const std::vector<uchar>& status);
///mylk 新增end
//myplan
    // FW: for planar mapping module
    cv::Mat _img_seg_mask; // passed to keyframe, for planar mapping
    cv::Mat _depth_img;    // passed to keyframe, for dense reconstruction (demo)
    cv::Mat _img_rgb;


public:
    //myplan
    cv::Mat all_lines_mat;
    Eigen::MatrixXd all_lines_eigen;

    int fSensor;//imp 第三次
    // Vocabulary used for relocalization.
    ORBVocabulary* mpORBvocabulary;
    LineVocabulary* mpLinevocabulary;//myline 线辞典型参

    // Feature extractor. The right is used only in the stereo case.Lineextractor
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;

    // Line Feature extractor. myline 左右目线提取器
    Lineextractor* mpLineextractorLeft, *mpLineextractorRight;

    // Frame timestamp.
    double mTimeStamp;

    // Calibration matrix and OpenCV distortion parameters.
    cv::Mat mK;
    static float fx;
    static float fy;
    static float cx;
    static float cy;
    static float invfx;
    static float invfy;
    cv::Mat mDistCoef;

    // Stereo baseline multiplied by fx.
    float mbf;

    // Stereo baseline in meters.
    float mb;

    // Threshold close/far points. Close points are inserted from 1 view.
    // Far points are inserted as in the monocular case from 2 views.
    float mThDepth;

    // Number of KeyPoints.
    int N;

    // Number of Keylines myline 线数量
    int N_l;

    // Vector of keypoints (original for visualization) and undistorted (actually used by the system).
    // In the stereo case, mvKeysUn is redundant as images must be rectified.
    // In the RGB-D case, RGB images can be distorted.
    std::vector<cv::KeyPoint> mvKeys, mvKeysRight;
    std::vector<cv::KeyPoint> mvKeysUn;

    // Vector of keylines myline 线容器
    std::vector<cv::line_descriptor::KeyLine> mvKeys_Line, mvKeysRight_Line;
    std::vector<cv::line_descriptor::KeyLine> mvKeysUn_Line;

    // Corresponding stereo coordinate and depth for each keypoint.
    std::vector<MapPoint*> mvpMapPoints;

    // MapLines associated to keylines, NULL pointer if no association.
    //myline 地图线
    std::vector<MapLine*> mvpMapLines;

    // "Monocular" keypoints have a negative value.
    std::vector<float> mvuRight;
    std::vector<float> mvDepth;

    // for PLslam myline 线特征
    std::vector<pair<float,float>> mvDepth_l;

    // for PLslam myline 线特征
    // std::vector<LineFeature*> stereo_ls;
    //myline 线特征
    std::vector<pair<float,float>> mvDisparity_l;//线的视差
    std::vector<Vector3d> mvle_l;//线段的三维向量表示

    // Flag to identify outlier associations.
    std::vector<Vector3d> mv3DpointInPrevFrame;
    std::vector<pair<Vector3d,Vector3d>> mv3DlineInPrevFrame;

    // Bag of Words Vector structures.
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    // ORB descriptor, each row associated to a keypoint.
    cv::Mat mDescriptors, mDescriptorsRight;

    // Line descriptor, each row associated to a keyline. myline 线+线描述子
    cv::Mat mDescriptors_Line, mDescriptorsRight_Line;

    // MapPoints associated to keypoints, NULL pointer if no association.
    // Flag to identify outlier associations.
    std::vector<bool> mvbOutlier;
    //线的外点myline
    std::vector<bool> mvbOutlier_Line;
    int mnCloseMPs;
    //myline 外点-线
    int mnCloseMLs;

    // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
    static float mfGridElementWidthInv;
    static float mfGridElementHeightInv;
    std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];


    // Camera pose.
    cv::Mat mTcw;
    cv::Mat mTcw_prev;

    // IMU linear velocity
    cv::Mat mVw;

    cv::Mat mPredRwb, mPredtwb, mPredVwb;
    IMU::Bias mPredBias;

    // IMU bias
    IMU::Bias mImuBias;

    // Imu calibration
    IMU::Calib mImuCalib;

    // Imu preintegration from last keyframe
    IMU::Preintegrated* mpImuPreintegrated;
    KeyFrame* mpLastKeyFrame;

    // Pointer to previous frame
    Frame* mpPrevFrame;
    IMU::Preintegrated* mpImuPreintegratedFrame;

    // Current and Next Frame id.
    static long unsigned int nNextId;
    long unsigned int mnId;

    // Reference Keyframe.
    KeyFrame* mpReferenceKF;

    // Scale pyramid info.
    int mnScaleLevels;
    float mfScaleFactor;
    float mfLogScaleFactor;
    vector<float> mvScaleFactors;
    vector<float> mvInvScaleFactors;
    vector<float> mvLevelSigma2;
    vector<float> mvInvLevelSigma2;

    // Scale pyramid info Lines.
    // myline 线金字塔的一些参数
    int mnScaleLevels_l;
    vector<float> mvScaleFactors_l;
    vector<float> mvInvScaleFactors_l;
    vector<float> mvLevelSigma2_l;
    vector<float> mvInvLevelSigma2_l;

    // Undistorted Image Bounds (computed once).
    static float mnMinX;
    static float mnMaxX;
    static float mnMinY;
    static float mnMaxY;

    static bool mbInitialComputations;

    // grid cell myline 网格
    double inv_width, inv_height; 

    // grid for Lines -> Used for Line Matching By Projection
    //GridStructure grid_Line;//myline 线网格参数

    int  n_inliers, n_inliers_pt, n_inliers_ls;  //imp 这个参数需要调查

    map<long unsigned int, cv::Point2f> mmProjectPoints;
    map<long unsigned int, cv::Point2f> mmMatchedInImage;

    string mNameFile;

    int mnDataset;

    double mTimeStereoMatch;
    double mTimeStereoMatch_Lines;
    double mTimeStereoMatchTotal;
    double mTimeORB_Ext;
    double mTimeLines_Ext;
    double mTimeTotal_Ext;

private:

    // Undistort keypoints given OpenCV distortion parameters.
    // Only for the RGB-D case. Stereo must be already rectified!
    // (called in the constructor).
    void UndistortKeyPoints();

    // Undistort keylines
    void UndistortKeyLines();

    // Computes image bounds for the undistorted image (called in the constructor).
    void ComputeImageBounds(const cv::Mat &imLeft);

    // Assign keypoints to the grid for speed up feature matching (called in the constructor).
    void AssignFeaturesToGrid();

    // Rotation, translation and camera center
    cv::Mat mRcw;
    cv::Mat mtcw;
    //==mtwc

    bool mbImuPreintegrated;

    std::mutex *mpMutexImu;



public:
    GeometricCamera* mpCamera, *mpCamera2;

    //Number of KeyPoints extracted in the left and right images
    int Nleft, Nright;
    //Number of Non Lapping Keypoints
    int monoLeft, monoRight;

    //For stereo matching
    std::vector<int> mvLeftToRightMatch, mvRightToLeftMatch;

    //For stereo fisheye matching
    static cv::BFMatcher BFmatcher;

    //Triangulated stereo observations using as reference the left camera. These are
    //computed during ComputeStereoFishEyeMatches
    std::vector<cv::Mat> mvStereo3Dpoints;

    //Grid for the right image
    std::vector<std::size_t> mGridRight[FRAME_GRID_COLS][FRAME_GRID_ROWS];

    cv::Mat mTlr, mRlr, mtlr, mTrl;

    Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth, GeometricCamera* pCamera, GeometricCamera* pCamera2, cv::Mat& Tlr,Frame* pPrevF = static_cast<Frame*>(NULL), const IMU::Calib &ImuCalib = IMU::Calib());

    //Stereo fisheye
    void ComputeStereoFishEyeMatches();

    bool isInFrustumChecks(MapPoint* pMP, float viewingCosLimit, bool bRight = false);

    cv::Mat UnprojectStereoFishEye(const int &i);

    cv::Mat imgLeft, imgRight;

    void PrintPointDistribution(){
        int left = 0, right = 0;
        int Nlim = (Nleft != -1) ? Nleft : N;
        for(int i = 0; i < N; i++){
            if(mvpMapPoints[i] && !mvbOutlier[i]){
                if(i < Nlim) left++;
                else right++;
            }
        }
        cout << "Point distribution in Frame: left-> " << left << " --- right-> " << right << endl;
    }
};

}// namespace ORB_SLAM

#endif // FRAME_H
