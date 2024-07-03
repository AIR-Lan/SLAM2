

#include "Frame.h"

#include "G2oTypes.h"
#include "MapPoint.h"
#include "MapLine.h"
#include "KeyFrame.h"
#include "ORBextractor.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include "GeometricCamera.h"
#include "LineMatcher.h"
#include <unordered_map>

#include <thread>
#include <include/CameraModels/Pinhole.h>
#include <include/CameraModels/KannalaBrandt8.h>
#include "Tracking.h"//imp 第三次增加
//// The previous image mylk 的一些变量定义
cv::Mat imGrayPre;
std::vector<cv::Point2f> prepoint, nextpoint;
std::vector<cv::Point2f> F_prepoint, F_nextpoint;
std::vector<cv::Point2f> F2_prepoint, F2_nextpoint;
std::vector<uchar> state;
std::vector<float> err;
std::vector<std::vector<cv::KeyPoint>> mvKeysPre;
namespace ORB_SLAM3
{

long unsigned int Frame::nNextId=0;
bool Frame::mbInitialComputations=true;
float Frame::cx, Frame::cy, Frame::fx, Frame::fy, Frame::invfx, Frame::invfy;
float Frame::mnMinX, Frame::mnMinY, Frame::mnMaxX, Frame::mnMaxY;
float Frame::mfGridElementWidthInv, Frame::mfGridElementHeightInv;

std::unordered_map<std::string, std::unordered_map<std::string, int>> color_map{
    // {"car", {{"r", 0}, {"g", 0}, {"b", 84}} },
    {"cycle", {{"r", 119}, {"g", 11}, {"b", 32}} },    
    {"car", {{"r", 0}, {"g", 0}, {"b", 142}} }
};

//For stereo fisheye matching
cv::BFMatcher Frame::BFmatcher = cv::BFMatcher(cv::NORM_HAMMING);

Frame::Frame(): mpcpi(NULL), mpImuPreintegrated(NULL), mpPrevFrame(NULL), mpImuPreintegratedFrame(NULL), mpReferenceKF(static_cast<KeyFrame*>(NULL)), mbImuPreintegrated(false)
{
}

//Frame::Frame(const int sensor) : fSensor(sensor) {
//    std::cout << "传入 Frame SENSOR: " << fSensor << std::endl;
//}


//Copy Constructor
Frame::Frame(const Frame &frame)
    :mpcpi(frame.mpcpi),mpORBvocabulary(frame.mpORBvocabulary), mpLinevocabulary(frame.mpLinevocabulary), mpORBextractorLeft(frame.mpORBextractorLeft), mpORBextractorRight(frame.mpORBextractorRight),
     mpLineextractorLeft(frame.mpLineextractorLeft), mpLineextractorRight(frame.mpLineextractorRight),
     mTimeStamp(frame.mTimeStamp), mK(frame.mK.clone()), mDistCoef(frame.mDistCoef.clone()),
     mbf(frame.mbf), mb(frame.mb), mThDepth(frame.mThDepth), N(frame.N),  N_l(frame.N_l), mvKeys(frame.mvKeys), mvKeys_Line(frame.mvKeys_Line),
     mvKeysRight(frame.mvKeysRight), mvKeysRight_Line(frame.mvKeysRight_Line), mvKeysUn(frame.mvKeysUn), mvKeysUn_Line(frame.mvKeysUn_Line), mvuRight(frame.mvuRight),
     mvDepth(frame.mvDepth), mvDepth_l(frame.mvDepth_l), mvDisparity_l(frame.mvDisparity_l), mvle_l(frame.mvle_l), mBowVec(frame.mBowVec), mFeatVec(frame.mFeatVec),
     mDescriptors(frame.mDescriptors.clone()), mDescriptors_Line(frame.mDescriptors_Line.clone()), mDescriptorsRight(frame.mDescriptorsRight.clone()), mDescriptorsRight_Line(frame.mDescriptorsRight_Line.clone()),
     mvpMapPoints(frame.mvpMapPoints), mvpMapLines(frame.mvpMapLines), mvbOutlier(frame.mvbOutlier), mvbOutlier_Line(frame.mvbOutlier_Line), mImuCalib(frame.mImuCalib), mnCloseMPs(frame.mnCloseMPs), mnCloseMLs(frame.mnCloseMLs), 
     mpImuPreintegrated(frame.mpImuPreintegrated), mpImuPreintegratedFrame(frame.mpImuPreintegratedFrame), mImuBias(frame.mImuBias),
     mnId(frame.mnId), mpReferenceKF(frame.mpReferenceKF), mnScaleLevels(frame.mnScaleLevels), mnScaleLevels_l(frame.mnScaleLevels_l),
     mfScaleFactor(frame.mfScaleFactor), mfLogScaleFactor(frame.mfLogScaleFactor),
     mvScaleFactors(frame.mvScaleFactors), mvInvScaleFactors(frame.mvInvScaleFactors), mvScaleFactors_l(frame.mvScaleFactors_l), mvInvScaleFactors_l(frame.mvInvScaleFactors_l), mNameFile(frame.mNameFile), mnDataset(frame.mnDataset),
     mvLevelSigma2(frame.mvLevelSigma2), mvInvLevelSigma2(frame.mvInvLevelSigma2), mvLevelSigma2_l(frame.mvLevelSigma2_l), mvInvLevelSigma2_l(frame.mvInvLevelSigma2_l), mpPrevFrame(frame.mpPrevFrame), mpLastKeyFrame(frame.mpLastKeyFrame), mbImuPreintegrated(frame.mbImuPreintegrated), mpMutexImu(frame.mpMutexImu),
     mpCamera(frame.mpCamera), mpCamera2(frame.mpCamera2), Nleft(frame.Nleft), Nright(frame.Nright),
     monoLeft(frame.monoLeft), monoRight(frame.monoRight), mvLeftToRightMatch(frame.mvLeftToRightMatch),
     mvRightToLeftMatch(frame.mvRightToLeftMatch), mvStereo3Dpoints(frame.mvStereo3Dpoints),
     mTlr(frame.mTlr.clone()), mRlr(frame.mRlr.clone()), mtlr(frame.mtlr.clone()), mTrl(frame.mTrl.clone()), mTimeStereoMatch(frame.mTimeStereoMatch), mTimeStereoMatch_Lines(frame.mTimeStereoMatch_Lines), 
     n_inliers(frame.n_inliers), n_inliers_pt(frame.n_inliers_pt), n_inliers_ls(frame.n_inliers_ls), 
     mTimeORB_Ext(frame.mTimeORB_Ext), mTimeLines_Ext(frame.mTimeLines_Ext), inv_width(frame.inv_width), inv_height(frame.inv_width),
     im_(frame.im_.clone()), rgb_(frame.rgb_.clone())
{
    for(int i=0;i<FRAME_GRID_COLS;i++)
        for(int j=0; j<FRAME_GRID_ROWS; j++){
            mGrid[i][j]=frame.mGrid[i][j];
            if(frame.Nleft > 0){
                mGridRight[i][j] = frame.mGridRight[i][j];
            }
        }

    if(!frame.mTcw.empty())
    {
        SetPose(frame.mTcw);
        mTcw_prev = frame.mTcw_prev.clone();
    }

    if(!frame.mVw.empty())
        mVw = frame.mVw.clone();

    // [EAO-SLAM] save groundtruth
    ////myplan 新增函数
    if(!frame.mGroundtruthPose_mat.empty())
    {
        mGroundtruthPose_mat = frame.mGroundtruthPose_mat;
        mGroundtruthPose_eigen = Eigen::Matrix4d::Zero(4, 4);
        // mGroundtruthPose_eigen = frame.mGroundtruthPose_eigen;
    }

    mmProjectPoints = frame.mmProjectPoints;
    mmMatchedInImage = frame.mmMatchedInImage;
}
///立体匹配双目，***没有线***用不到
Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth, GeometricCamera* pCamera, Frame* pPrevF, const IMU::Calib &ImuCalib)
    :mpcpi(NULL), mpORBvocabulary(voc),mpORBextractorLeft(extractorLeft),mpORBextractorRight(extractorRight), mTimeStamp(timeStamp), mK(K.clone()), mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
     mImuCalib(ImuCalib), mpImuPreintegrated(NULL), mpPrevFrame(pPrevF),mpImuPreintegratedFrame(NULL), mpReferenceKF(static_cast<KeyFrame*>(NULL)), mbImuPreintegrated(false),
     mpCamera(pCamera) ,mpCamera2(nullptr), mTimeStereoMatch(0), mTimeORB_Ext(0)
{
    // Frame ID
    // myStep 1 帧的ID 自增
    mnId=nNextId++;

    // Scale Level Info
    // myStep 2 计算图像金字塔的参数
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    // 获得层与层之间的缩放比
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    // 计算上面缩放比的对数
    mfLogScaleFactor = log(mfScaleFactor);
    // 获取每层图像的缩放因子
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    // 同样获取每层图像缩放因子的倒数
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    // 高斯模糊的时候，使用的方差
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    // 获取sigma^2的倒数
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
#ifdef SAVE_TIMES
    std::chrono::steady_clock::time_point time_StartExtORB = std::chrono::steady_clock::now();
#endif
        // Step 3 对左目右目图像提取ORB特征点, 第一个参数0-左图， 1-右图。为加速计算，同时开了两个线程计算
    thread threadLeft(&Frame::ExtractORB,this,0,imLeft,0,0);
        // 对右目图像提取orb特征
    thread threadRight(&Frame::ExtractORB,this,1,imRight,0,0);
        // 等待两张图像特征点提取过程完成
    threadLeft.join();
    threadRight.join();
#ifdef SAVE_TIMES
    std::chrono::steady_clock::time_point time_EndExtORB = std::chrono::steady_clock::now();

    mTimeORB_Ext = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndExtORB - time_StartExtORB).count();
#endif

        // mvKeys中保存的是左图像中的特征点，这里是获取左侧图像中特征点的个数
    N = mvKeys.size();
        // 如果左图像中没有成功提取到特征点那么就返回，也意味这这一帧的图像无法使用
    if(mvKeys.empty())
        return;
        // myStep 4 用OpenCV的矫正函数、内参对提取到的特征点进行矫正
    UndistortKeyPoints();

#ifdef SAVE_TIMES
    std::chrono::steady_clock::time_point time_StartStereoMatches = std::chrono::steady_clock::now();
#endif
        // Step 5 计算双目间特征点的匹配，只有匹配成功的特征点会计算其深度,深度存放在 mvDepth
        // mvuRight中存储的应该是左图像中的点所匹配的在右图像中的点的横坐标（纵坐标相同）
    ComputeStereoMatches();
#ifdef SAVE_TIMES
    std::chrono::steady_clock::time_point time_EndStereoMatches = std::chrono::steady_clock::now();
    
    mTimeStereoMatch = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndStereoMatches - time_StartStereoMatches).count();
#endif


    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
    mvbOutlier = vector<bool>(N,false);
    mmProjectPoints.clear();// = map<long unsigned int, cv::Point2f>(N, static_cast<cv::Point2f>(NULL));
    mmMatchedInImage.clear();


    // This is done only for the first Frame (or after a change in the calibration)
        //  Step 5 计算去畸变后图像边界，将特征点分配到网格中。这个过程一般是在第一帧或者是相机标定参数发生变化之后进行
    if(mbInitialComputations)
    {
        // 计算去畸变后图像的边界
        ComputeImageBounds(imLeft);
        // 表示一个图像像素相当于多少个图像网格列（宽）
        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/(mnMaxX-mnMinX);
        // 表示一个图像像素相当于多少个图像网格行（高）
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/(mnMaxY-mnMinY);



        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        // 猜测是因为这种除法计算需要的时间略长，所以这里直接存储了这个中间计算结果
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;
        // 特殊的初始化过程完成，标志复位
        mbInitialComputations=false;
    }
        // 双目相机基线长度
    mb = mbf/fx;

    if(pPrevF)
    {
        if(!pPrevF->mVw.empty())
            mVw = pPrevF->mVw.clone();
    }
    else
    {
        mVw = cv::Mat::zeros(3,1,CV_32F);
    }

    AssignFeaturesToGrid();

    mpMutexImu = new std::mutex();

    //Set no stereo fisheye information
    Nleft = -1;
    Nright = -1;
    mvLeftToRightMatch = vector<int>(0);
    mvRightToLeftMatch = vector<int>(0);
    mTlr = cv::Mat(3,4,CV_32F);
    mTrl = cv::Mat(3,4,CV_32F);
    mvStereo3Dpoints = vector<cv::Mat>(0);
    monoLeft = -1;
    monoRight = -1;
}

// Constructor for stereo cameras with lines双目加线
/// 立体匹配模式下的双目+线 myline 添加左右目线提取器的参数 mylk 双目的frame构造函数
Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight,vector<Detection> &mmDetectMap,const int &mSensor, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, Lineextractor* LineextractorLeft, Lineextractor* LineextractorRight, ORBVocabulary* voc, LineVocabulary* voc_l, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth, GeometricCamera* pCamera, Frame* pPrevF, const IMU::Calib &ImuCalib)
    :mpcpi(NULL), mpORBvocabulary(voc), mpLinevocabulary(voc_l), mpORBextractorLeft(extractorLeft),mpORBextractorRight(extractorRight),
     mpLineextractorLeft(LineextractorLeft),mpLineextractorRight(LineextractorRight),
     mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
     mImuCalib(ImuCalib), mpImuPreintegrated(NULL), mpPrevFrame(pPrevF), mpImuPreintegratedFrame(NULL), mpReferenceKF(static_cast<KeyFrame*>(NULL)), mbImuPreintegrated(false),
     mpCamera(pCamera) ,mpCamera2(nullptr), mTimeStereoMatch(0), mTimeStereoMatch_Lines(0), mTimeORB_Ext(0), mTimeLines_Ext(0),fSensor(mSensor)
{

//    mylk 将传入的 sensor 参数赋值给成员变量 fSensor。

    inv_width  = FRAME_GRID_COLS / static_cast<double>(imLeft.cols);//64/376
    inv_height = FRAME_GRID_ROWS / static_cast<double>(imRight.rows);//48/1241

    // Frame ID
    // Step 1 帧的ID 自增
    mnId=nNextId++;

    // Scale Level Info
    // Step 2 计算图像金字塔的参数
    // 获取图像金字塔的层数
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    // 获得层与层之间的缩放比
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    // 计算上面缩放比的对数
    mfLogScaleFactor = log(mfScaleFactor);
    // 获取每层图像的缩放因子
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    // 同样获取每层图像缩放因子的倒数
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    // 高斯模糊的时候，使用的方差
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    // 获取sigma^2的倒数
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();
    //mylk 新增
    fx = K.at<float>(0,0);
    fy = K.at<float>(1,1);
    cx = K.at<float>(0,2);
    cy = K.at<float>(1,2);

    // ORB extraction
#ifdef SAVE_TIMES
    std::chrono::steady_clock::time_point time_StartExtORB = std::chrono::steady_clock::now();
#endif
        // Step 3 对左目右目图像提取ORB特征点, 第一个参数0-左图， 1-右图。为加速计算，同时开了两个线程
    thread threadLeft(&Frame::ExtractORBKeyPoints,this,0,imLeft,0,0);
        // 对右目图像提取orb特征
    thread threadRight(&Frame::ExtractORBKeyPoints,this,1,imRight,0,0);
        // 等待两张图像特征点提取过程完成
    threadLeft.join();
    threadRight.join();
#ifdef SAVE_TIMES
    std::chrono::steady_clock::time_point time_EndExtORB = std::chrono::steady_clock::now();

    mTimeORB_Ext = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndExtORB - time_StartExtORB).count();
#endif
        //mylk 新增
        cv::Mat  imGrayT = imLeft;
        // Calculate the dynamic abnormal points and output the T matrix
        //有数据表示不是第一帧数据
        if(imGrayPre.data)
        {
            std::chrono::steady_clock::time_point tm1 = std::chrono::steady_clock::now();
            ProcessMovingObject(imLeft,mmDetectMap);
            std::chrono::steady_clock::time_point tm2 = std::chrono::steady_clock::now();
            //movingDetectTime= std::chrono::duration_cast<std::chrono::duration<double> >(tm2 - tm1).count();
            //经过光流极线约束后将当前帧保存到imGrayPre，进入下个循环，
            //imGrayPre，即为上一帧数据
            std::swap(imGrayPre, imGrayT);
        }
        else
        {
            std::swap(imGrayPre, imGrayT);
            flag_mov=0;
        }

    // Line Extraction
#ifdef SAVE_TIMES
    std::chrono::steady_clock::time_point time_StartExtLines = std::chrono::steady_clock::now();
#endif
    thread threadLeft_Line(&Frame::ExtractLine,this,0,imLeft);
    thread threadRight_Line(&Frame::ExtractLine,this,1,imRight);
    threadLeft_Line.join();
    threadRight_Line.join();
#ifdef SAVE_TIMES
    std::chrono::steady_clock::time_point time_EndExtLines = std::chrono::steady_clock::now();

    mTimeLines_Ext = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndExtLines - time_StartExtLines).count();
    mTimeTotal_Ext = mTimeORB_Ext + mTimeLines_Ext;
#endif

//    SelectLineObject(semantic_img);
        //myline 将线特征的参数报存
    mnScaleLevels_l = mpLineextractorLeft->nlevels_l;
    mvScaleFactors_l = mpLineextractorLeft->mvScaleFactor_l;
    mvInvScaleFactors_l = mpLineextractorLeft->mvInvScaleFactor_l;
    mvLevelSigma2_l =  mpLineextractorLeft->mvLevelSigma2_l;
    mvInvLevelSigma2_l = mpLineextractorLeft->mvInvLevelSigma2_l;


    //myline 保存线特征的数量
    N_l = mvKeys_Line.size();

    UndistortKeyLines();
#ifdef SAVE_TIMES
    std::chrono::steady_clock::time_point time_StartStereoMatches_Lines = std::chrono::steady_clock::now();
#endif
    ComputeStereoMatches_Lines();
#ifdef SAVE_TIMES
    std::chrono::steady_clock::time_point time_EndStereoMatches_Lines = std::chrono::steady_clock::now();

    mTimeStereoMatch_Lines = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndStereoMatches_Lines - time_StartStereoMatches_Lines).count();
    mTimeStereoMatchTotal = mTimeStereoMatch + mTimeStereoMatch_Lines;
#endif
    
 
    mvpMapLines = vector<MapLine*>(N_l,static_cast<MapLine*>(NULL));
    mvbOutlier_Line = vector<bool>(N_l,false);


}


//myline + rgbd的构造函数 mylk +lk mylk+mmDetectMap
Frame::Frame(const cv::Mat &rawImage, const cv::Mat &imGray, const cv::Mat &imDepth,const cv::Mat &img_seg_mask, vector<Detection> &mmDetectMap,const int &mSensor,const double &timeStamp,
             ORBextractor* extractor,Lineextractor* Lineextractor,ORBVocabulary* voc, LineVocabulary* voc_l,
             cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth,
             GeometricCamera* pCamera,const cv::Mat &grayimg, const cv::Mat &rgbimg,
             Frame* pPrevF, const IMU::Calib &ImuCalib)
        :mpcpi(NULL),mpORBvocabulary(voc),mpORBextractorLeft(extractor),mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
         mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
         mImuCalib(ImuCalib), mpImuPreintegrated(NULL), mpPrevFrame(pPrevF), mpImuPreintegratedFrame(NULL), mpReferenceKF(static_cast<KeyFrame*>(NULL)), mbImuPreintegrated(false),
         mpCamera(pCamera),mpCamera2(nullptr), mTimeStereoMatch(0), mTimeORB_Ext(0),
         mpLineextractorLeft(Lineextractor),mpLinevocabulary(voc_l),_img_seg_mask(img_seg_mask),// myplan
         mTimeStereoMatch_Lines(0), mTimeLines_Ext(0),fSensor(mSensor),//imp 第三次
    mColorImage(rawImage.clone()), mQuadricImage(rawImage.clone()),im_(grayimg.clone()),rgb_(rgbimg.clone())
{
    //mylk fSensor成员使用传递给构造函数的 mSensor 参数进行初始化
//std::cout<< "already in frame"<<std::endl;


    //MYLINE 新增网格匹配相关
    inv_width  = FRAME_GRID_COLS / static_cast<double>(imGray.cols);
    inv_height = FRAME_GRID_ROWS / static_cast<double>(imDepth.rows);
    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    // Step 2 计算图像金字塔的参数
    // 获取图像金字塔的层数
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    //mylk 新增
    fx = K.at<float>(0,0);
    fy = K.at<float>(1,1);
    cx = K.at<float>(0,2);
    cy = K.at<float>(1,2);

    // ORB extractionLineFeatures extraction
#ifdef SAVE_TIMES
    std::chrono::steady_clock::time_point time_StartExtORB = std::chrono::steady_clock::now();
#endif
    ExtractORBKeyPoints(0,imGray,0,0);//mylk 这里值进行构建金字塔和四叉树分配
    //在 StereoCalculEverything 首先进行特征点光流基础矩阵极线约束再判断动态点是否在动态类别框内
    //是，剔除动态框内的特征点

    //mylk 新增
    cv::Mat  imGrayT = imGray;
    // Calculate the dynamic abnormal points and output the T matrix
    if(imGrayPre.data)
    {
        std::chrono::steady_clock::time_point tm1 = std::chrono::steady_clock::now();
        // 首先进行特征点光流基础矩阵极线约束
        ProcessMovingObject(imGray,mmDetectMap);
        std::chrono::steady_clock::time_point tm2 = std::chrono::steady_clock::now();
        //movingDetectTime= std::chrono::duration_cast<std::chrono::duration<double> >(tm2 - tm1).count();
        std::swap(imGrayPre, imGrayT);
    }
    else
    {
        std::swap(imGrayPre, imGrayT);
        flag_mov=0;
    }



        ExtractLine(0,imGray); //myline



#ifdef SAVE_TIMES
    std::chrono::steady_clock::time_point time_EndExtORB = std::chrono::steady_clock::now();

mTimeORB_Ext = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndExtORB - time_StartExtORB).count();
#endif

    // 获取特征点的个数

    N_l = mvKeys_Line.size(); //myline

    //myline 保存线特征的数量
    //myline 将线特征的参数报存
    mnScaleLevels_l = mpLineextractorLeft->nlevels_l;
    mvScaleFactors_l = mpLineextractorLeft->mvScaleFactor_l;
    mvInvScaleFactors_l = mpLineextractorLeft->mvInvScaleFactor_l;
    mvLevelSigma2_l =  mpLineextractorLeft->mvLevelSigma2_l;
    mvInvLevelSigma2_l = mpLineextractorLeft->mvInvLevelSigma2_l;

    UndistortKeyLines();
    ////myplan 新增函数
//    filter_lines(keylines_raw, keylines_out);将线保存在all_lines_mat
    keylines_to_mat(mvKeysUn_Line, all_lines_mat,1);

    Eigen::MatrixXd all_lines_raw(all_lines_mat.rows,4);
    for (int rr=0;rr<all_lines_mat.rows;rr++)
        for (int cc=0;cc<4;cc++)
            all_lines_raw(rr,cc) = all_lines_mat.at<float>(rr,cc);
    // save to frame.
    all_lines_eigen = all_lines_raw;


        ComputeRGBDMatches_Lines(imDepth); //myline


//        std::cout << "after line extractor " << std::endl;
#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_StartStereoMatches = std::chrono::steady_clock::now();
#endif


    // 初始化本帧的地图点

    mvpMapLines = vector<MapLine*>(N_l,static_cast<MapLine*>(NULL));//myline

    mvbOutlier_Line = vector<bool>(N_l,false);//myline


}

//单目的构造函数
Frame::Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, GeometricCamera* pCamera, cv::Mat &distCoef, const float &bf, const float &thDepth, Frame* pPrevF, const IMU::Calib &ImuCalib)
    :mpcpi(NULL),mpORBvocabulary(voc),mpORBextractorLeft(extractor),mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
     mTimeStamp(timeStamp), mK(static_cast<Pinhole*>(pCamera)->toK()), mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
     mImuCalib(ImuCalib), mpImuPreintegrated(NULL),mpPrevFrame(pPrevF),mpImuPreintegratedFrame(NULL), mpReferenceKF(static_cast<KeyFrame*>(NULL)), mbImuPreintegrated(false), mpCamera(pCamera),
     mpCamera2(nullptr), mTimeStereoMatch(0), mTimeORB_Ext(0)
{
    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
#ifdef SAVE_TIMES
    std::chrono::steady_clock::time_point time_StartExtORB = std::chrono::steady_clock::now();
#endif
    ExtractORB(0,imGray,0,1000);
#ifdef SAVE_TIMES
    std::chrono::steady_clock::time_point time_EndExtORB = std::chrono::steady_clock::now();

    mTimeORB_Ext = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndExtORB - time_StartExtORB).count();
#endif


    N = mvKeys.size();
    if(mvKeys.empty())
        return;

    UndistortKeyPoints();

    // Set no stereo information
    mvuRight = vector<float>(N,-1);
    mvDepth = vector<float>(N,-1);
    mnCloseMPs = 0;

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));

    mmProjectPoints.clear();// = map<long unsigned int, cv::Point2f>(N, static_cast<cv::Point2f>(NULL));
    mmMatchedInImage.clear();

    mvbOutlier = vector<bool>(N,false);

    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imGray);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

        fx = static_cast<Pinhole*>(mpCamera)->toK().at<float>(0,0);
        fy = static_cast<Pinhole*>(mpCamera)->toK().at<float>(1,1);
        cx = static_cast<Pinhole*>(mpCamera)->toK().at<float>(0,2);
        cy = static_cast<Pinhole*>(mpCamera)->toK().at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }


    mb = mbf/fx;

    //Set no stereo fisheye information
    Nleft = -1;
    Nright = -1;
    mvLeftToRightMatch = vector<int>(0);
    mvRightToLeftMatch = vector<int>(0);
    mTlr = cv::Mat(3,4,CV_32F);
    mTrl = cv::Mat(3,4,CV_32F);
    mvStereo3Dpoints = vector<cv::Mat>(0);
    monoLeft = -1;
    monoRight = -1;

    AssignFeaturesToGrid();

    // mVw = cv::Mat::zeros(3,1,CV_32F);
    if(pPrevF)
    {
        if(!pPrevF->mVw.empty())
            mVw = pPrevF->mVw.clone();
    }
    else
    {
        mVw = cv::Mat::zeros(3,1,CV_32F);
    }

    mpMutexImu = new std::mutex();
}

bool Frame::CheckSame(const cv::Vec3b& intensity, const std::unordered_map<std::string, int>& rgb) {
    return rgb.at("b") == (int)intensity.val[0] && rgb.at("g") == (int)intensity.val[1] && rgb.at("r") == (int)intensity.val[2];
}

void Frame::SelectObject(const cv::Mat& semantic_img) {
    std::vector<cv::KeyPoint> tmpKeys;
    cv::Mat descriptors = mDescriptors.clone();
    int size_before = mvKeys.size();
    std::cout << "size before " << size_before;
    int k = 0;
    for (int i = 0; i < mvKeys.size(); ++i) {
        int col = mvKeys[i].pt.x;
        int row = mvKeys[i].pt.y;
        if (CheckDynamicPoint(semantic_img, col, row)) {
            continue;
        }
        tmpKeys.push_back(mvKeys[i]);
        mDescriptors.row(i).copyTo(descriptors.row(k));
        k++;
    }
    mvKeys = tmpKeys;
    cv::Rect rect = cv::Rect(0, 0, 32, k);
    mDescriptors = descriptors(rect);
    int size_after = mvKeys.size();
    std::cout << "size after " << size_after << std::endl;;
}

bool Frame::CheckDynamicPoint(const cv::Mat& semantic_img, const int col, const int row) {
        int border_width = 5;
        if (col < border_width || row < border_width || col >= semantic_img.cols - border_width || row >= semantic_img.rows - border_width) {
            // std::cout << "line key point start col " << col << " row " << row << " in border " << std::endl;
            return true;
        }
        cv::Vec3b intensity = semantic_img.at<cv::Vec3b>(row, col);
        cv::Vec3b left_intensity = semantic_img.at<cv::Vec3b>(row, col - border_width);
        cv::Vec3b right_intensity = semantic_img.at<cv::Vec3b>(row, col + border_width);
        cv::Vec3b bottom_intensity = semantic_img.at<cv::Vec3b>(row - border_width, col);
        cv::Vec3b top_intensity = semantic_img.at<cv::Vec3b>(row + border_width, col);
        if (CheckSame(intensity, color_map["car"]) || CheckSame(left_intensity, color_map["car"])
        || CheckSame(right_intensity, color_map["car"]) || CheckSame(top_intensity, color_map["car"])
        || CheckSame(bottom_intensity, color_map["car"])) {
            // std::cout << (int)intensity.val[0] << (int)intensity.val[1] << (int)intensity.val[2] << " "; 
            return true;
        }
        return false;
}

void Frame::SelectLineObject(const cv::Mat& semantic_img) {
    std::vector<cv::line_descriptor::KeyLine> tmpKeys;
    cv::Mat descriptors = mDescriptors_Line.clone();
    int size_before = mvKeys_Line.size();
    std::cout << "size before " << size_before;
    int k = 0;
    for (int i = 0; i < mvKeys_Line.size(); ++i) {
        int col = mvKeys_Line[i].startPointX;
        int row = mvKeys_Line[i].startPointY;
        if (CheckDynamicPoint(semantic_img, col, row)) {
            continue;
        }
        col = mvKeys_Line[i].endPointX;
        row = mvKeys_Line[i].endPointY;
        if (CheckDynamicPoint(semantic_img, col, row)) {
            continue;
        }
        tmpKeys.push_back(mvKeys_Line[i]);
        mDescriptors_Line.row(i).copyTo(descriptors.row(k));
        k++;
    }
    mvKeys_Line = tmpKeys;
    cv::Rect rect = cv::Rect(0, 0, 32, k);
    mDescriptors_Line = descriptors(rect);
    int size_after = mvKeys_Line.size();
    std::cout << "size after " << size_after << std::endl;;
}

void Frame::AssignFeaturesToGrid()
{
    // Fill matrix with points
    const int nCells = FRAME_GRID_COLS*FRAME_GRID_ROWS;

    int nReserve = 0.5f*N/(nCells);

    for(unsigned int i=0; i<FRAME_GRID_COLS;i++)
        for (unsigned int j=0; j<FRAME_GRID_ROWS;j++){
            mGrid[i][j].reserve(nReserve);
            if(Nleft != -1){
                mGridRight[i][j].reserve(nReserve);
            }
        }



    for(int i=0;i<N;i++)
    {
        const cv::KeyPoint &kp = (Nleft == -1) ? mvKeysUn[i]
                                                 : (i < Nleft) ? mvKeys[i]
                                                                 : mvKeysRight[i - Nleft];

        int nGridPosX, nGridPosY;
        if(PosInGrid(kp,nGridPosX,nGridPosY)){
            if(Nleft == -1 || i < Nleft)
                mGrid[nGridPosX][nGridPosY].push_back(i);
            else
                mGridRight[nGridPosX][nGridPosY].push_back(i - Nleft);
        }
    }
}

/**
 * @brief 赋值新的偏置
 * @param flag 左右标志位
 * @param im 图片
 * @param x0 界限
 * @param x1 界限
 */
    //mylk 提取器变量改动
void Frame::ExtractORB(int flag, const cv::Mat &im, const int x0, const int x1)
{
    vector<int> vLapping = {x0,x1};
    // 判断是左图还是右图
    if(flag==0)
        // 左图的话就套使用左图指定的特征点提取器，并将提取结果保存到对应的变量中
//        monoLeft = (*mpORBextractorLeft)(im,cv::Mat(),mvKeys,mDescriptors,vLapping);
        (*mpORBextractorLeft)(im,cv::Mat(),mvKeysTemp,mDescriptors,vLapping);
    else
        // 右图的话就需要使用右图指定的特征点提取器，并将提取结果保存到对应的变量中
//        monoRight = (*mpORBextractorRight)(im,cv::Mat(),mvKeysRight,mDescriptorsRight,vLapping);
        (*mpORBextractorRight)(im,cv::Mat(),mvRigghtKeysTemp,mDescriptorsRight,vLapping);
}

///mylk 计算RGB相机的
void Frame::CalculEverything( cv::Mat &imRGB, const cv::Mat &imGray,const cv::Mat &imDepth,cv::Mat &K,vector<Detection> mmDetectMap)
    {
        int flagprocess = 0;
        //对边界框遍历
        for(auto det : mmDetectMap)
        {
            if(mClassnames[det.class_idx] == "person"||mClassnames[det.class_idx] == "car")
            {
//                flagprocess=0;//IMP 消融实验 0
                flagprocess=1;
                break;
            }
        }
        //首先对特征点周围的灰度值进行遍历筛选出质量高的特征点，然后根据alcOpticalFlowPyrLK计算稀疏光流
        // 计算遍历特征点在前一帧和当前帧特征点在周围像素的差异判断特征点质量，筛选出追踪比较好的特征点
        //然后计算基础矩阵化，计算出当前帧特征点距离极线的距离，将当前帧特征点距离极线距离大于阈值的特征
        //点保存在T_M中，进行进一步操作，如果大于动态阈值的特征点并在动态框内直接删除
        if(!T_M.empty() && flagprocess )
        {
            std::chrono::steady_clock::time_point tc1 = std::chrono::steady_clock::now();
            //如果大于动态阈值的特征点并在动态框内直接删除
            flag_mov = mpORBextractorLeft->CheckMovingKeyPoints(imGray,mmDetectMap,mvKeysTemp,T_M);
//            std::cout<<fSensor<<std::endl;
            std::chrono::steady_clock::time_point tc2 = std::chrono::steady_clock::now();
            double tc= std::chrono::duration_cast<std::chrono::duration<double> >(tc2 - tc1).count();
            cout << "check time =" << tc*1000 <<  endl;
        }
        ExtractORBDesp(0,imGray,0,0);
        N = mvKeys.size();
        if(mvKeys.empty())
            return;
        UndistortKeyPoints();
        ComputeStereoFromRGBD(imDepth);
        // 将处理后的本帧的地图点存在容器中
        mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
        mvbOutlier = vector<bool>(N,false);

        // This is done only for the first Frame (or after a  calibration)
        mmProjectPoints.clear();
        mmMatchedInImage.clear();

        // 记录地图点是否为外点，初始化均为外点false
        mvbOutlier = vector<bool>(N,false);
        // This is done only for the first Frame (or after a change in the calibration)
        //  Step 5 计算去畸变后图像边界，将特征点分配到网格中。这个过程一般是在第一帧或者是相机标定参数发生变化之后进行
        if(mbInitialComputations)
        {
            // 计算去畸变后图像的边界
            ComputeImageBounds(imGray);

            // 表示一个图像像素相当于多少个图像网格列（宽）
            mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
            // 表示一个图像像素相当于多少个图像网格行（高）
            mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);
            // 猜测是因为这种除法计算需要的时间略长，所以这里直接存储了这个中间计算结果
            invfx = 1.0f/fx;
            invfy = 1.0f/fy;
            // 特殊的初始化过程完成，标志复位
            mbInitialComputations=false;
        }
        // 计算假想的基线长度 baseline= mbf/fx
        // 后面要对从RGBD相机输入的特征点,结合相机基线长度,焦距,以及点的深度等信息来计算其在假想的"右侧图像"上的匹配点
        Nleft = -1;
        Nright = -1;
        mvLeftToRightMatch = vector<int>(0);
        mvRightToLeftMatch = vector<int>(0);
        mTlr = cv::Mat(3,4,CV_32F);
        mTrl = cv::Mat(3,4,CV_32F);
        mvStereo3Dpoints = vector<cv::Mat>(0);
        monoLeft = -1;
        monoRight = -1;
        // 将特征点分配到图像网格中
        AssignFeaturesToGrid();

    }

///mylk 双目新增函数
void Frame::StereoCalculEverything( const cv::Mat &imGray,const cv::Mat &imRight,vector<Detection> mmDetectMap)
{
    int flagprocess = 0;
    for(auto det : mmDetectMap)
    {
        if(mClassnames[det.class_idx] == "person"||mClassnames[det.class_idx] == "car")
        {
            flagprocess=1;
//            flagprocess=0;//IMP 消融实验 0

            break;
        }
    }
    if(!T_M.empty() && flagprocess )
    {
        std::chrono::steady_clock::time_point tc1 = std::chrono::steady_clock::now();
        flag_mov = mpORBextractorLeft->CheckMovingKeyPoints(imGray,mmDetectMap,mvKeysTemp,T_M);
        //flag_mov = mpORBextractorLeft->CheckMovingKeyPoints(imRight,mmDetectMap,mvRigghtKeysTemp,T_M);
        std::chrono::steady_clock::time_point tc2 = std::chrono::steady_clock::now();
        double tc= std::chrono::duration_cast<std::chrono::duration<double> >(tc2 - tc1).count();
        cout << "check time =" << tc*1000 <<  endl;
    }
    thread threadLeftDesp(&Frame::ExtractStereoORBDesp,this,0,imGray,imGray,0,0);
    thread threadRightDesp(&Frame::ExtractStereoORBDesp,this,1,imRight,imRight,0,0);
    threadLeftDesp.join();
    threadRightDesp.join();
    N = mvKeys.size();
    if(mvKeys.empty())
        return;
    UndistortKeyPoints();
    ComputeStereoMatches();
    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
    mvbOutlier = vector<bool>(N,false);
    // This is done only for the first Frame (or after a change in the calibration)
    // 初始化本帧的地图点
    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));

    mmProjectPoints.clear();
    mmMatchedInImage.clear();
    // 记录地图点是否为外点，初始化均为外点false
    mvbOutlier = vector<bool>(N,false);
    // This is done only for the first Frame (or after a change in the calibration)
    //  Step 5 计算去畸变后图像边界，将特征点分配到网格中。这个过程一般是在第一帧或者是相机标定参数发生变化之后进行
    if(mbInitialComputations)
    {
        // 计算去畸变后图像的边界
        ComputeImageBounds(imGray);
        // 表示一个图像像素相当于多少个图像网格列（宽）
        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
        // 表示一个图像像素相当于多少个图像网格行（高）
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);
        // 猜测是因为这种除法计算需要的时间略长，所以这里直接存储了这个中间计算结果
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;
        // 特殊的初始化过程完成，标志复位
        mbInitialComputations=false;
    }
    // 计算假想的基线长度 baseline= mbf/fx
    // 后面要对从RGBD相机输入的特征点,结合相机基线长度,焦距,以及点的深度等信息来计算其在假想的"右侧图像"上的匹配点
    mb = mbf/fx;
    //Set no stereo fisheye information
    Nleft = -1;
    Nright = -1;
    mvLeftToRightMatch = vector<int>(0);
    mvRightToLeftMatch = vector<int>(0);
//    mvStereo3Dpoints = vector<Eigen::Vector3f>(0);
    std::vector<cv::Mat> mvStereo3Dpoints(0);
    monoLeft = -1;
    monoRight = -1;
    // 将特征点分配到图像网格中
    AssignFeaturesToGrid();
}
///mylk 新增加函数
void Frame::ExtractORBKeyPoints(int flag, const cv::Mat &im, const int x0, const int x1)
{
    vector<int> vLapping = {x0,x1};
    if(flag==0)
    {
        (*mpORBextractorLeft)(im,cv::Mat(),mvKeysTemp,mDescriptors,vLapping);
    }
    else
        (*mpORBextractorRight)(im,cv::Mat(),mvRigghtKeysTemp,mDescriptorsRight,vLapping);
}
///

//mylk 计算双目相机的
void Frame::ExtractStereoORBDesp(int flag,const cv::Mat &imgray, const cv::Mat &imRight,const int x0, const int x1)
{
    vector<int> vLapping = {x0,x1};
    if(flag==0)
        monoLeft = (*mpORBextractorLeft).ProcessDesp(imgray,cv::Mat(),mvKeysTemp,mvKeys,mDescriptors,vLapping);
    else
        monoRight = (*mpORBextractorRight).ProcessDesp(imRight,cv::Mat(),mvRigghtKeysTemp,mvKeysRight,mDescriptorsRight,vLapping);

}

//////mylk 计算RGB相机的
void Frame::ExtractORBDesp(int flag,const cv::Mat &imgray, const int x0, const int x1)
{
    vector<int> vLapping = {x0,x1};
    if(flag==0)
        monoLeft = (*mpORBextractorLeft).ProcessDesp(imgray,cv::Mat(),mvKeysTemp,mvKeys,mDescriptors,vLapping);
    else
        monoRight = (*mpORBextractorRight).ProcessDesp(imgray,cv::Mat(),mvRigghtKeysTemp,mvKeysRight,mDescriptorsRight,vLapping);

}


//myline
void Frame::ExtractLine(int flag, const cv::Mat &im)//MYLINE
{
    if(flag==0)
        (*mpLineextractorLeft)(im,cv::Mat(),mvKeys_Line,mDescriptors_Line);
    else
        (*mpLineextractorRight)(im,cv::Mat(),mvKeysRight_Line,mDescriptorsRight_Line);
}


void Frame::SetPose(cv::Mat Tcw)
{
    mTcw = Tcw.clone();
    UpdatePoseMatrices();
}

void Frame::GetPose(cv::Mat &Tcw)
{
    Tcw = mTcw.clone();
}

void Frame::SetNewBias(const IMU::Bias &b)
{
    mImuBias = b;
    if(mpImuPreintegrated)
        mpImuPreintegrated->SetNewBias(b);
}

void Frame::SetVelocity(const cv::Mat &Vwb)
{
    mVw = Vwb.clone();
}

void Frame::SetImuPoseVelocity(const cv::Mat &Rwb, const cv::Mat &twb, const cv::Mat &Vwb)
{
    mVw = Vwb.clone();
    cv::Mat Rbw = Rwb.t();
    cv::Mat tbw = -Rbw*twb;
    cv::Mat Tbw = cv::Mat::eye(4,4,CV_32F);
    Rbw.copyTo(Tbw.rowRange(0,3).colRange(0,3));
    tbw.copyTo(Tbw.rowRange(0,3).col(3));
    mTcw = mImuCalib.Tcb*Tbw;
    UpdatePoseMatrices();
}



void Frame::UpdatePoseMatrices()
{
    mRcw = mTcw.rowRange(0,3).colRange(0,3);
    mRwc = mRcw.t();
    mtcw = mTcw.rowRange(0,3).col(3);
    mOw = -mRcw.t()*mtcw;
}

cv::Mat Frame::GetImuPosition()
{
    return mRwc*mImuCalib.Tcb.rowRange(0,3).col(3)+mOw;
}

cv::Mat Frame::GetImuRotation()
{
    return mRwc*mImuCalib.Tcb.rowRange(0,3).colRange(0,3);
}

cv::Mat Frame::GetImuPose()
{
    cv::Mat Twb = cv::Mat::eye(4,4,CV_32F);
    Twb.rowRange(0,3).colRange(0,3) = mRwc*mImuCalib.Tcb.rowRange(0,3).colRange(0,3);
    Twb.rowRange(0,3).col(3) = mRwc*mImuCalib.Tcb.rowRange(0,3).col(3)+mOw;
    return Twb.clone();
}


bool Frame::isInFrustum(MapPoint *pMP, float viewingCosLimit)
{
    if(Nleft == -1){
        // cout << "\na";
        pMP->mbTrackInView = false;
        pMP->mTrackProjX = -1;
        pMP->mTrackProjY = -1;

        // 3D in absolute coordinates
        cv::Mat P = pMP->GetWorldPos();

        // cout << "b";

        // 3D in camera coordinates
        const cv::Mat Pc = mRcw*P+mtcw;
        const float Pc_dist = cv::norm(Pc);

        // Check positive depth
        const float &PcZ = Pc.at<float>(2);
        const float invz = 1.0f/PcZ;
        if(PcZ<0.0f)
            return false;

        const cv::Point2f uv = mpCamera->project(Pc);

        // cout << "c";

        if(uv.x<mnMinX || uv.x>mnMaxX)
            return false;
        if(uv.y<mnMinY || uv.y>mnMaxY)
            return false;

        // cout << "d";
        pMP->mTrackProjX = uv.x;
        pMP->mTrackProjY = uv.y;

        // Check distance is in the scale invariance region of the MapPoint
        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        const cv::Mat PO = P-mOw;
        const float dist = cv::norm(PO);

        if(dist<minDistance || dist>maxDistance)
            return false;

        // cout << "e";

        // Check viewing angle
        cv::Mat Pn = pMP->GetNormal();

        // cout << "f";

        const float viewCos = PO.dot(Pn)/dist;

        if(viewCos<viewingCosLimit)
            return false;

        // Predict scale in the image
        const int nPredictedLevel = pMP->PredictScale(dist,this);

        // cout << "g";

        // Data used by the tracking
        pMP->mbTrackInView = true;
        pMP->mTrackProjX = uv.x;
        pMP->mTrackProjXR = uv.x - mbf*invz;

        pMP->mTrackDepth = Pc_dist;
        // cout << "h";

        pMP->mTrackProjY = uv.y;
        pMP->mnTrackScaleLevel= nPredictedLevel;
        pMP->mTrackViewCos = viewCos;

        // cout << "i";

        return true;
    }
    else{
        pMP->mbTrackInView = false;
        pMP->mbTrackInViewR = false;
        pMP -> mnTrackScaleLevel = -1;
        pMP -> mnTrackScaleLevelR = -1;

        pMP->mbTrackInView = isInFrustumChecks(pMP,viewingCosLimit);
        pMP->mbTrackInViewR = isInFrustumChecks(pMP,viewingCosLimit,true);

        return pMP->mbTrackInView || pMP->mbTrackInViewR;
    }
}


bool Frame::isInFrustum_l(MapLine *pML, float viewingCosLimit)
{
    pML->mbTrackInView = false;

    // 3D in absolute coordinates
    Vector6d sep = pML->GetWorldPos();
    Vector3d sp_eigen = sep.head(3);
    Vector3d ep_eigen = sep.tail(3);
    cv::Mat sp = Converter::toCvMat(sp_eigen);
    cv::Mat ep = Converter::toCvMat(ep_eigen);
    {
        // 3D in camera coordinates
        const cv::Mat Pc = mRcw*sp+mtcw;
        const float &PcX = Pc.at<float>(0);
        const float &PcY= Pc.at<float>(1);
        const float &PcZ = Pc.at<float>(2);

        // Check positive depth
        if(PcZ<0.0f)
            return false;

        // Project in image and check it is not outside
        const float invz = 1.0f/PcZ;
        const float u=fx*PcX*invz+cx;
        const float v=fy*PcY*invz+cy;

        if(u<mnMinX || u>mnMaxX)
            return false;
        if(v<mnMinY || v>mnMaxY)
            return false;

        pML->mTrackProjsX = u;
        pML->mTrackProjsY = v;
    }
    {
        // 3D in camera coordinates
        const cv::Mat Pc = mRcw*ep+mtcw;
        const float &PcX = Pc.at<float>(0);
        const float &PcY= Pc.at<float>(1);
        const float &PcZ = Pc.at<float>(2);

        // Check positive depth
        if(PcZ<0.0f)
            return false;

        // Project in image and check it is not outside
        const float invz = 1.0f/PcZ;
        const float u=fx*PcX*invz+cx;
        const float v=fy*PcY*invz+cy;

        if(u<mnMinX || u>mnMaxX)
            return false;
        if(v<mnMinY || v>mnMaxY)
            return false;

        pML->mTrackProjeX = u;
        pML->mTrackProjeY = v;
    }

    Vector3d MidPoint = (sp_eigen+ep_eigen)/2;
    cv::Mat P = Converter::toCvMat(MidPoint);

    // Check distance is in the scale invariance region of the MapPoint
    const float maxDistance = pML->GetMaxDistanceInvariance();
    const float minDistance = pML->GetMinDistanceInvariance();
    const cv::Mat PO = P-mOw;
    const float dist = cv::norm(PO);

    if(dist<minDistance || dist>maxDistance)
        return false;

    // Check viewing angle
    cv::Mat Pn = pML->GetNormal();

    const float viewCos = PO.dot(Pn)/dist;

    if(viewCos<viewingCosLimit)
        return false;

    // Data used by the tracking
    pML->mbTrackInView = true;
    pML->mnTrackangle = atan2(pML->mTrackProjeY - pML->mTrackProjsY, pML->mTrackProjeX - pML->mTrackProjsX);

    return true;
}

bool Frame::ProjectPointDistort(MapPoint* pMP, cv::Point2f &kp, float &u, float &v)
{

    // 3D in absolute coordinates
    cv::Mat P = pMP->GetWorldPos();

    // 3D in camera coordinates
    const cv::Mat Pc = mRcw*P+mtcw;
    const float &PcX = Pc.at<float>(0);
    const float &PcY= Pc.at<float>(1);
    const float &PcZ = Pc.at<float>(2);

    // Check positive depth
    if(PcZ<0.0f)
    {
        cout << "Negative depth: " << PcZ << endl;
        return false;
    }

    // Project in image and check it is not outside
    const float invz = 1.0f/PcZ;
    u=fx*PcX*invz+cx;
    v=fy*PcY*invz+cy;

    // cout << "c";

    if(u<mnMinX || u>mnMaxX)
        return false;
    if(v<mnMinY || v>mnMaxY)
        return false;

    float u_distort, v_distort;

    float x = (u - cx) * invfx;
    float y = (v - cy) * invfy;
    float r2 = x * x + y * y;
    float k1 = mDistCoef.at<float>(0);
    float k2 = mDistCoef.at<float>(1);
    float p1 = mDistCoef.at<float>(2);
    float p2 = mDistCoef.at<float>(3);
    float k3 = 0;
    if(mDistCoef.total() == 5)
    {
        k3 = mDistCoef.at<float>(4);
    }

    // Radial distorsion
    float x_distort = x * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);
    float y_distort = y * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);

    // Tangential distorsion
    x_distort = x_distort + (2 * p1 * x * y + p2 * (r2 + 2 * x * x));
    y_distort = y_distort + (p1 * (r2 + 2 * y * y) + 2 * p2 * x * y);

    u_distort = x_distort * fx + cx;
    v_distort = y_distort * fy + cy;


    u = u_distort;
    v = v_distort;

    kp = cv::Point2f(u, v);

    return true;
}

cv::Mat Frame::inRefCoordinates(cv::Mat pCw)
{
    return mRcw*pCw+mtcw;
}

vector<size_t> Frame::GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel, const int maxLevel, const bool bRight) const
{
    vector<size_t> vIndices;
    vIndices.reserve(N);

    float factorX = r;
    float factorY = r;

    /*cout << "fX " << factorX << endl;
    cout << "fY " << factorY << endl;*/

    const int nMinCellX = max(0,(int)floor((x-mnMinX-factorX)*mfGridElementWidthInv));
    if(nMinCellX>=FRAME_GRID_COLS)
    {
        return vIndices;
    }

    const int nMaxCellX = min((int)FRAME_GRID_COLS-1,(int)ceil((x-mnMinX+factorX)*mfGridElementWidthInv));
    if(nMaxCellX<0)
    {
        return vIndices;
    }

    const int nMinCellY = max(0,(int)floor((y-mnMinY-factorY)*mfGridElementHeightInv));
    if(nMinCellY>=FRAME_GRID_ROWS)
    {
        return vIndices;
    }

    const int nMaxCellY = min((int)FRAME_GRID_ROWS-1,(int)ceil((y-mnMinY+factorY)*mfGridElementHeightInv));
    if(nMaxCellY<0)
    {
        return vIndices;
    }

    const bool bCheckLevels = (minLevel>0) || (maxLevel>=0);

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const vector<size_t> vCell = (!bRight) ? mGrid[ix][iy] : mGridRight[ix][iy];
            if(vCell.empty())
                continue;

            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = (Nleft == -1) ? mvKeysUn[vCell[j]]
                                                         : (!bRight) ? mvKeys[vCell[j]]
                                                                     : mvKeysRight[vCell[j]];
                if(bCheckLevels)
                {
                    if(kpUn.octave<minLevel)
                        continue;
                    if(maxLevel>=0)
                        if(kpUn.octave>maxLevel)
                            continue;
                }

                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<factorX && fabs(disty)<factorY)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
{
    posX = round((kp.pt.x-mnMinX)*mfGridElementWidthInv);
    posY = round((kp.pt.y-mnMinY)*mfGridElementHeightInv);

    //Keypoint's coordinates are undistorted, which could cause to go out of the image
    if(posX<0 || posX>=FRAME_GRID_COLS || posY<0 || posY>=FRAME_GRID_ROWS)
        return false;

    return true;
}


void Frame::ComputeBoW()
{
    if(mBowVec.empty())
    {
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
        mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
    }
}

void Frame::UndistortKeyPoints()
{
    if(mDistCoef.at<float>(0)==0.0)
    {
        mvKeysUn=mvKeys;
        return;
    }

    // Fill matrix with points
    cv::Mat mat(N,2,CV_32F);

    for(int i=0; i<N; i++)
    {
        mat.at<float>(i,0)=mvKeys[i].pt.x;
        mat.at<float>(i,1)=mvKeys[i].pt.y;
    }

    // Undistort points
    mat=mat.reshape(2);
    cv::undistortPoints(mat,mat, static_cast<Pinhole*>(mpCamera)->toK(),mDistCoef,cv::Mat(),mK);
    mat=mat.reshape(1);


    // Fill undistorted keypoint vector
    mvKeysUn.resize(N);
    for(int i=0; i<N; i++)
    {
        cv::KeyPoint kp = mvKeys[i];
        kp.pt.x=mat.at<float>(i,0);
        kp.pt.y=mat.at<float>(i,1);
        mvKeysUn[i]=kp;
    }

}
///myline 线的去畸变函数
void Frame::UndistortKeyLines()
{
    if(mDistCoef.at<float>(0)==0.0)
    {
        mvKeysUn_Line = mvKeys_Line;
        return;
    }

    N_l = mvKeys_Line.size();   // update N_l

    // Fill matrix with points
    cv::Mat mat_s(N_l,2,CV_32F);
    cv::Mat mat_e(N_l,2,CV_32F);
    for(int i=0; i<N_l; i++)
    {
        mat_s.at<float>(i,0)=mvKeys_Line[i].startPointX;
        mat_s.at<float>(i,1)=mvKeys_Line[i].startPointY;
        mat_e.at<float>(i,0)=mvKeys_Line[i].endPointX;
        mat_e.at<float>(i,1)=mvKeys_Line[i].endPointY;
    }

    // Undistort points
    mat_s=mat_s.reshape(2);
    mat_e=mat_e.reshape(2);
    cv::undistortPoints(mat_s,mat_s,mK,mDistCoef,cv::Mat(),mK);
    cv::undistortPoints(mat_e,mat_e,mK,mDistCoef,cv::Mat(),mK);
    mat_s=mat_s.reshape(1);
    mat_e=mat_e.reshape(1);

    // Fill undistorted keypoint vector
    mvKeysUn_Line.resize(N_l);
    for(int i=0; i<N_l; i++)
    { 
        mvKeysUn_Line[i].startPointX=mat_s.at<float>(i,0);
        mvKeysUn_Line[i].startPointY=mat_s.at<float>(i,1);
        mvKeysUn_Line[i].endPointX=mat_e.at<float>(i,0);
        mvKeysUn_Line[i].endPointY=mat_e.at<float>(i,1);
    }  
}

void Frame::ComputeImageBounds(const cv::Mat &imLeft)
{
    if(mDistCoef.at<float>(0)!=0.0)
    {
        cv::Mat mat(4,2,CV_32F);
        mat.at<float>(0,0)=0.0; mat.at<float>(0,1)=0.0;
        mat.at<float>(1,0)=imLeft.cols; mat.at<float>(1,1)=0.0;
        mat.at<float>(2,0)=0.0; mat.at<float>(2,1)=imLeft.rows;
        mat.at<float>(3,0)=imLeft.cols; mat.at<float>(3,1)=imLeft.rows;

        mat=mat.reshape(2);
        cv::undistortPoints(mat,mat,static_cast<Pinhole*>(mpCamera)->toK(),mDistCoef,cv::Mat(),mK);
        mat=mat.reshape(1);

        // Undistort corners
        mnMinX = min(mat.at<float>(0,0),mat.at<float>(2,0));
        mnMaxX = max(mat.at<float>(1,0),mat.at<float>(3,0));
        mnMinY = min(mat.at<float>(0,1),mat.at<float>(1,1));
        mnMaxY = max(mat.at<float>(2,1),mat.at<float>(3,1));
    }
    else
    {
        mnMinX = 0.0f;
        mnMaxX = imLeft.cols;
        mnMinY = 0.0f;
        mnMaxY = imLeft.rows;
    }
}

void Frame::ComputeStereoMatches()
{
    mvuRight = vector<float>(N,-1.0f);
    mvDepth = vector<float>(N,-1.0f);

    const int thOrbDist = (ORBmatcher::TH_HIGH+ORBmatcher::TH_LOW)/2;

    const int nRows = mpORBextractorLeft->mvImagePyramid[0].rows;

    //Assign keypoints to row table
    vector<vector<size_t> > vRowIndices(nRows,vector<size_t>());

    for(int i=0; i<nRows; i++)
        vRowIndices[i].reserve(200);

    const int Nr = mvKeysRight.size();

    for(int iR=0; iR<Nr; iR++)
    {
        const cv::KeyPoint &kp = mvKeysRight[iR];
        const float &kpY = kp.pt.y;
        const float r = 2.0f*mvScaleFactors[mvKeysRight[iR].octave];
        const int maxr = ceil(kpY+r);
        const int minr = floor(kpY-r);

        for(int yi=minr;yi<=maxr;yi++)
            vRowIndices[yi].push_back(iR);
    }

    // Set limits for search
    const float minZ = mb;
    const float minD = 0;
    const float maxD = mbf/minZ;

    // For each left keypoint search a match in the right image
    vector<pair<int, int> > vDistIdx;
    vDistIdx.reserve(N);

    for(int iL=0; iL<N; iL++)
    {
        const cv::KeyPoint &kpL = mvKeys[iL];
        const int &levelL = kpL.octave;
        const float &vL = kpL.pt.y;
        const float &uL = kpL.pt.x;

        const vector<size_t> &vCandidates = vRowIndices[vL];

        if(vCandidates.empty())
            continue;

        const float minU = uL-maxD;
        const float maxU = uL-minD;

        if(maxU<0)
            continue;

        int bestDist = ORBmatcher::TH_HIGH;
        size_t bestIdxR = 0;

        const cv::Mat &dL = mDescriptors.row(iL);

        // Compare descriptor to right keypoints
        for(size_t iC=0; iC<vCandidates.size(); iC++)
        {
            const size_t iR = vCandidates[iC];
            const cv::KeyPoint &kpR = mvKeysRight[iR];

            if(kpR.octave<levelL-1 || kpR.octave>levelL+1)
                continue;

            const float &uR = kpR.pt.x;

            if(uR>=minU && uR<=maxU)
            {
                const cv::Mat &dR = mDescriptorsRight.row(iR);
                const int dist = ORBmatcher::DescriptorDistance(dL,dR);

                if(dist<bestDist)
                {
                    bestDist = dist;
                    bestIdxR = iR;
                }
            }
        }

        // Subpixel match by correlation
        if(bestDist<thOrbDist)
        {
            // coordinates in image pyramid at keypoint scale
            const float uR0 = mvKeysRight[bestIdxR].pt.x;
            const float scaleFactor = mvInvScaleFactors[kpL.octave];
            const float scaleduL = round(kpL.pt.x*scaleFactor);
            const float scaledvL = round(kpL.pt.y*scaleFactor);
            const float scaleduR0 = round(uR0*scaleFactor);

            // sliding window search
            const int w = 5;
            cv::Mat IL = mpORBextractorLeft->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduL-w,scaleduL+w+1);
            //IL.convertTo(IL,CV_32F);
            //IL = IL - IL.at<float>(w,w) *cv::Mat::ones(IL.rows,IL.cols,CV_32F);
            IL.convertTo(IL,CV_16S);
            IL = IL - IL.at<short>(w,w);

            int bestDist = INT_MAX;
            int bestincR = 0;
            const int L = 5;
            vector<float> vDists;
            vDists.resize(2*L+1);

            const float iniu = scaleduR0+L-w;
            const float endu = scaleduR0+L+w+1;
            if(iniu<0 || endu >= mpORBextractorRight->mvImagePyramid[kpL.octave].cols)
                continue;

            for(int incR=-L; incR<=+L; incR++)
            {
                cv::Mat IR = mpORBextractorRight->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduR0+incR-w,scaleduR0+incR+w+1);
                //IR.convertTo(IR,CV_32F);
                //IR = IR - IR.at<float>(w,w) *cv::Mat::ones(IR.rows,IR.cols,CV_32F);
                IR.convertTo(IR,CV_16S);
                IR = IR - IR.at<short>(w,w);

                float dist = cv::norm(IL,IR,cv::NORM_L1);
                if(dist<bestDist)
                {
                    bestDist =  dist;
                    bestincR = incR;
                }

                vDists[L+incR] = dist;
            }

            if(bestincR==-L || bestincR==L)
                continue;

            // Sub-pixel match (Parabola fitting)
            const float dist1 = vDists[L+bestincR-1];
            const float dist2 = vDists[L+bestincR];
            const float dist3 = vDists[L+bestincR+1];

            const float deltaR = (dist1-dist3)/(2.0f*(dist1+dist3-2.0f*dist2));

            if(deltaR<-1 || deltaR>1)
                continue;

            // Re-scaled coordinate
            float bestuR = mvScaleFactors[kpL.octave]*((float)scaleduR0+(float)bestincR+deltaR);

            float disparity = (uL-bestuR);

            if(disparity>=minD && disparity<maxD)
            {
                if(disparity<=0)
                {
                    disparity=0.01;
                    bestuR = uL-0.01;
                }
                mvDepth[iL]=mbf/disparity;
                mvuRight[iL] = bestuR;
                vDistIdx.push_back(pair<int,int>(bestDist,iL));
            }
        }
    }

    sort(vDistIdx.begin(),vDistIdx.end());
    const float median = vDistIdx[vDistIdx.size()/2].first;
    const float thDist = 1.5f*1.4f*median;

    for(int i=vDistIdx.size()-1;i>=0;i--)
    {
        if(vDistIdx[i].first<thDist)
            break;
        else
        {
            mvuRight[vDistIdx[i].second]=-1;
            mvDepth[vDistIdx[i].second]=-1;
        }
    }
}
//输出深度值和视差
void Frame::ComputeStereoMatches_Lines()
{
    // Depth, Disparity and the 3D vector that expresses the observed inﬁnite line in the image plane
    //初始化线段点的深度值为-1
    mvDepth_l.resize(N_l,pair<float,float>(-1.0f,-1.0f));//线端点深度值
    mvDisparity_l.clear();//视差
    mvle_l.clear();//线段的三维向量表示
    mvDisparity_l.resize(mvKeys_Line.size(),pair<float,float>(-1,-1));
    mvle_l.resize(mvKeys_Line.size(),Vector3d(0,0,0));

    // Line segments stereo matching
    // --------------------------------------------------------------------------------------------------------------------
    if (mvKeys_Line.empty() || mvKeysRight_Line.empty())
        return;

    std::vector<line_2d> coords;   // line_2d type definition in LineMatcher.h
    coords.reserve(mvKeys_Line.size());
    for (const KeyLine &kl : mvKeys_Line)
        //计算出线的端点在哪个网格
        coords.push_back(std::make_pair(std::make_pair(kl.startPointX * inv_width, kl.startPointY * inv_height),
                                        std::make_pair(kl.endPointX * inv_width, kl.endPointY * inv_height))); 

    //Fill in grid & directions
    list<pair<int, int>> line_coords;
    GridStructure grid(FRAME_GRID_ROWS, FRAME_GRID_COLS);//48 64 个网格
    std::vector<std::pair<double, double>> directions(mvKeysRight_Line.size());
    //对每个线段进行遍历
    for (unsigned int idx = 0; idx < mvKeysRight_Line.size(); ++idx)
    {
        const KeyLine &kl = mvKeysRight_Line[idx];

        std::pair<double, double> &v = directions[idx];
        //对于每个线段的起点和终点计算x y方向上的偏移量
        v = std::make_pair((kl.endPointX - kl.startPointX) * inv_width, (kl.endPointY - kl.startPointY) * inv_height);
        //将偏移量标准化
        normalize(v);
        //计算起点终点所有像素坐标存在line_coords
       getLineCoords(kl.startPointX * inv_width, kl.startPointY * inv_height, kl.endPointX * inv_width, kl.endPointY * inv_height, line_coords);
       // 对所有起点和终点分配网格索引
       for (const std::pair<int, int> &p : line_coords)
            grid.at(p.first, p.second).push_back(idx);
    }

    GridWindow w;
    int size_width = 7;
    int size_height = 2; // You can increase this value for non rectified images
    w.width = std::make_pair(size_width, 0);
    w.height = std::make_pair(size_height, size_height);

    std::vector<int> matches_12;
    //根据网格的线段匹配法，双向余弦相似度匹配，低于阈值跳过
    LineMatcher::matchGrid(coords, mDescriptors_Line, grid, mDescriptorsRight_Line, directions, w, matches_12);

    // bucle around lmatches
    Mat mDescriptors_Line_aux;
    //对匹配好的线段进行遍历
    for (unsigned int i1 = 0; i1 < matches_12.size(); ++i1) {
        const int i2 = matches_12[i1];
        //检查线段2的索引是否有效，若无效则跳过当前匹配对。
        if (i2 < 0) continue;

        // estimate the disparity of the endpoints
        //构造表示线段1和线段2起点的齐次坐标向量
        Vector3d sp_l; sp_l << mvKeys_Line[i1].startPointX, mvKeys_Line[i1].startPointY, 1.0;
//        构造表示线段1和线段2终点的齐次坐标向量
        Vector3d ep_l; ep_l << mvKeys_Line[i1].endPointX,   mvKeys_Line[i1].endPointY,   1.0;
        Vector3d sp_r; sp_r << mvKeysRight_Line[i2].startPointX, mvKeysRight_Line[i2].startPointY, 1.0;
        Vector3d ep_r; ep_r << mvKeysRight_Line[i2].endPointX,   mvKeysRight_Line[i2].endPointY,   1.0;
       //计算线段2的极线（epipolar line）通过计算sp_r和ep_r的叉积，
        Vector3d le_r; le_r << sp_r.cross(ep_r);
        //判断左右线的重叠程度，将重叠的两条线段设为最小的哪个的长度
        double overlap = lineSegmentOverlapStereo( sp_l(1), ep_l(1), sp_r(1), ep_r(1) );

        double disp_s, disp_e;
        //计算起点的视差
        //将起点的x坐标与起点和右图线段的y坐标差值以及右图线段的x坐标与起点和左图线段的y坐标差值相乘。
        //将乘积除以右图线段的y坐标与左图线段的y坐标差值。
        //将计算得到的x坐标作为起点的视差，起点的y坐标保持不变。
        //计算立体图像中对应点之间的水平位移量。
        //左右目起点终点的坐标估计
        sp_r << ( sp_r(0)*( sp_l(1) - ep_r(1) ) + ep_r(0)*( sp_r(1) - sp_l(1) ) ) / ( sp_r(1)-ep_r(1) ) , sp_l(1) ,  1.0;
        ep_r << ( sp_r(0)*( ep_l(1) - ep_r(1) ) + ep_r(0)*( sp_r(1) - ep_l(1) ) ) / ( sp_r(1)-ep_r(1) ) , ep_l(1) ,  1.0;
        //通过左右视差设定阈值对线进行过滤 输入左目的起始点x y 终点xy 右目的 起始点x y 终点x y
        filterLineSegmentDisparity( sp_l.head(2), ep_l.head(2), sp_r.head(2), ep_r.head(2), disp_s, disp_e );

        // check minimal disparity
        int minDisp = 1;
        float lineHorizTh = 0.1;
        float stereoOverlapTh = 0.75;
        //判断线起始点左右目的视差>1，且终点左右目的视差>1，且起始和重点的x坐标的绝对值>0.1，
        // 且起始和重点的y坐标的绝对值>0.1，且左右目线段重叠度大于0.75
        //这样的线存在视差大，可以进行双目立体匹配，这样的线段是稳定可靠的，进一步保存视差和深度
        if( disp_s >= minDisp && disp_e >= minDisp
            && std::abs( sp_l(1)-ep_l(1) ) > lineHorizTh
            && std::abs( sp_r(1)-ep_r(1) ) > lineHorizTh
            && overlap > stereoOverlapTh )
        {
            //第一个线段起始点的视差，第二个值表示线段终止点的视差。
            mvDisparity_l[i1] = make_pair(disp_s,disp_e);
            //起点终点点的深度
            // //起点 将右图像的基线与起点的视差（disp_s）相除，并乘以左右相机的焦距比（mbf）
            //mbf变量表示左右相机的焦距比乘以基线
            //深度值 = 左右相机的焦距比 / 视差值
            mvDepth_l[i1] = pair<float,float>(mbf/float(disp_s), mbf/float(disp_e));
        }
    }
    // 存储每个线段在图像平面上的表示的三维向量不管深度值怎么样
    for (int i=0; i < N_l; i++) {
        Vector3d sp_lun; sp_lun << mvKeysUn_Line[i].startPointX, mvKeysUn_Line[i].startPointY, 1.0;
        Vector3d ep_lun; ep_lun << mvKeysUn_Line[i].endPointX,   mvKeysUn_Line[i].endPointY,   1.0;
        Vector3d le_l; le_l << sp_lun.cross(ep_lun); le_l = le_l / std::sqrt( le_l(0)*le_l(0) + le_l(1)*le_l(1) );
        mvle_l[i] = le_l;
    }
}

double Frame::lineSegmentOverlapStereo(double spl_obs, double epl_obs, double spl_proj, double epl_proj)
{

    double overlap = 1.f;
    float lineHorizTh = 0.1;

    if( fabs( epl_obs - spl_obs ) > lineHorizTh ) // normal lines (verticals included)
    {
        double sln    = min(spl_obs,  epl_obs);
        double eln    = max(spl_obs,  epl_obs);
        double spn    = min(spl_proj, epl_proj);
        double epn    = max(spl_proj, epl_proj);

        double length = eln-spn;

        if ( (epn < sln) || (spn > eln) )
            overlap = 0.f;
        else{
            if ( (epn>eln) && (spn<sln) )
                overlap = eln-sln;
            else
                overlap = min(eln,epn) - max(sln,spn);
        }

        if(length>0.01f)
            overlap = overlap / length;
        else
            overlap = 0.f;

        if( overlap > 1.f )
            overlap = 1.f;

    }

    return overlap;
}
///myline 语义覆盖函数
void Frame::filterLineSegmentDisparity( Vector2d spl, Vector2d epl, Vector2d spr, Vector2d epr, double &disp_s, double &disp_e )
{
    disp_s = spl(0) - spr(0);
    disp_e = epl(0) - epr(0);
    // if they are too different, ignore them
    float lsMinDispRatio = 0.7;
    if(  min( disp_s, disp_e ) / max( disp_s, disp_e ) < lsMinDispRatio )
    {
        disp_s = -1.0;
        disp_e = -1.0;
    }
}


void Frame::ComputeStereoFromRGBD(const cv::Mat &imDepth)
{
    mvuRight = vector<float>(N,-1);
    mvDepth = vector<float>(N,-1);

    for(int i=0; i<N; i++)
    {
        const cv::KeyPoint &kp = mvKeys[i];
        const cv::KeyPoint &kpU = mvKeysUn[i];

        const float &v = kp.pt.y;
        const float &u = kp.pt.x;

        const float d = imDepth.at<float>(v,u);

        if(d>0)
        {
            mvDepth[i] = d;
            mvuRight[i] = kpU.pt.x-mbf/d;
        }
    }
}


//void Frame::ComputeRGBDMatches_Lines(const cv::Mat& imDepth)
//{
//    mvDepth_l.resize(N_l, std::pair<float, float>(-1.0f, -1.0f));
//    mvDisparity_l.clear();
//    mvle_l.clear();
//    mvDisparity_l.resize(mvKeys_Line.size(), std::pair<float, float>(-1, -1));
//    mvle_l.resize(mvKeys_Line.size(), Vector3d(0, 0, 0));
//
//    if (imDepth.data)
//    {
//        for (int i = 0; i < N_l; i++)
//        {
//            int u = mvKeysUn_Line[i].startPointX;
//            int v = mvKeysUn_Line[i].startPointY;
//
//
//            //对于每个具有正深度值的线进行更新视差,向量,深度值
//            // estimate the disparity of the endpoints
//            // 存储每个线段在图像平面上的表示的三维向量
//            Vector3d sp_l;
//            sp_l << mvKeysUn_Line[i].startPointX, mvKeysUn_Line[i].startPointY, 1.0;
//            Vector3d ep_l;
//            ep_l << mvKeysUn_Line[i].endPointX, mvKeysUn_Line[i].endPointY, 1.0;
//            Vector3d le_l;
//            //点乘求向量
//            le_l << sp_l.cross(ep_l);
//            // 对表示线段的三维向量进行归一化处理
//            le_l = le_l / std::sqrt(le_l(0) * le_l(0) + le_l(1) * le_l(1));
//            // 存储每个线段在图像平面上的表示的三维向量
//            mvle_l[i] = le_l;
//
//            int u1 = mvKeysUn_Line[i].endPointX;
//            int v1 = mvKeysUn_Line[i].endPointY;
//
//            const float d = imDepth.at<float>(v, u);//起始点对应深度图的像素位置
//            const float d1 = imDepth.at<float>(v1, u1);//终止点对应深度图的像素位置
//            //只有深度大于0的线才更新深度和视差
//            if (d > 0 && d1 > 0)
//            {
//
//                //判断只有视差大于0.1才更新深度值
//                float lineHorizTh = 0.1;
//                if (std::abs(sp_l(0) - ep_l(0)) > lineHorizTh || std::abs(sp_l(1) - ep_l(1)) > lineHorizTh)
//                {
//                    //float z =  mvKeysUn_Line[i].startPointX-mbf/d;
//                    float z = d;
//                    float z1 = d1;
//
//                    //第一个线段起始点的深度值，第二个值表示线段终止点的深度值。
//                    mvDepth_l[i] = std::make_pair(z, z1);
//                    //第一个线段起始点的视差，第二个值表示线段终止点的视差。
//                    mvDisparity_l[i] = std::make_pair(mbf / z , mbf / z1);
//                    //mvuRight[i] = kpU.pt.x-mbf/d;
////                    std::cout << "RGBD MBF===== " << mbf<<std::endl;
////                    std::cout << "RGB线点的深度值1=" << mvDepth_l[i].first <<"RGB线点的深度值2="<< mvDepth_l[i].second << std::endl;
////                    std::cout << "RGB线点的视差值1=" << mvDisparity_l[i].first <<"RGB线点的视差值2="<< mvDisparity_l[i].second << std::endl;
//                }
//
//            }
//
//        }
//    }
//}

    void Frame::ComputeRGBDMatches_Lines(const cv::Mat& imDepth)
    {
        mvDepth_l.resize(N_l, std::pair<float, float>(-1.0f, -1.0f));
        mvDisparity_l.clear();
        mvle_l.clear();
        mvDisparity_l.resize(mvKeys_Line.size(), std::pair<float, float>(-1, -1));
        mvle_l.resize(mvKeys_Line.size(), Vector3d(0, 0, 0));

        if (imDepth.data)
        {
            for (int i = 0; i < N_l; i++)
            {
                int u = mvKeysUn_Line[i].startPointX;
                int v = mvKeysUn_Line[i].startPointY;

                // 对于每个具有正深度值的线进行更新视差、向量、深度值
                // 存储每个线段在图像平面上的表示的三维向量
                Vector3d sp_l;
                sp_l << mvKeysUn_Line[i].startPointX, mvKeysUn_Line[i].startPointY, 1.0;
                Vector3d ep_l;
                ep_l << mvKeysUn_Line[i].endPointX, mvKeysUn_Line[i].endPointY, 1.0;
                Vector3d le_l;
                // 点乘求向量
                le_l << sp_l.cross(ep_l);
                // 对表示线段的三维向量进行归一化处理
                le_l = le_l / std::sqrt(le_l(0) * le_l(0) + le_l(1) * le_l(1));
                // 存储每个线段在图像平面上的表示的三维向量
                mvle_l[i] = le_l;

                int u1 = mvKeysUn_Line[i].endPointX;
                int v1 = mvKeysUn_Line[i].endPointY;

                const float d = imDepth.at<float>(v, u);     // 起始点对应深度图的像素位置
                const float d1 = imDepth.at<float>(v1, u1);  // 终止点对应深度图的像素位置

                // 只有深度大于0的线才更新深度和视差
                if (d > 0 && d1 > 0)
                {
                    // 判断只有视差大于0.1才更新深度值
                    float lineHorizTh = 0.3;
                    if (std::abs(sp_l(0) - ep_l(0)) > lineHorizTh || std::abs(sp_l(1) - ep_l(1)) > lineHorizTh)
                    {
                        float z = d;
                        float z1 = d1;

                        // 剔除冗余检测和线段长度异常的线段
                        // 这里您可以添加适当的条件进行剔除操作，下面是一个示例条件：
                        const float minLength = 10.0;  // 最小允许的线段长度
                        const float maxLength = 50.0; // 最大允许的线段长度

                        // 计算线段长度
                        float lineLength = std::sqrt((u - u1) * (u - u1) + (v - v1) * (v - v1));

                        if (lineLength >= minLength && lineLength <= maxLength)
                        {
                            // 第一个线段起始点的深度值，第二个值表示线段终止点的深度值
                            mvDepth_l[i] = std::make_pair(z, z1);
                            // 第一个线段起始点的视差，第二个值表示线段终止点的视差
                            mvDisparity_l[i] = std::make_pair(mbf / z, mbf / z1);
                        }
                    }
                }
            }
        }
    }









//myplan 线转为cvmat 这里的scale可能有错误
void Frame:: keylines_to_mat(const std::vector<cv::line_descriptor::KeyLine>& keylines_src, cv::Mat& linesmat_out, float scale)
{
    linesmat_out.create(keylines_src.size(),4,CV_32FC1);  // CV_32SC1
    for (int j=0;j<(int)keylines_src.size();j++)
    {
        linesmat_out.at<float>(j,0)=keylines_src[j].startPointX*scale;
        linesmat_out.at<float>(j,1)=keylines_src[j].startPointY*scale;
        linesmat_out.at<float>(j,2)=keylines_src[j].endPointX*scale;
        linesmat_out.at<float>(j,3)=keylines_src[j].endPointY*scale;
    }
}

cv::Mat Frame::UnprojectStereo(const int &i)
{
    const float z = mvDepth[i];
    if(z>0)
    {
        const float u = mvKeysUn[i].pt.x;
        const float v = mvKeysUn[i].pt.y;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);
        return mRwc*x3Dc+mOw;
    }
    else
        return cv::Mat();
}

Eigen::Vector3d Frame::UnprojectStereoLines(const double &u, const double &v, const double &z)
{
    // convert the point in image to the point in world coordinate.
    Eigen::Vector3d P;      // point in camera coordinate
    P(0) = (u-cx)*z*invfx;
    P(1) = (v-cy)*z*invfy;
    P(2) = z;
    return Converter::toMatrix3d(mRwc)*P+Converter::toVector3d(mOw);
}

Eigen::Vector2d Frame::projection(const Eigen::Vector3d &P )
{
    // convert the 3Dpoint in camera coordinate to the point in image.
    Vector2d uv_unit;
    uv_unit(0) = cx + fx * P(0) / P(2);
    uv_unit(1) = cy + fy * P(1) / P(2);
    return uv_unit;
}

bool Frame::imuIsPreintegrated()
{
    unique_lock<std::mutex> lock(*mpMutexImu);
    return mbImuPreintegrated;
}

void Frame::setIntegrated()
{
    unique_lock<std::mutex> lock(*mpMutexImu);
    mbImuPreintegrated = true;
}
//左右目模式
Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth, GeometricCamera* pCamera, GeometricCamera* pCamera2, cv::Mat& Tlr,Frame* pPrevF, const IMU::Calib &ImuCalib)
        :mpcpi(NULL), mpORBvocabulary(voc),mpORBextractorLeft(extractorLeft),mpORBextractorRight(extractorRight), mTimeStamp(timeStamp), mK(K.clone()), mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
         mImuCalib(ImuCalib), mpImuPreintegrated(NULL), mpPrevFrame(pPrevF),mpImuPreintegratedFrame(NULL), mpReferenceKF(static_cast<KeyFrame*>(NULL)), mbImuPreintegrated(false), mpCamera(pCamera), mpCamera2(pCamera2), mTlr(Tlr)
{
    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    imgLeft = imLeft.clone();
    imgRight = imRight.clone();
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
    thread threadLeft(&Frame::ExtractORB,this,0,imLeft,static_cast<KannalaBrandt8*>(mpCamera)->mvLappingArea[0],static_cast<KannalaBrandt8*>(mpCamera)->mvLappingArea[1]);
    thread threadRight(&Frame::ExtractORB,this,1,imRight,static_cast<KannalaBrandt8*>(mpCamera2)->mvLappingArea[0],static_cast<KannalaBrandt8*>(mpCamera2)->mvLappingArea[1]);
    threadLeft.join();
    threadRight.join();
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

    Nleft = mvKeys.size();
    Nright = mvKeysRight.size();
    N = Nleft + Nright;

    if(N == 0)
        return;

    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imLeft);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf / fx;

    mRlr = mTlr.rowRange(0,3).colRange(0,3);
    mtlr = mTlr.col(3);

    cv::Mat Rrl = mTlr.rowRange(0,3).colRange(0,3).t();
    cv::Mat trl = Rrl * (-1 * mTlr.col(3));

    cv::hconcat(Rrl,trl,mTrl);

    ComputeStereoFishEyeMatches();
    std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();

    //Put all descriptors in the same matrix
    cv::vconcat(mDescriptors,mDescriptorsRight,mDescriptors);

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(nullptr));
    mvbOutlier = vector<bool>(N,false);

    AssignFeaturesToGrid();
    std::chrono::steady_clock::time_point t4 = std::chrono::steady_clock::now();

    mpMutexImu = new std::mutex();

    UndistortKeyPoints();
    std::chrono::steady_clock::time_point t5 = std::chrono::steady_clock::now();

    double t_read = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t1 - t0).count();
    double t_orbextract = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2 - t1).count();
    double t_stereomatches = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t3 - t2).count();
    double t_assign = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t4 - t3).count();
    double t_undistort = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t5 - t4).count();

    /*cout << "Reading time: " << t_read << endl;
    cout << "Extraction time: " << t_orbextract << endl;
    cout << "Matching time: " << t_stereomatches << endl;
    cout << "Assignment time: " << t_assign << endl;
    cout << "Undistortion time: " << t_undistort << endl;*/

}

void Frame::ComputeStereoFishEyeMatches() {
    //Speed it up by matching keypoints in the lapping area
    vector<cv::KeyPoint> stereoLeft(mvKeys.begin() + monoLeft, mvKeys.end());
    vector<cv::KeyPoint> stereoRight(mvKeysRight.begin() + monoRight, mvKeysRight.end());

    cv::Mat stereoDescLeft = mDescriptors.rowRange(monoLeft, mDescriptors.rows);
    cv::Mat stereoDescRight = mDescriptorsRight.rowRange(monoRight, mDescriptorsRight.rows);

    mvLeftToRightMatch = vector<int>(Nleft,-1);
    mvRightToLeftMatch = vector<int>(Nright,-1);
    mvDepth = vector<float>(Nleft,-1.0f);
    mvuRight = vector<float>(Nleft,-1);
    mvStereo3Dpoints = vector<cv::Mat>(Nleft);
    mnCloseMPs = 0;

    //Perform a brute force between Keypoint in the left and right image
    vector<vector<cv::DMatch>> matches;

    BFmatcher.knnMatch(stereoDescLeft,stereoDescRight,matches,2);

    int nMatches = 0;
    int descMatches = 0;

    //Check matches using Lowe's ratio
    for(vector<vector<cv::DMatch>>::iterator it = matches.begin(); it != matches.end(); ++it){
        if((*it).size() >= 2 && (*it)[0].distance < (*it)[1].distance * 0.7){
            //For every good match, check parallax and reprojection error to discard spurious matches
            cv::Mat p3D;
            descMatches++;
            float sigma1 = mvLevelSigma2[mvKeys[(*it)[0].queryIdx + monoLeft].octave], sigma2 = mvLevelSigma2[mvKeysRight[(*it)[0].trainIdx + monoRight].octave];
            float depth = static_cast<KannalaBrandt8*>(mpCamera)->TriangulateMatches(mpCamera2,mvKeys[(*it)[0].queryIdx + monoLeft],mvKeysRight[(*it)[0].trainIdx + monoRight],mRlr,mtlr,sigma1,sigma2,p3D);
            if(depth > 0.0001f){
                mvLeftToRightMatch[(*it)[0].queryIdx + monoLeft] = (*it)[0].trainIdx + monoRight;
                mvRightToLeftMatch[(*it)[0].trainIdx + monoRight] = (*it)[0].queryIdx + monoLeft;
                mvStereo3Dpoints[(*it)[0].queryIdx + monoLeft] = p3D.clone();
                mvDepth[(*it)[0].queryIdx + monoLeft] = depth;
                nMatches++;
            }
        }
    }
}

bool Frame::isInFrustumChecks(MapPoint *pMP, float viewingCosLimit, bool bRight) {
    // 3D in absolute coordinates
    cv::Mat P = pMP->GetWorldPos();

    cv::Mat mR, mt, twc;
    if(bRight){
        cv::Mat Rrl = mTrl.colRange(0,3).rowRange(0,3);
        cv::Mat trl = mTrl.col(3);
        mR = Rrl * mRcw;
        mt = Rrl * mtcw + trl;
        twc = mRwc * mTlr.rowRange(0,3).col(3) + mOw;
    }
    else{
        mR = mRcw;
        mt = mtcw;
        twc = mOw;
    }

    // 3D in camera coordinates
    cv::Mat Pc = mR*P+mt;
    const float Pc_dist = cv::norm(Pc);
    const float &PcZ = Pc.at<float>(2);

    // Check positive depth
    if(PcZ<0.0f)
        return false;

    // Project in image and check it is not outside
    cv::Point2f uv;
    if(bRight) uv = mpCamera2->project(Pc);
    else uv = mpCamera->project(Pc);

    if(uv.x<mnMinX || uv.x>mnMaxX)
        return false;
    if(uv.y<mnMinY || uv.y>mnMaxY)
        return false;

    // Check distance is in the scale invariance region of the MapPoint
    const float maxDistance = pMP->GetMaxDistanceInvariance();
    const float minDistance = pMP->GetMinDistanceInvariance();
    const cv::Mat PO = P-twc;
    const float dist = cv::norm(PO);

    if(dist<minDistance || dist>maxDistance)
        return false;

    // Check viewing angle
    cv::Mat Pn = pMP->GetNormal();

    const float viewCos = PO.dot(Pn)/dist;

    if(viewCos<viewingCosLimit)
        return false;

    // Predict scale in the image
    const int nPredictedLevel = pMP->PredictScale(dist,this);

    if(bRight){
        pMP->mTrackProjXR = uv.x;
        pMP->mTrackProjYR = uv.y;
        pMP->mnTrackScaleLevelR= nPredictedLevel;
        pMP->mTrackViewCosR = viewCos;
        pMP->mTrackDepthR = Pc_dist;
    }
    else{
        pMP->mTrackProjX = uv.x;
        pMP->mTrackProjY = uv.y;
        pMP->mnTrackScaleLevel= nPredictedLevel;
        pMP->mTrackViewCos = viewCos;
        pMP->mTrackDepth = Pc_dist;
    }

    return true;
}

cv::Mat Frame::UnprojectStereoFishEye(const int &i){
    return mRwc*mvStereo3Dpoints[i]+mOw;
}

/// mylk 新增函数 kitti=6 // imp 换数据集修改这
// Epipolar constraints and output the T matrix.
////首先对特征点周围的灰度值进行遍历筛选出质量高的特征点，然后根据alcOpticalFlowPyrLK计算稀疏光流
//// 计算遍历特征点在前一帧和当前帧特征点在周围像素的差异判断特征点质量，筛选出追踪比较好的特征点
////然后计算基础矩阵化，计算出当前帧特征点距离极线的距离，筛选出稳定可靠的特征点
void Frame::ProcessMovingObject(const cv::Mat &imgray,vector<Detection> &mmDetectMap)
{
    /// Clear the previous data kitti=6 person=1
//    ORB_SLAM3::Tracking trackingObj;//imp 使用这个必须在Tracking中有无参构造函数用来调用成员属性
    double limit_kitti_dis_epi;

//    std::cout<<"fSensor==="<<fSensor<<std::endl;
    if (fSensor == 1)
    {
        limit_kitti_dis_epi =6;
    }
    else if (fSensor == 2)
    {
        limit_kitti_dis_epi =0.5;
    }
//    std::cout<<"动态阈值======"<< limit_kitti_dis_epi<<std::endl;
    F_prepoint.clear();
    F_nextpoint.clear();
    F2_prepoint.clear();
    F2_nextpoint.clear();
    T_M.clear();

    // Detect dynamic target and ultimately optput the T matrix
        //用于在图像中检测好的特征点。
        // 灰度图像，输出特征点向量存储检测到的特征点坐标，需要检测的最大特征点数目，
        // 3 表示 Shi-Tomasi 角点检测，
        // true：表示进行非最大值抑制，0.04：表示非最大值抑制时的最小角点响应值阈值
    cv::goodFeaturesToTrack(imGrayPre, prepoint, 1000, 0.01, 8, cv::Mat(), 3, true, 0.04);
        //对已检测到的角点进行亚像素级别的精确化。该函数通过在角点周围的邻域内拟合亚像素级的二次曲线，来提高角点坐标的准确
        //灰度图像，角点向量，角点邻域的半宽和半高，cv::Size(-1, -1)表示不使用搜索窗口，当满足迭代次数达到 20 或者亚像素精度达到 0.03 时停止迭代
    cv::cornerSubPix(imGrayPre, prepoint, cv::Size(10, 10), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03));
    //计算稀疏光流的函数
    //前一帧灰度，当前帧灰度图，起始帧中的特征点坐标作为输入的初始位置，输出出的特征点在目标帧中的位置
    ///state：输出的特征点追踪状态
    //err：输出的特征点追踪误差，5：表示金字塔层数，用于指定
    //当满足迭代次数达到 20 或者光流精度达到 0.01 时，停止迭代
    cv::calcOpticalFlowPyrLK(imGrayPre, imgray, prepoint, nextpoint, state, err, cv::Size(22, 22), 5, cv::TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.01));

    for (int i = 0; i < state.size(); i++)
    {
        if(state[i] != 0)
            //该特征点在当前帧中有成功追踪的结果
        {
            int dx[10] = { -1, 0, 1, -1, 0, 1, -1, 0, 1 };
            int dy[10] = { -1, -1, -1, 0, 0, 0, 1, 1, 1 };
            int x1 = prepoint[i].x, y1 = prepoint[i].y;
            int x2 = nextpoint[i].x, y2 = nextpoint[i].y;
            //检查特征点的坐标是否在图像的边界范围内
            if ((x1 < limit_edge_corner || x1 >= imgray.cols - limit_edge_corner || x2 < limit_edge_corner || x2 >= imgray.cols - limit_edge_corner
                 || y1 < limit_edge_corner || y1 >= imgray.rows - limit_edge_corner || y2 < limit_edge_corner || y2 >= imgray.rows - limit_edge_corner))
            {
                state[i] = 0;
                continue;
            }
            //计算了前一帧和当前帧特征点周围像素的灰度差异，并根据灰度差异的总和来判断特征点的质量
            double sum_check = 0;
            for (int j = 0; j < 9; j++)
                sum_check += abs(imGrayPre.at<uchar>(y1 + dy[j], x1 + dx[j]) - imgray.at<uchar>(y2 + dy[j], x2 + dx[j]));
            //灰度值差异过大剔除
            if (sum_check > limit_of_check) state[i] = 0;
            if (state[i])
            {
                //将经过初步筛选的前后帧特征点保存，进行下一步基础矩阵极线约束
                F_prepoint.push_back(prepoint[i]);
                F_nextpoint.push_back(nextpoint[i]);
            }
        }
    }

    //计算基础矩阵  并根据极线约束 筛选特征点。
    //通过 mask 矩阵可以知道哪些关键点对应的是内点（符合基础矩阵约束）
    // 以及哪些是外点（不符合基础矩阵约束）
    // F-Matrix
    cv::Mat mask = cv::Mat(cv::Size(1, 300), CV_8UC1);
    //计算基础矩阵。不符合要求的特征点的mask值设置为零
    cv::Mat F = cv::findFundamentalMat(F_prepoint, F_nextpoint, mask, cv::FM_RANSAC, 0.1, 0.99);
    for (int i = 0; i < mask.rows; i++)
    {
        if (mask.at<uchar>(i, 0) == 0);//则跳过该特征点
        else
        {
            //，通过计算特征点在第二个视图上的投影与极线的距离，判断特征点的匹配是否满足几何关系。
            // 如果 dd 的值小于一个阈值，说明特征点的匹配符合极线约束，可以认为是有效的匹配点。
            //否则，根据基础矩阵F计算特征点在当前帧上的极线约束。
            //根据基础矩阵和特征点坐标计算出A、B、C三个参数，并计算特征点与对应匹配点的极线距离dd。
            // Circle(pre_frame, F_prepoint[i], 6, Scalar(255, 255, 0), 3);
            double A = F.at<double>(0, 0)*F_prepoint[i].x + F.at<double>(0, 1)*F_prepoint[i].y + F.at<double>(0, 2);
            double B = F.at<double>(1, 0)*F_prepoint[i].x + F.at<double>(1, 1)*F_prepoint[i].y + F.at<double>(1, 2);
            double C = F.at<double>(2, 0)*F_prepoint[i].x + F.at<double>(2, 1)*F_prepoint[i].y + F.at<double>(2, 2);
            //14讲167，p2^T * F * p1 = 0>>[p2.x, p2.y, 1] * F * [p1.x, p1.y, 1]^T = 0
            //[p2.x, p2.y, 1] * [F11, F12, F13; F21, F22, F23; F31, F32, F33] * [p1.x, p1.y, 1]^T = 0
            double dd = fabs(A*F_nextpoint[i].x + B*F_nextpoint[i].y + C) / sqrt(A*A + B*B); //Epipolar constraints
            if (dd <= 0.1)
            {
                F2_prepoint.push_back(F_prepoint[i]);
                F2_nextpoint.push_back(F_nextpoint[i]);
            }
        }
    }
    // //将前一帧和当前帧年能够稳定追踪的特征点保存，准备在framedrawer中画出光流箭头
    F_prepoint = F2_prepoint;
    F_nextpoint = F2_nextpoint;
    F_prepoint_draw = F_prepoint;
    //MYNOTE 这些当前帧的特征点传入到FrameDrwer 画出红色小点
    //MYNOTE 意思是前后帧满足lk基础矩阵约束的特征点进行下一步特帧点的跟踪和匹配
    //mynote 这里提取出来的特征点还没进行Desp计算，这里只是预处理后可以稳定跟踪的特征点
    F_nextpoint_draw = F_nextpoint;
    state_draw = state;
    for (int i = 0; i < prepoint.size(); i++)
    {
        if (state[i] != 0)
        {
            double A = F.at<double>(0, 0)*prepoint[i].x + F.at<double>(0, 1)*prepoint[i].y + F.at<double>(0, 2);
            double B = F.at<double>(1, 0)*prepoint[i].x + F.at<double>(1, 1)*prepoint[i].y + F.at<double>(1, 2);
            double C = F.at<double>(2, 0)*prepoint[i].x + F.at<double>(2, 1)*prepoint[i].y + F.at<double>(2, 2);
            double dd = fabs(A*nextpoint[i].x + B*nextpoint[i].y + C) / sqrt(A*A + B*B);

            // Judge outliers
            if (dd <= limit_kitti_dis_epi) continue;
            T_M.push_back(nextpoint[i]);
        }
    }
//    if(T_M.size() != 0)//注释掉
//    {
//        for(auto det : mmDetectMap)
//        {
//            for(auto  point : T_M)
//            {
//                if(det.class_idx == 0 && det.bbox.contains(point))
//                    T_M_dyna.push_back(point);
//                else
//                    continue;
//            }
//            if(T_M_dyna.size()/T_M.size() > 0.8)
//            {
//                //det.color_b = 0;det.color_g = 0; det.color_r =255;
//                person_area.push_back(det.bbox);
//            }
//            else
//                continue;
//        }
//    }

}

} //namespace ORB_SLAM
