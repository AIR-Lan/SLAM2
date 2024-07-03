

#include "Tracking.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"ORBmatcher.h"
#include"LineMatcher.h"
#include"FrameDrawer.h"
#include"Converter.h"
#include"Initializer.h"
#include"G2oTypes.h"
#include"Optimizer.h"
#include"PnPsolver.h"

#include<iostream>

#include<mutex>
#include<chrono>
#include <include/CameraModels/Pinhole.h>
#include <include/CameraModels/KannalaBrandt8.h>
#include <include/MLPnPsolver.h>

#include <fstream>
//myplan
#include "ProbabilityMapping.h"
//myplan
#include "planar_mapping_module.h"

// cube slam
#include "detect_3d_cuboid/matrix_utils.h"
#include "detect_3d_cuboid/detect_3d_cuboid.h"
#include "detect_3d_cuboid/object_3d_util.h"
#include "Object.h"

using namespace std;

int frame_id_tracking = -1;
bool ORB_SLAM3::Tracking::mbReadedGroundtruth = false;

typedef Vector5f VI;
int index_eao = 0;
bool VIC(const VI& lhs, const VI& rhs)
{
    return lhs[index_eao] > rhs[index_eao];
}

namespace ORB_SLAM3
{

/**
 * @brief 跟踪线程构造函数
 * @param pSys 系统类指针
 * @param pVoc 词典
 * @param pFrameDrawer 画图像的
 * @param pMapDrawer 画地图的
 * @param pAtlas atlas
 * @param pKFDB 关键帧词典数据库
 * @param strSettingPath 参数文件路径
 * @param sensor 传感器类型
 * @param settings 参数类
 * @param _strSeqName 序列名字，没用到
 */
    //myline 把参数线辞典添加进去 双目加线的追踪线程
Tracking::Tracking(System *pSys, ORBVocabulary* pVoc, LineVocabulary* pVoc_l, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Atlas *pAtlas, KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor, const string &_nameSeq):
    mState(NO_IMAGES_YET), mSensor(sensor), mTrackedFr(0), mbStep(false),
    mbOnlyTracking(false), mbMapUpdated(false), mbVO(false), mpORBVocabulary(pVoc), mpLineVocabulary(pVoc_l), mpKeyFrameDB(pKFDB),
    mpInitializer(static_cast<Initializer*>(NULL)), mpSystem(pSys), mpViewer(NULL),
    mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpAtlas(pAtlas), mnLastRelocFrameId(0), time_recently_lost(5.0),
    mnInitialFrameId(0), mbCreatedMap(false), mnFirstFrameId(0), mpCamera2(nullptr)
{
    // Load camera parameters from settings file
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    bool b_parse_cam = ParseCamParamFile(fSettings);
    if(!b_parse_cam)
    {
        std::cout << "*Error with the camera parameters in the config file*" << std::endl;
    }

    // Load ORB parameters
    bool b_parse_orb = ParseORBParamFile(fSettings);
    if(!b_parse_orb)
    {
        std::cout << "*Error with the ORB parameters in the config file*" << std::endl;
    }

    // Load Line parameters
    //myline 添加线参数
    bool b_parse_line = true;
    if(mSensor==System::STEREO || mSensor==System::IMU_STEREO|| mSensor==System::RGBD)
    {
        b_parse_line = ParseLineParamFile(fSettings);//IMP 在这里进行线提取器的实例化
        if(!b_parse_line)
        {
            std::cout << "*Error with the Line parameters in the config file*" << std::endl;
        }
    }
    //mypaln 用第一帧的真值进行优化
    // STEP read groundtruth ++++++++++++++++++++++++++++++++++++++++++++++++
    // notice: Only the camera pose of the initial frame is used to determine the ground plane normal.

    if (mbReadedGroundtruth == false)
    {
        ifstream infile("/home/lzh/VINS/EAO-SLAM/data/groundtruth.txt", ios::in);
        if (!infile.is_open())
        {
            cout << "tum groundtruth file open fail" << endl;
            exit(233);
        }
        else
        {
            std::cout << "read groundtruth.txt" << std::endl;
            mbReadedGroundtruth = true;
        }

        vector<double> row;
        double tmp;
        string line;
        cv::Mat cam_pose_mat;

        string s0;
        getline(infile, s0);
        getline(infile, s0);
        getline(infile, s0);

        // save as vector<vector<int>> _mat format.
        while (getline(infile, line))
        {
            // string to int.
            istringstream istr(line);
            while (istr >> tmp)
            {
                row.push_back(tmp);
            }

            mGroundtruth_mat.push_back(row); // vector<int> row.
            // Assuming mGroundtruth_mat is already populated
            //打印初始化myplan

            row.clear();
            istr.clear();
            line.clear();
        }
        infile.close();
    }
    // read groundtruth --------------------------------------------------


    initID = 0; lastID = 0;

    // Load IMU parameters
    bool b_parse_imu = true;
    if(sensor==System::IMU_MONOCULAR || sensor==System::IMU_STEREO)
    {
        b_parse_imu = ParseIMUParamFile(fSettings);
        if(!b_parse_imu)
        {
            std::cout << "*Error with the IMU parameters in the config file*" << std::endl;
        }

        mnFramesToResetIMU = mMaxFrames;
    }

    mbInitWith3KFs = false;
    mnNumDataset = 0;

    if(mSensor==System::STEREO || mSensor==System::IMU_STEREO)
    {    
        if(!b_parse_cam || !b_parse_orb || !b_parse_imu || !b_parse_line)
        {
            std::cerr << "**ERROR in the config file, the format is not correct**" << std::endl;
            try
            {
                throw -1;
            }
            catch(exception &e)
            {

            }
        }
    }
    else
    {
        if(!b_parse_cam || !b_parse_orb || !b_parse_imu)
        {
            std::cerr << "**ERROR in the config file, the format is not correct**" << std::endl;
            try
            {
                throw -1;
            }
            catch(exception &e)
            {

            }
        }
    }
    
    //f_track_stats.open("tracking_stats"+ _nameSeq + ".txt");
    /*f_track_stats.open("tracking_stats.txt");
    f_track_stats << "# timestamp, Num KF local, Num MP local, time" << endl;
    f_track_stats << fixed ;*/

#ifdef SAVE_TIMES
    f_track_times.open("tracking_times.txt");
    f_track_times << "# ORB_Ext(ms), Stereo matching(ms), Preintegrate_IMU(ms), Pose pred(ms), LocalMap_track(ms), NewKF_dec(ms)" << endl;
    f_track_times << fixed ;
#endif
}



//没线的tracking应该用不到
Tracking::Tracking(System *pSys, ORBVocabulary* pVoc, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Atlas *pAtlas, KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor, const string &_nameSeq):
    mState(NO_IMAGES_YET), mSensor(sensor), mTrackedFr(0), mbStep(false),
    mbOnlyTracking(false), mbMapUpdated(false), mbVO(false), mpORBVocabulary(pVoc), mpKeyFrameDB(pKFDB),
    mpInitializer(static_cast<Initializer*>(NULL)), mpSystem(pSys), mpViewer(NULL),
    mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpAtlas(pAtlas), mnLastRelocFrameId(0), time_recently_lost(5.0),
    mnInitialFrameId(0), mbCreatedMap(false), mnFirstFrameId(0), mpCamera2(nullptr)
{
    // Load camera parameters from settings file
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    bool b_parse_cam = ParseCamParamFile(fSettings);
    if(!b_parse_cam)
    {
        std::cout << "*Error with the camera parameters in the config file*" << std::endl;
    }

    // Load ORB parameters
    bool b_parse_orb = ParseORBParamFile(fSettings);
    if(!b_parse_orb)
    {
        std::cout << "*Error with the ORB parameters in the config file*" << std::endl;
    }

    initID = 0; lastID = 0;

    // Load IMU parameters
    bool b_parse_imu = true;
    if(sensor==System::IMU_MONOCULAR || sensor==System::IMU_STEREO)
    {
        b_parse_imu = ParseIMUParamFile(fSettings);
        if(!b_parse_imu)
        {
            std::cout << "*Error with the IMU parameters in the config file*" << std::endl;
        }

        mnFramesToResetIMU = mMaxFrames;
    }

    mbInitWith3KFs = false;


    //Rectification parameters
    /*mbNeedRectify = false;
    if((mSensor == System::STEREO || mSensor == System::IMU_STEREO) && sCameraName == "PinHole")
    {
        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fSettings["LEFT.K"] >> K_l;
        fSettings["RIGHT.K"] >> K_r;

        fSettings["LEFT.P"] >> P_l;
        fSettings["RIGHT.P"] >> P_r;

        fSettings["LEFT.R"] >> R_l;
        fSettings["RIGHT.R"] >> R_r;

        fSettings["LEFT.D"] >> D_l;
        fSettings["RIGHT.D"] >> D_r;

        int rows_l = fSettings["LEFT.height"];
        int cols_l = fSettings["LEFT.width"];
        int rows_r = fSettings["RIGHT.height"];
        int cols_r = fSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty()
                || rows_l==0 || cols_l==0 || rows_r==0 || cols_r==0)
        {
            mbNeedRectify = false;
        }
        else
        {
            mbNeedRectify = true;
            // M1r y M2r son los outputs (igual para l)
            // M1r y M2r son las matrices relativas al mapeo correspondiente a la rectificación de mapa en el eje X e Y respectivamente
            //cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
            //cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);
        }


    }
    else
    {
        int cols = 752;
        int rows = 480;
        cv::Mat R_l = cv::Mat::eye(3, 3, CV_32F);
    }*/

    mnNumDataset = 0;

    if(!b_parse_cam || !b_parse_orb || !b_parse_imu)
    {
        std::cerr << "**ERROR in the config file, the format is not correct**" << std::endl;
        try
        {
            throw -1;
        }
        catch(exception &e)
        {

        }
    }

    //f_track_stats.open("tracking_stats"+ _nameSeq + ".txt");
    /*f_track_stats.open("tracking_stats.txt");
    f_track_stats << "# timestamp, Num KF local, Num MP local, time" << endl;
    f_track_stats << fixed ;*/

#ifdef SAVE_TIMES
    f_track_times.open("tracking_times.txt");
    f_track_times << "# ORB_Ext(ms), Stereo matching(ms), Preintegrate_IMU(ms), Pose pred(ms), LocalMap_track(ms), NewKF_dec(ms)" << endl;
    f_track_times << fixed ;
#endif
}

Tracking::~Tracking()
{
    //f_track_stats.close();
#ifdef SAVE_TIMES
    f_track_times.close();
#endif
}

bool Tracking::ParseCamParamFile(cv::FileStorage &fSettings)
{
    mDistCoef = cv::Mat::zeros(4,1,CV_32F);
    cout << endl << "Camera Parameters: " << endl;
    bool b_miss_params = false;

    string sCameraName = fSettings["Camera.type"];
    if(sCameraName == "PinHole")
    {
        float fx, fy, cx, cy;

        // Camera calibration parameters
        cv::FileNode node = fSettings["Camera.fx"];
        if(!node.empty() && node.isReal())
        {
            fx = node.real();
        }
        else
        {
            std::cerr << "*Camera.fx parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.fy"];
        if(!node.empty() && node.isReal())
        {
            fy = node.real();
        }
        else
        {
            std::cerr << "*Camera.fy parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.cx"];
        if(!node.empty() && node.isReal())
        {
            cx = node.real();
        }
        else
        {
            std::cerr << "*Camera.cx parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.cy"];
        if(!node.empty() && node.isReal())
        {
            cy = node.real();
        }
        else
        {
            std::cerr << "*Camera.cy parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        // Distortion parameters
        node = fSettings["Camera.k1"];
        if(!node.empty() && node.isReal())
        {
            mDistCoef.at<float>(0) = node.real();
        }
        else
        {
            std::cerr << "*Camera.k1 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.k2"];
        if(!node.empty() && node.isReal())
        {
            mDistCoef.at<float>(1) = node.real();
        }
        else
        {
            std::cerr << "*Camera.k2 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.p1"];
        if(!node.empty() && node.isReal())
        {
            mDistCoef.at<float>(2) = node.real();
        }
        else
        {
            std::cerr << "*Camera.p1 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.p2"];
        if(!node.empty() && node.isReal())
        {
            mDistCoef.at<float>(3) = node.real();
        }
        else
        {
            std::cerr << "*Camera.p2 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.k3"];
        if(!node.empty() && node.isReal())
        {
            mDistCoef.resize(5);
            mDistCoef.at<float>(4) = node.real();
        }

        if(b_miss_params)
        {
            return false;
        }

        vector<float> vCamCalib{fx,fy,cx,cy};

        mpCamera = new Pinhole(vCamCalib);

        mpAtlas->AddCamera(mpCamera);


        std::cout << "- Camera: Pinhole" << std::endl;
        std::cout << "- fx: " << fx << std::endl;
        std::cout << "- fy: " << fy << std::endl;
        std::cout << "- cx: " << cx << std::endl;
        std::cout << "- cy: " << cy << std::endl;
        std::cout << "- k1: " << mDistCoef.at<float>(0) << std::endl;
        std::cout << "- k2: " << mDistCoef.at<float>(1) << std::endl;


        std::cout << "- p1: " << mDistCoef.at<float>(2) << std::endl;
        std::cout << "- p2: " << mDistCoef.at<float>(3) << std::endl;

        if(mDistCoef.rows==5)
            std::cout << "- k3: " << mDistCoef.at<float>(4) << std::endl;

        mK = cv::Mat::eye(3,3,CV_32F);
        mK.at<float>(0,0) = fx;
        mK.at<float>(1,1) = fy;
        mK.at<float>(0,2) = cx;
        mK.at<float>(1,2) = cy;

    }
    else if(sCameraName == "KannalaBrandt8")
    {
        float fx, fy, cx, cy;
        float k1, k2, k3, k4;

        // Camera calibration parameters
        cv::FileNode node = fSettings["Camera.fx"];
        if(!node.empty() && node.isReal())
        {
            fx = node.real();
        }
        else
        {
            std::cerr << "*Camera.fx parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }
        node = fSettings["Camera.fy"];
        if(!node.empty() && node.isReal())
        {
            fy = node.real();
        }
        else
        {
            std::cerr << "*Camera.fy parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.cx"];
        if(!node.empty() && node.isReal())
        {
            cx = node.real();
        }
        else
        {
            std::cerr << "*Camera.cx parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.cy"];
        if(!node.empty() && node.isReal())
        {
            cy = node.real();
        }
        else
        {
            std::cerr << "*Camera.cy parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        // Distortion parameters
        node = fSettings["Camera.k1"];
        if(!node.empty() && node.isReal())
        {
            k1 = node.real();
        }
        else
        {
            std::cerr << "*Camera.k1 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }
        node = fSettings["Camera.k2"];
        if(!node.empty() && node.isReal())
        {
            k2 = node.real();
        }
        else
        {
            std::cerr << "*Camera.k2 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.k3"];
        if(!node.empty() && node.isReal())
        {
            k3 = node.real();
        }
        else
        {
            std::cerr << "*Camera.k3 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.k4"];
        if(!node.empty() && node.isReal())
        {
            k4 = node.real();
        }
        else
        {
            std::cerr << "*Camera.k4 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        if(!b_miss_params)
        {
            vector<float> vCamCalib{fx,fy,cx,cy,k1,k2,k3,k4};
            mpCamera = new KannalaBrandt8(vCamCalib);

            std::cout << "- Camera: Fisheye" << std::endl;
            std::cout << "- fx: " << fx << std::endl;
            std::cout << "- fy: " << fy << std::endl;
            std::cout << "- cx: " << cx << std::endl;
            std::cout << "- cy: " << cy << std::endl;
            std::cout << "- k1: " << k1 << std::endl;
            std::cout << "- k2: " << k2 << std::endl;
            std::cout << "- k3: " << k3 << std::endl;
            std::cout << "- k4: " << k4 << std::endl;

            mK = cv::Mat::eye(3,3,CV_32F);
            mK.at<float>(0,0) = fx;
            mK.at<float>(1,1) = fy;
            mK.at<float>(0,2) = cx;
            mK.at<float>(1,2) = cy;
        }

        if(mSensor==System::STEREO || mSensor==System::IMU_STEREO){
            // Right camera
            // Camera calibration parameters
            cv::FileNode node = fSettings["Camera2.fx"];
            if(!node.empty() && node.isReal())
            {
                fx = node.real();
            }
            else
            {
                std::cerr << "*Camera2.fx parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }
            node = fSettings["Camera2.fy"];
            if(!node.empty() && node.isReal())
            {
                fy = node.real();
            }
            else
            {
                std::cerr << "*Camera2.fy parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera2.cx"];
            if(!node.empty() && node.isReal())
            {
                cx = node.real();
            }
            else
            {
                std::cerr << "*Camera2.cx parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera2.cy"];
            if(!node.empty() && node.isReal())
            {
                cy = node.real();
            }
            else
            {
                std::cerr << "*Camera2.cy parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            // Distortion parameters
            node = fSettings["Camera2.k1"];
            if(!node.empty() && node.isReal())
            {
                k1 = node.real();
            }
            else
            {
                std::cerr << "*Camera2.k1 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }
            node = fSettings["Camera2.k2"];
            if(!node.empty() && node.isReal())
            {
                k2 = node.real();
            }
            else
            {
                std::cerr << "*Camera2.k2 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera2.k3"];
            if(!node.empty() && node.isReal())
            {
                k3 = node.real();
            }
            else
            {
                std::cerr << "*Camera2.k3 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera2.k4"];
            if(!node.empty() && node.isReal())
            {
                k4 = node.real();
            }
            else
            {
                std::cerr << "*Camera2.k4 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }


            int leftLappingBegin = -1;
            int leftLappingEnd = -1;

            int rightLappingBegin = -1;
            int rightLappingEnd = -1;

            node = fSettings["Camera.lappingBegin"];
            if(!node.empty() && node.isInt())
            {
                leftLappingBegin = node.operator int();
            }
            else
            {
                std::cout << "WARNING: Camera.lappingBegin not correctly defined" << std::endl;
            }
            node = fSettings["Camera.lappingEnd"];
            if(!node.empty() && node.isInt())
            {
                leftLappingEnd = node.operator int();
            }
            else
            {
                std::cout << "WARNING: Camera.lappingEnd not correctly defined" << std::endl;
            }
            node = fSettings["Camera2.lappingBegin"];
            if(!node.empty() && node.isInt())
            {
                rightLappingBegin = node.operator int();
            }
            else
            {
                std::cout << "WARNING: Camera2.lappingBegin not correctly defined" << std::endl;
            }
            node = fSettings["Camera2.lappingEnd"];
            if(!node.empty() && node.isInt())
            {
                rightLappingEnd = node.operator int();
            }
            else
            {
                std::cout << "WARNING: Camera2.lappingEnd not correctly defined" << std::endl;
            }

            node = fSettings["Tlr"];
            if(!node.empty())
            {
                mTlr = node.mat();
                if(mTlr.rows != 3 || mTlr.cols != 4)
                {
                    std::cerr << "*Tlr matrix have to be a 3x4 transformation matrix*" << std::endl;
                    b_miss_params = true;
                }
            }
            else
            {
                std::cerr << "*Tlr matrix doesn't exist*" << std::endl;
                b_miss_params = true;
            }

            if(!b_miss_params)
            {
                static_cast<KannalaBrandt8*>(mpCamera)->mvLappingArea[0] = leftLappingBegin;
                static_cast<KannalaBrandt8*>(mpCamera)->mvLappingArea[1] = leftLappingEnd;

                mpFrameDrawer->both = true;

                vector<float> vCamCalib2{fx,fy,cx,cy,k1,k2,k3,k4};
                mpCamera2 = new KannalaBrandt8(vCamCalib2);

                static_cast<KannalaBrandt8*>(mpCamera2)->mvLappingArea[0] = rightLappingBegin;
                static_cast<KannalaBrandt8*>(mpCamera2)->mvLappingArea[1] = rightLappingEnd;

                std::cout << "- Camera1 Lapping: " << leftLappingBegin << ", " << leftLappingEnd << std::endl;

                std::cout << std::endl << "Camera2 Parameters:" << std::endl;
                std::cout << "- Camera: Fisheye" << std::endl;
                std::cout << "- fx: " << fx << std::endl;
                std::cout << "- fy: " << fy << std::endl;
                std::cout << "- cx: " << cx << std::endl;
                std::cout << "- cy: " << cy << std::endl;
                std::cout << "- k1: " << k1 << std::endl;
                std::cout << "- k2: " << k2 << std::endl;
                std::cout << "- k3: " << k3 << std::endl;
                std::cout << "- k4: " << k4 << std::endl;

                std::cout << "- mTlr: \n" << mTlr << std::endl;

                std::cout << "- Camera2 Lapping: " << rightLappingBegin << ", " << rightLappingEnd << std::endl;
            }
        }

        if(b_miss_params)
        {
            return false;
        }

        mpAtlas->AddCamera(mpCamera);
        mpAtlas->AddCamera(mpCamera2);
    }
    else
    {
        std::cerr << "*Not Supported Camera Sensor*" << std::endl;
        std::cerr << "Check an example configuration file with the desired sensor" << std::endl;
    }
    //IMP 第三次改这里 rgb的基线长度不对
    if(mSensor==System::STEREO || mSensor==System::IMU_STEREO || mSensor==System::RGBD)
    {
        cv::FileNode node = fSettings["Camera.bf"];
        if(!node.empty() && node.isReal())
        {
            mbf = node.real();
        }
        else
        {
            std::cerr << "*Camera.bf parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

    }

    float fps = fSettings["Camera.fps"];
    if(fps==0)
        fps=30;

    // Max/Min Frames to insert keyframes and to check relocalisation
    mMinFrames = 0;
    mMaxFrames = fps;

    cout << "- fps: " << fps << endl;


    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;

    if(mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    if(mSensor==System::STEREO || mSensor==System::RGBD || mSensor==System::IMU_STEREO)
    {
        float fx = mpCamera->getParameter(0);
        cv::FileNode node = fSettings["ThDepth"];
        if(!node.empty()  && node.isReal())
        {
            mThDepth = node.real();
            mThDepth = mbf*mThDepth/fx;
            cout << endl << "Depth Threshold (Close/Far Points-Lines): " << mThDepth << endl;
        }
        else
        {
            std::cerr << "*ThDepth parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }


    }

    if(mSensor==System::RGBD)
    {
        cv::FileNode node = fSettings["DepthMapFactor"];
        if(!node.empty() && node.isReal())
        {
            mDepthMapFactor = node.real();
            if(fabs(mDepthMapFactor)<1e-5)
                mDepthMapFactor=1;
            else
                mDepthMapFactor = 1.0f/mDepthMapFactor;
        }
        else
        {
            std::cerr << "*DepthMapFactor parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

    }

    if(b_miss_params)
    {
        return false;
    }

    return true;
}

bool Tracking::ParseORBParamFile(cv::FileStorage &fSettings)
{
    bool b_miss_params = false;
    int nFeatures, nLevels, fIniThFAST, fMinThFAST;
    float fScaleFactor;

    cv::FileNode node = fSettings["ORBextractor.nFeatures"];
    if(!node.empty() && node.isInt())
    {
        nFeatures = node.operator int();
    }
    else
    {
        std::cerr << "*ORBextractor.nFeatures parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["ORBextractor.scaleFactor"];
    if(!node.empty() && node.isReal())
    {
        fScaleFactor = node.real();
    }
    else
    {
        std::cerr << "*ORBextractor.scaleFactor parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["ORBextractor.nLevels"];
    if(!node.empty() && node.isInt())
    {
        nLevels = node.operator int();
    }
    else
    {
        std::cerr << "*ORBextractor.nLevels parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["ORBextractor.iniThFAST"];
    if(!node.empty() && node.isInt())
    {
        fIniThFAST = node.operator int();
    }
    else
    {
        std::cerr << "*ORBextractor.iniThFAST parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["ORBextractor.minThFAST"];
    if(!node.empty() && node.isInt())
    {
        fMinThFAST = node.operator int();
    }
    else
    {
        std::cerr << "*ORBextractor.minThFAST parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    if(b_miss_params)
    {
        return false;
    }

    mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST,mSensor);//IMP 第三次

    if(mSensor==System::STEREO || mSensor==System::IMU_STEREO)
        mpORBextractorRight = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST,mSensor);

    if(mSensor==System::MONOCULAR || mSensor==System::IMU_MONOCULAR)
        mpIniORBextractor = new ORBextractor(5*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST,mSensor);

    cout << endl << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
    cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

    return true;
}

bool Tracking::ParseLineParamFile(cv::FileStorage &fSettings)
{
    bool b_miss_params = false;
    int lsd_nfeatures, lsd_refine, extractor, levels;
    float lsd_scale, scale;
    double min_line_length;

    cv::FileNode node = fSettings["nfeatures"];
    if(!node.empty() && node.isInt())
    {
        lsd_nfeatures = node.operator int();
    }
    else
    {
        std::cerr << "*nfeatures parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["lsd_refine"];
    if(!node.empty() && node.isInt())
    {
        lsd_refine = node.operator int();
    }
    else
    {
        std::cerr << "*lsd_refine parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["lsd_scale"];
    if(!node.empty() && node.isReal())
    {
        lsd_scale = node.real();
    }
    else
    {
        std::cerr << "*lsd_scale parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["levels"];
    if(!node.empty() && node.isInt() && (node.operator int()==1 || node.operator int()==2))
    {
        levels = node.operator int();
    }
    else
    {
        std::cerr << "*levels parameter doesn't exist or is not an integer equal to 1 or 2*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["scale"];
    if(!node.empty() && node.isReal())
    {
        scale = node.real();
    }
    else
    {
        std::cerr << "*scale parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    } 

    node = fSettings["extractor"];
    if(!node.empty() && node.isInt() && (node.operator int()==0 || node.operator int()==1))
    {
        extractor = node.operator int();
    }
    else
    {
        std::cerr << "*extractor parameter doesn't exist or is not an integer equal to 0 or 1*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["SLAM"];
    if(!node.empty() && node.isInt() && (node.operator int()==0 || node.operator int()==1 || node.operator int()==2 || node.operator int()==3))
    {
        SLAM = node.operator int();
    }
    else
    {
        std::cerr << "*SLAM parameter doesn't exist or is not an integer equal to 0 or 1 or 2 or 3*" << std::endl;
        b_miss_params = true;
    }


    if(b_miss_params)
    {
        return false;
    }
    //myline 加线提取器 IMP 线提取器实例化
    mpLineextractorLeft = new Lineextractor(lsd_nfeatures, lsd_refine, lsd_scale, levels, scale, extractor);
    mpLineextractorRight = new Lineextractor(lsd_nfeatures, lsd_refine, lsd_scale, levels, scale, extractor);

    if(extractor==0)
        cout << endl << "Line Detector: LSD" << endl;
    else if(extractor==1)
        cout << endl << "Line Detector: ED Lines" << endl;
    

    cout << endl  << "Line Extractor Parameters: " << endl;
    if(lsd_nfeatures!=0)
        cout << "- Number of Line Features: " << lsd_nfeatures << endl;
    else 
        cout << "- Using All Line Features " << endl;
    if(extractor==0) 
    {
        cout << "- Refinement Factor: " << lsd_refine << endl;
        cout << "- Image Scale Factor: " << lsd_scale << endl;
    }
    cout << "- Levels of the Pyramid: " << levels << endl;
    cout << "- Scale of the Pyramid: " << scale << endl <<endl;

    if(SLAM==0)
        cout << "Point Line SLAM" << endl << endl;
    if(SLAM==1)
        cout << "Only Line Pose Estimation and Mapping using the Euclidean distance as an error function (Loop Closing thread is deactivated)" << endl << endl;
    if(SLAM==2)
        cout << "Only Line Pose Estimation and Mapping using the angular distance as an error function (Loop Closing thread is deactivated)" << endl << endl;
    if(SLAM==3)
        cout << "Only Line Pose Estimation and Mapping using both the Euclidean and angular distance as an error function (Loop Closing thread is deactivated)" << endl <<endl;

    return true;
}

bool Tracking::ParseIMUParamFile(cv::FileStorage &fSettings)
{
    bool b_miss_params = false;

    cv::Mat Tbc;
    cv::FileNode node = fSettings["Tbc"];
    if(!node.empty())
    {
        Tbc = node.mat();
        if(Tbc.rows != 4 || Tbc.cols != 4)
        {
            std::cerr << "*Tbc matrix have to be a 4x4 transformation matrix*" << std::endl;
            b_miss_params = true;
        }
    }
    else
    {
        std::cerr << "*Tbc matrix doesn't exist*" << std::endl;
        b_miss_params = true;
    }

    cout << endl;

    cout << "Left camera to Imu Transform (Tbc): " << endl << Tbc << endl;

    float freq, Ng, Na, Ngw, Naw;

    node = fSettings["IMU.Frequency"];
    if(!node.empty() && node.isInt())
    {
        freq = node.operator int();
    }
    else
    {
        std::cerr << "*IMU.Frequency parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["IMU.NoiseGyro"];
    if(!node.empty() && node.isReal())
    {
        Ng = node.real();
    }
    else
    {
        std::cerr << "*IMU.NoiseGyro parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["IMU.NoiseAcc"];
    if(!node.empty() && node.isReal())
    {
        Na = node.real();
    }
    else
    {
        std::cerr << "*IMU.NoiseAcc parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["IMU.GyroWalk"];
    if(!node.empty() && node.isReal())
    {
        Ngw = node.real();
    }
    else
    {
        std::cerr << "*IMU.GyroWalk parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["IMU.AccWalk"];
    if(!node.empty() && node.isReal())
    {
        Naw = node.real();
    }
    else
    {
        std::cerr << "*IMU.AccWalk parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    if(b_miss_params)
    {
        return false;
    }

    const float sf = sqrt(freq);
    cout << endl;
    cout << "IMU frequency: " << freq << " Hz" << endl;
    cout << "IMU gyro noise: " << Ng << " rad/s/sqrt(Hz)" << endl;
    cout << "IMU gyro walk: " << Ngw << " rad/s^2/sqrt(Hz)" << endl;
    cout << "IMU accelerometer noise: " << Na << " m/s^2/sqrt(Hz)" << endl;
    cout << "IMU accelerometer walk: " << Naw << " m/s^3/sqrt(Hz)" << endl;

    mpImuCalib = new IMU::Calib(Tbc,Ng*sf,Na*sf,Ngw/sf,Naw/sf);

    mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(),*mpImuCalib);


    return true;
}

void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}

void Tracking::SetLoopClosing(LoopClosing *pLoopClosing)
{
    mpLoopClosing=pLoopClosing;
}

void Tracking::SetViewer(Viewer *pViewer)
{
    mpViewer=pViewer;
}

//change 添加这一函数
void Tracking::SetDetector(YOLOV5* pDetector)
{
    mpDetector = pDetector;
}

//myplan 添加
void Tracking::set_planar_mapping_module(Planar_Mapping_module *planar_mapper)
{
//    spdlog::info(">tracking< link planar_mapper to tracker");
    _planar_mapper = planar_mapper;
}

void Tracking::SetStepByStep(bool bSet)
{
    bStepByStep = bSet;
}

// 输入左右目图像，可以为RGB、BGR、RGBA、GRAY
// 1、将图像转为mImGray和imGrayRight并初始化mCurrentFrame
// 2、进行tracking过程
// 输出世界坐标系到该帧相机坐标系的变换矩阵
    //myline 准备进入双目追踪函数
cv::Mat Tracking::GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp, string filename)
{
//    std::cout << "enter GrabImageStereo" << std::endl;

    mImGray = imRectLeft;//存储左目图像
    cv::Mat imGrayRight = imRectRight;//存储右目图像
    mImRight = imRectRight;

    /// imp 将当前Frame的保存
    cv::Mat im1 = imRectLeft;
    cv::imwrite("/home/lzh/VINS/SPL_SLAM/Exp_Picture/pl_all/orginal_frame.jpg", im1);

    if(isYoloObject)//change 添加这一函数
    {
        ///*******************************YOLO*************** 这样操作 实际YOLO中还是检测的彩色图 maybe

        cv::Mat InputImage;//图像容器
        InputImage = mImGray.clone();//将输入图像克隆到定义容器
        //For KITTI
        //imp 双目数据集非kitti需要修改这
        mpDetector->image_width = 1241;//设置图像的宽
        mpDetector->image_height = 376;//高
        ////将输入图像变为cvmat类型，存储在YOLOV5类成员变量mRGB中，主要是往yolo检测器中传递图像用的
        mpDetector->GetImage(InputImage);
        mpDetector->Detect();//执行目标检测操作
        //将目标检测器中的动态区域信息赋值给左侧 ORB 特征提取器
        mpORBextractorLeft->mvDynamicArea = mpDetector->mvDynamicArea;
        //调试用
//        cout << "mpDetector->mvDynamicArea.size()" <<mpDetector->mvDynamicArea.size()<<  endl;
        mpORBextractorLeft->mClassnames = mpDetector->mClassnames;//MYLK 新增
        mmDetectMap = mpDetector->mmDetectMap;//MYLK 新增
        mpLineextractorLeft->mvDynamicArea =mpDetector->mvDynamicArea;//imp 新增
        mpFrameDrawer->mvDynamicArea =mpDetector->mvDynamicArea;//IMP 第四次修改
        {
            //互斥锁
            std::unique_lock<std::mutex> lock(mpViewer->mMutexPAFinsh);
            mpViewer->mmDetectMap = mpDetector->mmDetectMap;//将检测框
            mpViewer->mClassnames = mpDetector->mClassnames;//检测类别传到viewer
        }
    }

    if(mImGray.channels()==3)
    {
        // mystep 1 ：将RGB或RGBA图像转为灰度图像
        if(mbRGB)//标志位
        {
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_RGB2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_BGR2GRAY);
        }
    }
        // 这里考虑得十分周全,甚至连四通道的图像都考虑到了
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_RGBA2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_BGRA2GRAY);
        }
    }
    // 双目模式，注意跟两个相机模式区分开
    //立体匹配下的双目
//    myline 需要重新构造Frame类，添加左右目线提取器的参数
//    std::cout<< "before frame"<<std::endl;
    if (mSensor == System::STEREO && !mpCamera2)
    {
        mCurrentFrame = Frame(mImGray, imGrayRight,
                              mmDetectMap,                //mylk 新加的
                              mSensor,                       //mylk 新加的
                              timestamp,
                              mpORBextractorLeft,
                              mpORBextractorRight,
                              mpLineextractorLeft,
                              mpLineextractorRight, mpORBVocabulary,
                              mpLineVocabulary,
                              mK,
                              mDistCoef,
                              mbf,
                              mThDepth,
                              mpCamera);
    }
//        std::cout<< "after frame"<<std::endl;
        //    mylk 改动
//    else if(mSensor == System::IMU_STEREO && !mpCamera2)
//        mCurrentFrame = Frame(mImGray,imGrayRight, timestamp,mpORBextractorLeft, mpORBextractorRight, mpLineextractorLeft, mpLineextractorRight, mpORBVocabulary, mpLineVocabulary, mK,mDistCoef,mbf,mThDepth,mpCamera,&mLastFrame,*mpImuCalib);
  /* else if(mSensor == System::STEREO && mpCamera2)
        mCurrentFrame = Frame(mImGray,imGrayRight,timestamp,mpORBextractorLeft,mpORBextractorRight,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth,mpCamera,mpCamera2,mTlr);
        //mCurrentFrame = Frame(mImGray,imGrayRight,timestamp,mpORBextractorLeft, mpORBextractorRight, mpLineextractorLeft, mpLineextractorRight, mpORBVocabulary, mpLineVocabulary,mK,mDistCoef,mbf,mThDepth,mpCamera,&mLastFrame,*mpImuCalib);
    else if(mSensor == System::IMU_STEREO && mpCamera2)
        mCurrentFrame = Frame(mImGray,imGrayRight,timestamp,mpORBextractorLeft,mpORBextractorRight,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth,mpCamera,mpCamera2,mTlr,&mLastFrame,*mpImuCalib); */


    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    mCurrentFrame.mNameFile = filename;
    mCurrentFrame.mnDataset = mnNumDataset;
    // mylk 改动
    mCurrentFrame.mClassnames = mpViewer->mClassnames;
    cv::Mat k;
    // mylk Remove dynamic points
    mCurrentFrame.StereoCalculEverything(mImGray,imGrayRight,mpDetector->mmDetectMap);
    // myStep 2 ：进入跟踪追踪函数
    TrackWithLines(timestamp);///myline 重写函数
        if(isYoloObject)
        {
            mpDetector->mvDynamicArea.clear();
            mpDetector->mmDetectMap.clear();
            mpLineextractorLeft->mvDynamicArea.clear();//imp mylk 去除线完毕需要清空容器
            mpDetector->mmDetectMap.clear();
            this->mmDetectMap.clear();//mylk 新增
        }

//        if(isYoloObject)
//        {
//            mpDetector->mvDynamicArea.clear();
//            mpDetector->mmDetectMap.clear();
//        }

//    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
//
//    double t_track = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t1 - t0).count();


    /*cout << "trracking time: " << t_track << endl;
    f_track_stats << setprecision(0) << mCurrentFrame.mTimeStamp*1e9 << ",";
    f_track_stats << mvpLocalKeyFrames.size() << ",";
    f_track_stats << mvpLocalMapPoints.size() << ",";
    f_track_stats << setprecision(6) << t_track << endl;*/

#ifdef SAVE_TIMES
    f_track_times << mCurrentFrame.mTimeTotal_Ext << ",";
    f_track_times << mCurrentFrame.mTimeStereoMatchTotal << ",";
    f_track_times << mTime_PreIntIMU << ",";
    f_track_times << mTime_PosePred << ",";
    f_track_times << mTime_LocalMapTrack << ",";
    f_track_times << mTime_NewKF_Dec << ",";
    f_track_times << t_track << endl;
#endif

    return mCurrentFrame.mTcw.clone();
}


cv::Mat Tracking::GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const cv::Mat &img_seg_mask,const double &timestamp, string filename)
    {
        //myplan 多了这两行
        frame_id_tracking++;
        cv::Mat rawImage = imRGB.clone();

        mImGray = imRGB;
        mImDepth = imD;
        mImRGB = imRGB;
        cv::Mat imDepth = imD;
        _img_seg_mask= img_seg_mask;

// STEP object detection.++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//这个isyoloobject 从pangolin中在mapviewer中点击，如果选择则显示目标框
        if(isYoloObject)//change 添加这一函数
        {
            cv::Mat InputImage;
            InputImage = imRGB.clone();
            auto start = std::chrono::high_resolution_clock::now();
            mpDetector->GetImage(InputImage);
            mpDetector->Detect();
            auto end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
            // It should be known that it takes longer time at first time
//        std::cout << "Process a frame takes : " << duration.count() << " ms" << std::endl;

            mpORBextractorLeft->mvDynamicArea = mpDetector->mvDynamicArea;
            mpORBextractorLeft->mClassnames = mpDetector->mClassnames;//MYLK 新增
            mmDetectMap = mpDetector->mmDetectMap;//MYLK 新增
            mpLineextractorLeft->mvDynamicArea = mpDetector->mvDynamicArea;//imp 新增
            mpFrameDrawer->mvDynamicArea = mpDetector->mvDynamicArea;//IMP 第四次修改
            {
                std::unique_lock<std::mutex> lock(mpViewer->mMutexPAFinsh);
                mpViewer->mmDetectMap = mpDetector->mmDetectMap;//将检测框
                mpViewer->mClassnames = mpDetector->mClassnames;//检测类别传到viewer
            }
        }
// STEP object detection END++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


        if(mImGray.channels()==3)
        {
            if(mbRGB)
                cvtColor(mImGray,mImGray,CV_RGB2GRAY);
            else
                cvtColor(mImGray,mImGray,CV_BGR2GRAY);
        }
        else if(mImGray.channels()==4)
        {
            if(mbRGB)
                cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
            else
                cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
        }
        // undistort image
        //myplan 这里多两个去畸变的函数，通常在Frame中，感觉没什么用，
        //灰度图像去畸变
        cv::Mat gray_imu;
        cv::undistort(mImGray, gray_imu, mK, mDistCoef);

        //彩色图像去畸变
        cv::Mat rgb_imu;
        cv::undistort(imRGB, rgb_imu, mK, mDistCoef);
//这两个数据用来半稠密建图

        if((fabs(mDepthMapFactor-1.0f)>1e-5) || imDepth.type()!=CV_32F)
            imDepth.convertTo(imDepth,CV_32F,mDepthMapFactor);

        // Step 3：构造Frame
//        std::cout<<"before frame"<<std::endl;
        mCurrentFrame = Frame(//IMP 顺序必须三个一致
                  rawImage,       // myplan 彩色图
                  mImGray,
                  imDepth,
                  _img_seg_mask,             //myplan
                  mmDetectMap,                //mylk 新加的
                  mSensor,                       //mylk 新加的
                  timestamp,
                  mpORBextractorLeft,
                  mpLineextractorLeft,// // myplan 线提取器
                  mpORBVocabulary,
                  mpLineVocabulary,
                  mK,
                  mDistCoef,
                  mbf,
                  mThDepth,
                  mpCamera,
                  gray_imu,//myplan 灰度图去畸变 用来半稠密建图
                  rgb_imu);//myplan 彩色图去畸变
//    for (unsigned int i1 = 0; i1 < mCurrentFrame.mvDepth_l.size(); ++i1) {
//        std::cout << "after frame 线点的深度值" << mCurrentFrame.mvDepth_l[i1].first << mCurrentFrame.mvDepth_l[i1].second
//                  << std::endl;
//    }
//        std::cout<<"after frame"<<std::endl;
/// imp 将当前Frame的保存
    cv::Mat im1 = mImGray;
//    cv::imwrite("/home/lzh/VINS/SPL_SLAM/Exp_Picture/pl_all/orginal_frame.jpg", im1);

        mCurrentFrame.mNameFile = filename;
        mCurrentFrame.mnDataset = mnNumDataset;

        //mylk 新增
        mCurrentFrame.mClassnames = mpViewer->mClassnames;
        cv::Mat k;
        // Remove dynamic points
//        std::cout<<"before cal"<<std::endl;
        mCurrentFrame.CalculEverything(mImRGB,mImGray,imDepth,k,mpDetector->mmDetectMap);
//        std::cout<<"after cal"<<std::endl;

//        std::cout<<"mstate="<<mState<<std::endl;

    // STEP get current camera groundtruth by timestamp. +++++++++++++++++++++++++++++++++++++++++++
    // notice: only use the first frame's pose.
    //myplan 利用第一帧的真值对初始化的位姿进行优化
    string timestamp_string = to_string(timestamp);
    string timestamp_short_string = timestamp_string.substr(0, timestamp_string.length() - 4);

    Eigen::MatrixXd truth_frame_poses(1, 8); // camera pose Eigen format.
    cv::Mat cam_pose_mat;                    // camera pose Mat format.

    for (auto &row : mGroundtruth_mat)
    {
        string row_string = to_string(row[0]);//1341847980.79
        string row_short_string = row_string.substr(0, row_string.length() - 4);

        if (row_short_string == timestamp_short_string)
        {
            // vector --> Eigen.
            for (int i = 0; i < (int)row.size(); i++)
            {
                truth_frame_poses(0) = row[0];
                truth_frame_poses(1) = row[1];
                truth_frame_poses(2) = row[2];
                truth_frame_poses(3) = row[3];
                truth_frame_poses(4) = row[4];
                truth_frame_poses(5) = row[5];
                truth_frame_poses(6) = row[6];
                truth_frame_poses(7) = row[7];
            }

            // Eigen --> SE3.
            g2o::SE3Quat cam_pose_se3(truth_frame_poses.row(0).tail<7>());
            // std::cout << "cam_pose_se3\n" << cam_pose_se3 << std::endl;

            // SE3 --> Mat.
            cam_pose_mat = Converter::toCvMat(cam_pose_se3);

            // save to current frame.
            mCurrentFrame.mGroundtruthPose_mat = cam_pose_mat;
            if (!mCurrentFrame.mGroundtruthPose_mat.empty())
            {
                mCurrentFrame.mGroundtruthPose_eigen = Converter::toEigenMatrixXd(mCurrentFrame.mGroundtruthPose_mat);
            }
            break;
        }
        else
        {
            mCurrentFrame.mGroundtruthPose_mat = cv::Mat::zeros(4, 4, CV_8UC1);
            mCurrentFrame.mGroundtruthPose_eigen = Eigen::Matrix4d::Zero(4, 4);
        }

    }
    // get the camera groundtruth by timestamp. ----------------------------------------------------------------------

//    std::cout<<"mCurrentFrame.mGroundtruthPose_mat=="<<mCurrentFrame.mGroundtruthPose_mat<<std::endl;
        TrackWithLines(timestamp); // 执行需要加锁的函数


        if(isYoloObject)//mylk 改动和双目的不一样
        {
            mpDetector->mmDetectMap.clear();
            this->mmDetectMap.clear();//mylk 新增
            mpDetector->mvDynamicArea.clear();
            mpORBextractorLeft->mvDynamicArea.clear();
            mpLineextractorLeft->mvDynamicArea.clear();//imp mylk 去除线完毕需要清空容器
        }

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();



        /*f_track_stats << setprecision(0) << mCurrentFrame.mTimeStamp*1e9 << ",";
        f_track_stats << mvpLocalKeyFrames.size() << ",";
        f_track_stats << mvpLocalMapPoints.size() << ",";
        f_track_stats << setprecision(6) << t_track << endl;*/

#ifdef SAVE_TIMES
        f_track_times << mCurrentFrame.mTimeORB_Ext << ",";
    f_track_times << mCurrentFrame.mTimeStereoMatch << ",";
    f_track_times << mTime_PreIntIMU << ",";
    f_track_times << mTime_PosePred << ",";
    f_track_times << mTime_LocalMapTrack << ",";
    f_track_times << mTime_NewKF_Dec << ",";
    f_track_times << t_track << endl;
#endif

        return mCurrentFrame.mTcw.clone();
    }


cv::Mat Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp, string filename)
{
    mImGray = im;

    if(isYoloObject)//change 添加这一函数
    {
        ///*******************************YOLO*************** 这样操作 实际YOLO中还是检测的彩色图 maybe

        cv::Mat InputImage;
        InputImage = mImGray.clone();
        mpDetector->image_width = 640;
        mpDetector->image_height = 480;
        mpDetector->GetImage(InputImage);
        mpDetector->Detect();
        mpORBextractorLeft->mvDynamicArea = mpDetector->mvDynamicArea;
        {
            std::unique_lock<std::mutex> lock(mpViewer->mMutexPAFinsh);
            mpViewer->mmDetectMap = mpDetector->mmDetectMap;
            mpViewer->mClassnames = mpDetector->mClassnames;
        }
    }

    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }

    if (mSensor == System::MONOCULAR)
    {
        if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET ||(lastID - initID) < mMaxFrames)
            mCurrentFrame = Frame(mImGray,timestamp,mpIniORBextractor,mpORBVocabulary,mpCamera,mDistCoef,mbf,mThDepth);
        else
            mCurrentFrame = Frame(mImGray,timestamp,mpORBextractorLeft,mpORBVocabulary,mpCamera,mDistCoef,mbf,mThDepth);
    }
    else if(mSensor == System::IMU_MONOCULAR)
    {
        if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET)
        {
            mCurrentFrame = Frame(mImGray,timestamp,mpIniORBextractor,mpORBVocabulary,mpCamera,mDistCoef,mbf,mThDepth,&mLastFrame,*mpImuCalib);
        }
        else
            mCurrentFrame = Frame(mImGray,timestamp,mpORBextractorLeft,mpORBVocabulary,mpCamera,mDistCoef,mbf,mThDepth,&mLastFrame,*mpImuCalib);
    }

    if (mState==NO_IMAGES_YET)
        t0=timestamp;

    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();

    mCurrentFrame.mNameFile = filename;
    mCurrentFrame.mnDataset = mnNumDataset;

    lastID = mCurrentFrame.mnId;
    Track();

    if(isYoloObject)//change 添加这一函数
    {
        mpDetector->mvDynamicArea.clear();
        mpDetector->mmDetectMap.clear();
    }

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    double t_track = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t1 - t0).count();

    /*f_track_stats << setprecision(0) << mCurrentFrame.mTimeStamp*1e9 << ",";
    f_track_stats << mvpLocalKeyFrames.size() << ",";
    f_track_stats << mvpLocalMapPoints.size() << ",";
    f_track_stats << setprecision(6) << t_track << endl;*/

#ifdef SAVE_TIMES
    f_track_times << mCurrentFrame.mTimeORB_Ext << ",";
    f_track_times << mCurrentFrame.mTimeStereoMatch << ",";
    f_track_times << mTime_PreIntIMU << ",";
    f_track_times << mTime_PosePred << ",";
    f_track_times << mTime_LocalMapTrack << ",";
    f_track_times << mTime_NewKF_Dec << ",";
    f_track_times << t_track << endl;
#endif

    return mCurrentFrame.mTcw.clone();
}


//myplan
void Tracking::SetSemiDenseMapping(ProbabilityMapping *pSemiDenseMapping)
{
    mpSemiDenseMapping = pSemiDenseMapping;
}



void Tracking::GrabImuData(const IMU::Point &imuMeasurement)
{
    unique_lock<mutex> lock(mMutexImuQueue);
    mlQueueImuData.push_back(imuMeasurement);
}

void Tracking::PreintegrateIMU()
{
    //cout << "start preintegration" << endl;

    if(!mCurrentFrame.mpPrevFrame)
    {
        Verbose::PrintMess("non prev frame ", Verbose::VERBOSITY_NORMAL);
        mCurrentFrame.setIntegrated();
        return;
    }

    // cout << "start loop. Total meas:" << mlQueueImuData.size() << endl;

    mvImuFromLastFrame.clear();
    mvImuFromLastFrame.reserve(mlQueueImuData.size());
    if(mlQueueImuData.size() == 0)
    {
        Verbose::PrintMess("Not IMU data in mlQueueImuData!!", Verbose::VERBOSITY_NORMAL);
        mCurrentFrame.setIntegrated();
        return;
    }

    while(true)
    {
        bool bSleep = false;
        {
            unique_lock<mutex> lock(mMutexImuQueue);
            if(!mlQueueImuData.empty())
            {
                IMU::Point* m = &mlQueueImuData.front();
                cout.precision(17);
                if(m->t<mCurrentFrame.mpPrevFrame->mTimeStamp-0.001l)
                {
                    mlQueueImuData.pop_front();
                }
                else if(m->t<mCurrentFrame.mTimeStamp-0.001l)
                {
                    mvImuFromLastFrame.push_back(*m);
                    mlQueueImuData.pop_front();
                }
                else
                {
                    mvImuFromLastFrame.push_back(*m);
                    break;
                }
            }
            else
            {
                break;
                bSleep = true;
            }
        }
        if(bSleep)
            usleep(500);
    }


    const int n = mvImuFromLastFrame.size()-1;
    IMU::Preintegrated* pImuPreintegratedFromLastFrame = new IMU::Preintegrated(mLastFrame.mImuBias,mCurrentFrame.mImuCalib);

    for(int i=0; i<n; i++)
    {
        float tstep;
        cv::Point3f acc, angVel;
        if((i==0) && (i<(n-1)))
        {
            float tab = mvImuFromLastFrame[i+1].t-mvImuFromLastFrame[i].t;
            float tini = mvImuFromLastFrame[i].t-mCurrentFrame.mpPrevFrame->mTimeStamp;
            acc = (mvImuFromLastFrame[i].a+mvImuFromLastFrame[i+1].a-
                    (mvImuFromLastFrame[i+1].a-mvImuFromLastFrame[i].a)*(tini/tab))*0.5f;
            angVel = (mvImuFromLastFrame[i].w+mvImuFromLastFrame[i+1].w-
                    (mvImuFromLastFrame[i+1].w-mvImuFromLastFrame[i].w)*(tini/tab))*0.5f;
            tstep = mvImuFromLastFrame[i+1].t-mCurrentFrame.mpPrevFrame->mTimeStamp;
        }
        else if(i<(n-1))
        {
            acc = (mvImuFromLastFrame[i].a+mvImuFromLastFrame[i+1].a)*0.5f;
            angVel = (mvImuFromLastFrame[i].w+mvImuFromLastFrame[i+1].w)*0.5f;
            tstep = mvImuFromLastFrame[i+1].t-mvImuFromLastFrame[i].t;
        }
        else if((i>0) && (i==(n-1)))
        {
            float tab = mvImuFromLastFrame[i+1].t-mvImuFromLastFrame[i].t;
            float tend = mvImuFromLastFrame[i+1].t-mCurrentFrame.mTimeStamp;
            acc = (mvImuFromLastFrame[i].a+mvImuFromLastFrame[i+1].a-
                    (mvImuFromLastFrame[i+1].a-mvImuFromLastFrame[i].a)*(tend/tab))*0.5f;
            angVel = (mvImuFromLastFrame[i].w+mvImuFromLastFrame[i+1].w-
                    (mvImuFromLastFrame[i+1].w-mvImuFromLastFrame[i].w)*(tend/tab))*0.5f;
            tstep = mCurrentFrame.mTimeStamp-mvImuFromLastFrame[i].t;
        }
        else if((i==0) && (i==(n-1)))
        {
            acc = mvImuFromLastFrame[i].a;
            angVel = mvImuFromLastFrame[i].w;
            tstep = mCurrentFrame.mTimeStamp-mCurrentFrame.mpPrevFrame->mTimeStamp;
        }

        if (!mpImuPreintegratedFromLastKF)
            cout << "mpImuPreintegratedFromLastKF does not exist" << endl;
        mpImuPreintegratedFromLastKF->IntegrateNewMeasurement(acc,angVel,tstep);
        pImuPreintegratedFromLastFrame->IntegrateNewMeasurement(acc,angVel,tstep);
    }

    mCurrentFrame.mpImuPreintegratedFrame = pImuPreintegratedFromLastFrame;
    mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;
    mCurrentFrame.mpLastKeyFrame = mpLastKeyFrame;

    mCurrentFrame.setIntegrated();

    Verbose::PrintMess("Preintegration is finished!! ", Verbose::VERBOSITY_DEBUG);
}


bool Tracking::PredictStateIMU()
{
    if(!mCurrentFrame.mpPrevFrame)
    {
        Verbose::PrintMess("No last frame", Verbose::VERBOSITY_NORMAL);
        return false;
    }

    if(mbMapUpdated && mpLastKeyFrame)
    {
        const cv::Mat twb1 = mpLastKeyFrame->GetImuPosition();
        const cv::Mat Rwb1 = mpLastKeyFrame->GetImuRotation();
        const cv::Mat Vwb1 = mpLastKeyFrame->GetVelocity();

        const cv::Mat Gz = (cv::Mat_<float>(3,1) << 0,0,-IMU::GRAVITY_VALUE);
        const float t12 = mpImuPreintegratedFromLastKF->dT;

        cv::Mat Rwb2 = IMU::NormalizeRotation(Rwb1*mpImuPreintegratedFromLastKF->GetDeltaRotation(mpLastKeyFrame->GetImuBias()));
        cv::Mat twb2 = twb1 + Vwb1*t12 + 0.5f*t12*t12*Gz+ Rwb1*mpImuPreintegratedFromLastKF->GetDeltaPosition(mpLastKeyFrame->GetImuBias());
        cv::Mat Vwb2 = Vwb1 + t12*Gz + Rwb1*mpImuPreintegratedFromLastKF->GetDeltaVelocity(mpLastKeyFrame->GetImuBias());
        mCurrentFrame.SetImuPoseVelocity(Rwb2,twb2,Vwb2);
        mCurrentFrame.mPredRwb = Rwb2.clone();
        mCurrentFrame.mPredtwb = twb2.clone();
        mCurrentFrame.mPredVwb = Vwb2.clone();
        mCurrentFrame.mImuBias = mpLastKeyFrame->GetImuBias();
        mCurrentFrame.mPredBias = mCurrentFrame.mImuBias;
        return true;
    }
    else if(!mbMapUpdated)
    {
        const cv::Mat twb1 = mLastFrame.GetImuPosition();
        const cv::Mat Rwb1 = mLastFrame.GetImuRotation();
        const cv::Mat Vwb1 = mLastFrame.mVw;
        const cv::Mat Gz = (cv::Mat_<float>(3,1) << 0,0,-IMU::GRAVITY_VALUE);
        const float t12 = mCurrentFrame.mpImuPreintegratedFrame->dT;

        cv::Mat Rwb2 = IMU::NormalizeRotation(Rwb1*mCurrentFrame.mpImuPreintegratedFrame->GetDeltaRotation(mLastFrame.mImuBias));
        cv::Mat twb2 = twb1 + Vwb1*t12 + 0.5f*t12*t12*Gz+ Rwb1*mCurrentFrame.mpImuPreintegratedFrame->GetDeltaPosition(mLastFrame.mImuBias);
        cv::Mat Vwb2 = Vwb1 + t12*Gz + Rwb1*mCurrentFrame.mpImuPreintegratedFrame->GetDeltaVelocity(mLastFrame.mImuBias);

        mCurrentFrame.SetImuPoseVelocity(Rwb2,twb2,Vwb2);
        mCurrentFrame.mPredRwb = Rwb2.clone();
        mCurrentFrame.mPredtwb = twb2.clone();
        mCurrentFrame.mPredVwb = Vwb2.clone();
        mCurrentFrame.mImuBias =mLastFrame.mImuBias;
        mCurrentFrame.mPredBias = mCurrentFrame.mImuBias;
        return true;
    }
    else
        cout << "not IMU prediction!!" << endl;

    return false;
}


void Tracking::ComputeGyroBias(const vector<Frame*> &vpFs, float &bwx,  float &bwy, float &bwz)
{
    const int N = vpFs.size();
    vector<float> vbx;
    vbx.reserve(N);
    vector<float> vby;
    vby.reserve(N);
    vector<float> vbz;
    vbz.reserve(N);

    cv::Mat H = cv::Mat::zeros(3,3,CV_32F);
    cv::Mat grad  = cv::Mat::zeros(3,1,CV_32F);
    for(int i=1;i<N;i++)
    {
        Frame* pF2 = vpFs[i];
        Frame* pF1 = vpFs[i-1];
        cv::Mat VisionR = pF1->GetImuRotation().t()*pF2->GetImuRotation();
        cv::Mat JRg = pF2->mpImuPreintegratedFrame->JRg;
        cv::Mat E = pF2->mpImuPreintegratedFrame->GetUpdatedDeltaRotation().t()*VisionR;
        cv::Mat e = IMU::LogSO3(E);
        assert(fabs(pF2->mTimeStamp-pF1->mTimeStamp-pF2->mpImuPreintegratedFrame->dT)<0.01);

        cv::Mat J = -IMU::InverseRightJacobianSO3(e)*E.t()*JRg;
        grad += J.t()*e;
        H += J.t()*J;
    }

    cv::Mat bg = -H.inv(cv::DECOMP_SVD)*grad;
    bwx = bg.at<float>(0);
    bwy = bg.at<float>(1);
    bwz = bg.at<float>(2);

    for(int i=1;i<N;i++)
    {
        Frame* pF = vpFs[i];
        pF->mImuBias.bwx = bwx;
        pF->mImuBias.bwy = bwy;
        pF->mImuBias.bwz = bwz;
        pF->mpImuPreintegratedFrame->SetNewBias(pF->mImuBias);
        pF->mpImuPreintegratedFrame->Reintegrate();
    }
}

void Tracking::ComputeVelocitiesAccBias(const vector<Frame*> &vpFs, float &bax,  float &bay, float &baz)
{
    const int N = vpFs.size();
    const int nVar = 3*N +3; // 3 velocities/frame + acc bias
    const int nEqs = 6*(N-1);

    cv::Mat J(nEqs,nVar,CV_32F,cv::Scalar(0));
    cv::Mat e(nEqs,1,CV_32F,cv::Scalar(0));
    cv::Mat g = (cv::Mat_<float>(3,1)<<0,0,-IMU::GRAVITY_VALUE);

    for(int i=0;i<N-1;i++)
    {
        Frame* pF2 = vpFs[i+1];
        Frame* pF1 = vpFs[i];
        cv::Mat twb1 = pF1->GetImuPosition();
        cv::Mat twb2 = pF2->GetImuPosition();
        cv::Mat Rwb1 = pF1->GetImuRotation();
        cv::Mat dP12 = pF2->mpImuPreintegratedFrame->GetUpdatedDeltaPosition();
        cv::Mat dV12 = pF2->mpImuPreintegratedFrame->GetUpdatedDeltaVelocity();
        cv::Mat JP12 = pF2->mpImuPreintegratedFrame->JPa;
        cv::Mat JV12 = pF2->mpImuPreintegratedFrame->JVa;
        float t12 = pF2->mpImuPreintegratedFrame->dT;
        // Position p2=p1+v1*t+0.5*g*t^2+R1*dP12
        J.rowRange(6*i,6*i+3).colRange(3*i,3*i+3) += cv::Mat::eye(3,3,CV_32F)*t12;
        J.rowRange(6*i,6*i+3).colRange(3*N,3*N+3) += Rwb1*JP12;
        e.rowRange(6*i,6*i+3) = twb2-twb1-0.5f*g*t12*t12-Rwb1*dP12;
        // Velocity v2=v1+g*t+R1*dV12
        J.rowRange(6*i+3,6*i+6).colRange(3*i,3*i+3) += -cv::Mat::eye(3,3,CV_32F);
        J.rowRange(6*i+3,6*i+6).colRange(3*(i+1),3*(i+1)+3) += cv::Mat::eye(3,3,CV_32F);
        J.rowRange(6*i+3,6*i+6).colRange(3*N,3*N+3) -= Rwb1*JV12;
        e.rowRange(6*i+3,6*i+6) = g*t12+Rwb1*dV12;
    }

    cv::Mat H = J.t()*J;
    cv::Mat B = J.t()*e;
    cv::Mat x(nVar,1,CV_32F);
    cv::solve(H,B,x);

    bax = x.at<float>(3*N);
    bay = x.at<float>(3*N+1);
    baz = x.at<float>(3*N+2);

    for(int i=0;i<N;i++)
    {
        Frame* pF = vpFs[i];
        x.rowRange(3*i,3*i+3).copyTo(pF->mVw);
        if(i>0)
        {
            pF->mImuBias.bax = bax;
            pF->mImuBias.bay = bay;
            pF->mImuBias.baz = baz;
            pF->mpImuPreintegratedFrame->SetNewBias(pF->mImuBias);
        }
    }
}

void Tracking::ResetFrameIMU()
{
    // TODO To implement...
}


void Tracking::Track()
{
#ifdef SAVE_TIMES
    mTime_PreIntIMU = 0;
    mTime_PosePred = 0;
    mTime_LocalMapTrack = 0;
    mTime_NewKF_Dec = 0;
#endif

    if (bStepByStep)
    {
        while(!mbStep)
            usleep(500);
        mbStep = false;
    }

    if(mpLocalMapper->mbBadImu)
    {
        cout << "TRACK: Reset map because local mapper set the bad imu flag " << endl;
        mpSystem->ResetActiveMap();
        return;
    }

    Map* pCurrentMap = mpAtlas->GetCurrentMap();

    if(mState!=NO_IMAGES_YET)
    {
        if(mLastFrame.mTimeStamp>mCurrentFrame.mTimeStamp)
        {
            cerr << "ERROR: Frame with a timestamp older than previous frame detected!" << endl;
            unique_lock<mutex> lock(mMutexImuQueue);
            mlQueueImuData.clear();
            CreateMapInAtlas();
            return;
        }
        else if(mCurrentFrame.mTimeStamp>mLastFrame.mTimeStamp+1.0)
        {
            cout << "id last: " << mLastFrame.mnId << "    id curr: " << mCurrentFrame.mnId << endl;
            if(mpAtlas->isInertial())
            {

                if(mpAtlas->isImuInitialized())
                {
                    cout << "Timestamp jump detected. State set to LOST. Reseting IMU integration..." << endl;
                    if(!pCurrentMap->GetIniertialBA2())
                    {
                        mpSystem->ResetActiveMap();
                    }
                    else
                    {
                        CreateMapInAtlas();
                    }
                }
                else
                {
                    cout << "Timestamp jump detected, before IMU initialization. Reseting..." << endl;
                    mpSystem->ResetActiveMap();
                }
            }

            return;
        }
    }

    // myStep 3 IMU模式下设置IMU的Bias参数,还要保证上一帧存在
    if ((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO) && mpLastKeyFrame)
        mCurrentFrame.SetNewBias(mpLastKeyFrame->GetImuBias());

    if(mState==NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }

    mLastProcessedState=mState;
    // Step 4 IMU模式且没有创建地图的情况下对IMU数据进行预积分
    if ((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO) && !mbCreatedMap)
    {
#ifdef SAVE_TIMES
        std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
#endif
        PreintegrateIMU();
#ifdef SAVE_TIMES
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        mTime_PreIntIMU = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t1 - t0).count();
#endif


    }
    mbCreatedMap = false;

    // Get Map Mutex -> Map cannot be changed
    unique_lock<mutex> lock(pCurrentMap->mMutexMapUpdate);

    mbMapUpdated = false;

    int nCurMapChangeIndex = pCurrentMap->GetMapChangeIndex();
    int nMapChangeIndex = pCurrentMap->GetLastMapChange();
    if(nCurMapChangeIndex>nMapChangeIndex)
    {
        // cout << "Map update detected" << endl;
        pCurrentMap->SetLastMapChange(nCurMapChangeIndex);
        mbMapUpdated = true;
    }

    // myStep 5 初始化
    if(mState==NOT_INITIALIZED)
    {
        if(mSensor==System::STEREO || mSensor==System::RGBD || mSensor==System::IMU_STEREO)
            StereoInitialization();
        else
        {
            MonocularInitialization();
        }

        mpFrameDrawer->Update(this);

        if(mState!=OK) // If rightly initialized, mState=OK
        {
            mLastFrame = Frame(mCurrentFrame);
            return;
        }

        if(mpAtlas->GetAllMaps().size() == 1)
        {
            mnFirstFrameId = mCurrentFrame.mnId;
        }
    }
    else
    {
        // System is initialized. Track Frame.
        bool bOK;

        // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
        if(!mbOnlyTracking)
        {
#ifdef SAVE_TIMES
        std::chrono::steady_clock::time_point timeStartPosePredict = std::chrono::steady_clock::now();
#endif

            // State OK
            // Local Mapping is activated. This is the normal behaviour, unless
            // you explicitly activate the "only tracking" mode.
            if(mState==OK)
            {

                // Local Mapping might have changed some MapPoints tracked in last frame
                CheckReplacedInLastFrame();

                if((mVelocity.empty() && !pCurrentMap->isImuInitialized()) || mCurrentFrame.mnId<mnLastRelocFrameId+2)
                {
                    //Verbose::PrintMess("TRACK: Track with respect to the reference KF ", Verbose::VERBOSITY_DEBUG);
                    bOK = TrackReferenceKeyFrame();
                }
                else
                {
                    //Verbose::PrintMess("TRACK: Track with motion model", Verbose::VERBOSITY_DEBUG);
                    bOK = TrackWithMotionModel();
                    if(!bOK)
                        bOK = TrackReferenceKeyFrame();
                }


                if (!bOK)
                {
                    if ( mCurrentFrame.mnId<=(mnLastRelocFrameId+mnFramesToResetIMU) &&
                         (mSensor==System::IMU_MONOCULAR || mSensor==System::IMU_STEREO))
                    {
                        mState = LOST;
                    }
                    else if(pCurrentMap->KeyFramesInMap()>10)
                    {
                        cout << "KF in map: " << pCurrentMap->KeyFramesInMap() << endl;
                        mState = RECENTLY_LOST;
                        mTimeStampLost = mCurrentFrame.mTimeStamp;
                        //mCurrentFrame.SetPose(mLastFrame.mTcw);
                    }
                    else
                    {
                        mState = LOST;
                    }
                }
            }
            else
            {

                if (mState == RECENTLY_LOST)
                {
                    Verbose::PrintMess("Lost for a short time", Verbose::VERBOSITY_NORMAL);

                    bOK = true;
                    if((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO))
                    {
                        if(pCurrentMap->isImuInitialized())
                            PredictStateIMU();
                        else
                            bOK = false;

                        if (mCurrentFrame.mTimeStamp-mTimeStampLost>time_recently_lost)
                        {
                            mState = LOST;
                            Verbose::PrintMess("Track Lost...", Verbose::VERBOSITY_NORMAL);
                            bOK=false;
                        }
                    }
                    else
                    {
                        // TODO fix relocalization
                        bOK = Relocalization();
                        if(!bOK)
                        {
                            mState = LOST;
                            Verbose::PrintMess("Track Lost...", Verbose::VERBOSITY_NORMAL);
                            bOK=false;
                        }
                    }
                }
                else if (mState == LOST)
                {

                    Verbose::PrintMess("A new map is started...", Verbose::VERBOSITY_NORMAL);

                    if (pCurrentMap->KeyFramesInMap()<10)
                    {
                        mpSystem->ResetActiveMap();
                        cout << "Reseting current map..." << endl;
                    }else
                        CreateMapInAtlas();

                    if(mpLastKeyFrame)
                        mpLastKeyFrame = static_cast<KeyFrame*>(NULL);

                    Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

                    return;
                }
            }


#ifdef SAVE_TIMES
        std::chrono::steady_clock::time_point timeEndPosePredict = std::chrono::steady_clock::now();

        mTime_PosePred = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(timeEndPosePredict - timeStartPosePredict).count();
#endif

        }
        else
        {
            // Localization Mode: Local Mapping is deactivated (TODO Not available in inertial mode)
            if(mState==LOST)
            {
                if(mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO)
                    Verbose::PrintMess("IMU. State LOST", Verbose::VERBOSITY_NORMAL);
                bOK = Relocalization();
            }
            else
            {
                if(!mbVO)
                {
                    // In last frame we tracked enough MapPoints in the map
                    if(!mVelocity.empty())
                    {
                        bOK = TrackWithMotionModel();
                    }
                    else
                    {
                        bOK = TrackReferenceKeyFrame();
                    }
                }
                else
                {
                    // In last frame we tracked mainly "visual odometry" points.

                    // We compute two camera poses, one from motion model and one doing relocalization.
                    // If relocalization is sucessfull we choose that solution, otherwise we retain
                    // the "visual odometry" solution.

                    bool bOKMM = false;
                    bool bOKReloc = false;
                    vector<MapPoint*> vpMPsMM;
                    vector<bool> vbOutMM;
                    cv::Mat TcwMM;
                    if(!mVelocity.empty())
                    {
                        bOKMM = TrackWithMotionModel();
                        vpMPsMM = mCurrentFrame.mvpMapPoints;
                        vbOutMM = mCurrentFrame.mvbOutlier;
                        TcwMM = mCurrentFrame.mTcw.clone();
                    }
                    bOKReloc = Relocalization();

                    if(bOKMM && !bOKReloc)
                    {
                        mCurrentFrame.SetPose(TcwMM);
                        mCurrentFrame.mvpMapPoints = vpMPsMM;
                        mCurrentFrame.mvbOutlier = vbOutMM;

                        if(mbVO)
                        {
                            for(int i =0; i<mCurrentFrame.N; i++)
                            {
                                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                                {
                                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                                }
                            }
                        }
                    }
                    else if(bOKReloc)
                    {
                        mbVO = false;
                    }

                    bOK = bOKReloc || bOKMM;
                }
            }
        }

        if(!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;

        // If we have an initial estimation of the camera pose and matching. Track the local map.
        if(!mbOnlyTracking)
        {
            if(bOK)
            {
#ifdef SAVE_TIMES
                std::chrono::steady_clock::time_point time_StartTrackLocalMap = std::chrono::steady_clock::now();
#endif
                bOK = TrackLocalMap();
#ifdef SAVE_TIMES
                std::chrono::steady_clock::time_point time_EndTrackLocalMap = std::chrono::steady_clock::now();

                mTime_LocalMapTrack = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndTrackLocalMap - time_StartTrackLocalMap).count();
#endif


            }
            if(!bOK)
                cout << "Fail to track local map!" << endl;
        }
        else
        {
            // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
            // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
            // the camera we will use the local map again.
            if(bOK && !mbVO)
                bOK = TrackLocalMap();
        }

        if(bOK)
            mState = OK;
        else if (mState == OK)
        {
            if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO)
            {
                Verbose::PrintMess("Track lost for less than one second...", Verbose::VERBOSITY_NORMAL);
                if(!pCurrentMap->isImuInitialized() || !pCurrentMap->GetIniertialBA2())
                {
                    cout << "IMU is not or recently initialized. Reseting active map..." << endl;
                    mpSystem->ResetActiveMap();
                }

                mState=RECENTLY_LOST;
            }
            else
                mState=LOST; // visual to lost

            if(mCurrentFrame.mnId>mnLastRelocFrameId+mMaxFrames)
            {
                mTimeStampLost = mCurrentFrame.mTimeStamp;
            }
        }

        // Save frame if recent relocalization, since they are used for IMU reset (as we are making copy, it shluld be once mCurrFrame is completely modified)
        if((mCurrentFrame.mnId<(mnLastRelocFrameId+mnFramesToResetIMU)) && (mCurrentFrame.mnId > mnFramesToResetIMU) && ((mSensor == System::IMU_MONOCULAR) || (mSensor == System::IMU_STEREO)) && pCurrentMap->isImuInitialized())
        {
            // TODO check this situation
            Verbose::PrintMess("Saving pointer to frame. imu needs reset...", Verbose::VERBOSITY_NORMAL);
            Frame* pF = new Frame(mCurrentFrame);
            pF->mpPrevFrame = new Frame(mLastFrame);

            // Load preintegration
            pF->mpImuPreintegratedFrame = new IMU::Preintegrated(mCurrentFrame.mpImuPreintegratedFrame);
        }

        if(pCurrentMap->isImuInitialized())
        {
            if(bOK)
            {
                if(mCurrentFrame.mnId==(mnLastRelocFrameId+mnFramesToResetIMU))
                {
                    cout << "RESETING FRAME!!!" << endl;
                    ResetFrameIMU();
                }
                else if(mCurrentFrame.mnId>(mnLastRelocFrameId+30))
                    mLastBias = mCurrentFrame.mImuBias;
            }
        }

        // Update drawer
        mpFrameDrawer->Update(this);
        if(!mCurrentFrame.mTcw.empty())
            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

        if(bOK || mState==RECENTLY_LOST)
        {
            // Update motion model
            if(!mLastFrame.mTcw.empty() && !mCurrentFrame.mTcw.empty())
            {
                cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
                mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
                mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
                mVelocity = mCurrentFrame.mTcw*LastTwc;
            }
            else
                mVelocity = cv::Mat();

            if(mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO)
                mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

            // Clean VO matches
            for(int i=0; i<mCurrentFrame.N; i++)
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(pMP)
                    if(pMP->Observations()<1)
                    {
                        mCurrentFrame.mvbOutlier[i] = false;
                        mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                    }
            }

            // Delete temporal MapPoints
            for(list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend =  mlpTemporalPoints.end(); lit!=lend; lit++)
            {
                MapPoint* pMP = *lit;
                delete pMP;
            }
            mlpTemporalPoints.clear();

#ifdef SAVE_TIMES
            std::chrono::steady_clock::time_point timeStartNewKF = std::chrono::steady_clock::now();
#endif
            bool bNeedKF = NeedNewKeyFrame();
#ifdef SAVE_TIMES
            std::chrono::steady_clock::time_point timeEndNewKF = std::chrono::steady_clock::now();

            mTime_NewKF_Dec = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(timeEndNewKF - timeStartNewKF).count();
#endif



            // Check if we need to insert a new keyframe
            if(bNeedKF && (bOK|| (mState==RECENTLY_LOST && (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO))))
                CreateNewKeyFrame();

            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame. Only has effect if lastframe is tracked
            for(int i=0; i<mCurrentFrame.N;i++)
            {
                if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
            }
        }

        // Reset if the camera get lost soon after initialization
        if(mState==LOST)
        {
            if(pCurrentMap->KeyFramesInMap()<=5)
            {
                mpSystem->ResetActiveMap();
                return;
            }
            if ((mSensor == System::IMU_MONOCULAR) || (mSensor == System::IMU_STEREO))
                if (!pCurrentMap->isImuInitialized())
                {
                    Verbose::PrintMess("Track lost before IMU initialisation, reseting...", Verbose::VERBOSITY_QUIET);
                    mpSystem->ResetActiveMap();
                    return;
                }

            CreateMapInAtlas();
        }

        if(!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;

        mLastFrame = Frame(mCurrentFrame);
    }




    if(mState==OK || mState==RECENTLY_LOST)
    {
        // Store frame pose information to retrieve the complete camera trajectory afterwards.
        if(!mCurrentFrame.mTcw.empty())
        {
            cv::Mat Tcr = mCurrentFrame.mTcw*mCurrentFrame.mpReferenceKF->GetPoseInverse();
            mlRelativeFramePoses.push_back(Tcr);
            mlpReferences.push_back(mCurrentFrame.mpReferenceKF);
            mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
            mlbLost.push_back(mState==LOST);
        }
        else
        {
            // This can happen if tracking is lost
            mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
            mlpReferences.push_back(mlpReferences.back());
            mlFrameTimes.push_back(mlFrameTimes.back());
            mlbLost.push_back(mState==LOST);
        }

    }
}
/**
 * @brief 跟踪过程，包括恒速模型跟踪、参考关键帧跟踪、局部地图跟踪
 * track包含两部分：估计运动、跟踪局部地图
 *
 * Step 1：初始化
 * Step 2：跟踪
 * Step 3：记录位姿信息，用于轨迹复现
 */
void Tracking::TrackWithLines(const double &timestamp)
{
#ifdef SAVE_TIMES
    mTime_PreIntIMU = 0;
    mTime_PosePred = 0;
    mTime_LocalMapTrack = 0;
    mTime_NewKF_Dec = 0;
#endif
//    std::cout<<"Tracking Sensor==="<<mSensor<<std::endl;
    if (bStepByStep)
    {
        while(!mbStep)
            usleep(500);
        mbStep = false;
    }

    // myStep 1 如局部建图里认为IMU有问题，重置当前活跃地图
    if(mpLocalMapper->mbBadImu)
    {
        cout << "TRACK: Reset map because local mapper set the bad imu flag " << endl;
        mpSystem->ResetActiveMap();
        return;
    }

    Map* pCurrentMap = mpAtlas->GetCurrentMap();
// myStep 2 处理时间戳异常的情况
    if(mState!=NO_IMAGES_YET)
    {
        if(mLastFrame.mTimeStamp>mCurrentFrame.mTimeStamp)
        {
            ///上一帧的时间戳大于当前帧时间戳有问题
            cerr << "ERROR: Frame with a timestamp older than previous frame detected!" << endl;
            unique_lock<mutex> lock(mMutexImuQueue);
            mlQueueImuData.clear();
            // 创建新地图
            CreateMapInAtlas();
            return;
        }
        /// 如果当前图像时间戳和前一帧图像时间戳大于1s，说明时间戳明显跳变了，重置地图后直接返回
        else if(mCurrentFrame.mTimeStamp>mLastFrame.mTimeStamp+1.0)
        {
            cout << "id last: " << mLastFrame.mnId << "    id curr: " << mCurrentFrame.mnId << endl;
            //根据是否是imu模式,进行imu的补偿
            if(mpAtlas->isInertial())
            {
                // 如果当前地图imu成功初始化
                if(mpAtlas->isImuInitialized())
                {
                    cout << "Timestamp jump detected. State set to LOST. Reseting IMU integration..." << endl;
                    // IMU完成第3次初始化（在localmapping线程里）
                    if(!pCurrentMap->GetIniertialBA2())
                    {
                        // 如果当前子图中imu没有经过BA2，重置active地图，也就是之前的数据不要了
                        mpSystem->ResetActiveMap();
                    }
                    else
                    {
                        // 如果当前子图中imu进行了BA2，重新创建新的子图，保存当前地图
                        CreateMapInAtlas();
                    }
                }
                else
                {
                    cout << "Timestamp jump detected, before IMU initialization. Reseting..." << endl;
                    mpSystem->ResetActiveMap();
                }
            }

            return;
        }
    }

        // myStep 3 IMU模式下设置IMU的Bias参数,还要保证上一帧存在
    if ((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO) && mpLastKeyFrame)
        mCurrentFrame.SetNewBias(mpLastKeyFrame->GetImuBias());

    if(mState==NO_IMAGES_YET)
    {

        mState = NOT_INITIALIZED;
    }

    mLastProcessedState=mState;
        // myStep  4 IMU模式且没有创建地图的情况下对IMU数据进行预积分
    if ((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO) && !mbCreatedMap)
    {
#ifdef SAVE_TIMES
        std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
#endif
        // IMU数据进行预积分
        PreintegrateIMU();
#ifdef SAVE_TIMES
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        mTime_PreIntIMU = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t1 - t0).count();
#endif


    }
    mbCreatedMap = false;

    // Get Map Mutex -> Map cannot be changed
    // 地图更新时加锁。保证地图不会发生变化
    // 疑问:这样子会不会影响地图的实时更新?
    // 回答：主要耗时在构造帧中特征点的提取和匹配部分,在那个时候地图是没有被上锁的,有足够的时间更新地图
    unique_lock<mutex> lock(pCurrentMap->mMutexMapUpdate);

    mbMapUpdated = false;///track刚进来改变
        // 判断地图id是否更新了
    int nCurMapChangeIndex = pCurrentMap->GetMapChangeIndex();
    int nMapChangeIndex = pCurrentMap->GetLastMapChange();
    if(nCurMapChangeIndex>nMapChangeIndex)
    {
        // cout << "Map update detected" << endl;
        // 检测到地图更新了，当前帧索引比上一帧大
        pCurrentMap->SetLastMapChange(nCurMapChangeIndex);
        mbMapUpdated = true;
    }

        // myStep 5 初始化
    if(mState==NOT_INITIALIZED)
    {
        // mynote 双目RGBD相机的初始化共用一个函数
        StereoInitializationWithLines();//myline
//        StereoInitialization();//rgbd 用这个
        mpFrameDrawer->Update(this);

        if(mState!=OK) // If rightly initialized, mState=OK
        {
            mLastFrame = Frame(mCurrentFrame);
            return;
        }

        if(mpAtlas->GetAllMaps().size() == 1)
        {
            mnFirstFrameId = mCurrentFrame.mnId;
        }
    }
    else
    {
        // System is initialized. Track Frame.
        bool bOK;

        // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
        if(!mbOnlyTracking)
        {
#ifdef SAVE_TIMES
        std::chrono::steady_clock::time_point timeStartPosePredict = std::chrono::steady_clock::now();
#endif

            // State OK
            // Local Mapping is activated. This is the normal behaviour, unless
            // you explicitly activate the "only tracking" mode.
            if(mState==OK)
            {
                // Local Mapping might have changed some MapPoints and MapLines tracked in last frame
                // myStep 6.1 检查并更新上一帧被替换的MapPoints
                // 局部建图线程则可能会对原有的地图点进行替换.在这里进行检查
                CheckReplacedInLastFrameWithLines();//myline 函数重写
                // myStep 6.2 运动模型是空的并且imu未初始化或刚完成重定位，跟踪参考关键帧；否则恒速模型跟踪
                // 第一个条件,如果运动模型为空并且imu未初始化,说明是刚开始第一帧跟踪，或者已经跟丢了。
                // 第二个条件,如果当前帧紧紧地跟着在重定位的帧的后面，我们用重定位帧来恢复位姿
                // mnLastRelocFrameId 上一次重定位的那一帧
                if((mVelocity.empty() && !pCurrentMap->isImuInitialized()) || mCurrentFrame.mnId<mnLastRelocFrameId+2)
                {
                    //Verbose::PrintMess("TRACK: Track with respect to the reference KF ", Verbose::VERBOSITY_DEBUG);

                    bOK = TrackReferenceKeyFrameWithLines();///关键帧跟踪 myline 重写
                }
                else
                {
                    //Verbose::PrintMess("TRACK: Track with motion model", Verbose::VERBOSITY_DEBUG);
                    // 用恒速模型跟踪。所谓的恒速就是假设上上帧到上一帧的位姿=上一帧的位姿到当前帧位姿
                    // 根据恒速模型设定当前帧的初始位姿，用最近的普通帧来跟踪当前的普通帧
                    // 通过投影的方式在参考帧中找当前帧特征点的匹配点，优化每个特征点所对应3D点的投影误差即可得到位姿
                   //myplan 主要修改这里
                    bOK = TrackWithMotionModelWithLines(timestamp);///恒速模型跟踪 myline 重写
                    if(!bOK)
                    // 根据恒速模型失败了，只能根据参考关键帧来跟踪
                        bOK = TrackReferenceKeyFrameWithLines();///关键帧跟踪
                }


                if (!bOK)
                {
                    if (mCurrentFrame.mnId<=(mnLastRelocFrameId+mnFramesToResetIMU) &&
                         (mSensor==System::IMU_STEREO))
                    {
                        mState = LOST;
                    }
                    else if(pCurrentMap->KeyFramesInMap()>10)
                    {
                        // 条件1：当前地图中关键帧数目较多（大于10）
                        // 条件2（隐藏条件）：当前帧距离上次重定位帧超过1s（说明还比较争气，值的救）或者非IMU模式
                        // 同时满足条件1，2，则将状态标记为RECENTLY_LOST，
                        cout << "KF in map: " << pCurrentMap->KeyFramesInMap() << endl;
                        mState = RECENTLY_LOST;
                        // 记录丢失时间
                        mTimeStampLost = mCurrentFrame.mTimeStamp;
                        //mCurrentFrame.SetPose(mLastFrame.mTcw);
                    }
                    else
                    {
                        mState = LOST;
                    }
                }
            }
            else
            {
                // 如果是RECENTLY_LOST状态
                if (mState == RECENTLY_LOST)
                {
                    Verbose::PrintMess("Lost for a short time", Verbose::VERBOSITY_NORMAL);
                    // bOK先置为true
                    bOK = true;
                    if((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO))
                    {
                        // IMU模式下可以用IMU来预测位姿，看能否拽回来
                        // myStep 6.4 如果当前地图中IMU已经成功初始化，就用IMU数据预测位姿
                        if(pCurrentMap->isImuInitialized())
                            PredictStateIMU();
                        else
                            bOK = false;

                        if (mCurrentFrame.mTimeStamp-mTimeStampLost>time_recently_lost)
                        {
                            // 如果IMU模式下当前帧距离跟丢帧超过5s还没有找回（time_recently_lost默认为5s）
                            // 放弃了，将RECENTLY_LOST状态改为LOST
                            mState = LOST;
                            Verbose::PrintMess("Track Lost...", Verbose::VERBOSITY_NORMAL);
                            bOK=false;
                        }
                    }
                    else//非imu模式下重定位跟踪
                    {
                        // TODO fix relocalization
                        bOK = Relocalization();
                        if(!bOK)
                        {
                            mState = LOST;
                            Verbose::PrintMess("Track Lost...", Verbose::VERBOSITY_NORMAL);
                            bOK=false;
                        }
                    }
                }
                else if (mState == LOST)
                {

                    Verbose::PrintMess("A new map is started...", Verbose::VERBOSITY_NORMAL);

                    if (pCurrentMap->KeyFramesInMap()<10)
                    {
                        mpSystem->ResetActiveMap();
                        cout << "Reseting current map..." << endl;
                    }else
                        CreateMapInAtlas();

                    if(mpLastKeyFrame)
                        mpLastKeyFrame = static_cast<KeyFrame*>(NULL);

                    Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

                    return;
                }
            }


#ifdef SAVE_TIMES
        std::chrono::steady_clock::time_point timeEndPosePredict = std::chrono::steady_clock::now();

        mTime_PosePred = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(timeEndPosePredict - timeStartPosePredict).count();
#endif

        }
        else
        {
            // Localization Mode: Local Mapping is deactivated (TODO Not available in inertial mode)
            if(mState==LOST)
            {
                if(mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO)
                    Verbose::PrintMess("IMU. State LOST", Verbose::VERBOSITY_NORMAL);
                // // Step 6.1 LOST状态进行重定位
                bOK = Relocalization();
            }
            else
            {
                // mbVO是mbOnlyTracking为true时的才有的一个变量
                // mbVO为false表示此帧匹配了很多的MapPoints，跟踪很正常 (注意有点反直觉)
                // mbVO为true表明此帧匹配了很少的MapPoints，少于10个，要跟丢
                if(!mbVO)
                {
                    // In last frame we tracked enough MapPoints in the map
                    // In last frame we tracked enough MapPoints in the map
                    // Step 6.2 如果跟踪状态正常，使用恒速模型或参考关键帧跟踪
                    if(!mVelocity.empty())
                    {
                        // 优先使用恒速模型跟踪
                        bOK = TrackWithMotionModelWithLines(timestamp);
                    }
                    else
                    {
                        // 如果恒速模型不被满足,那么就只能够通过参考关键帧来跟踪
                        bOK = TrackReferenceKeyFrameWithLines();
                    }
                }
                else
                {
                    // In last frame we tracked mainly "visual odometry" points.

                    // We compute two camera poses, one from motion model and one doing relocalization.
                    // If relocalization is sucessfull we choose that solution, otherwise we retain
                    // the "visual odometry" solution.
                    // mbVO为true，表明此帧匹配了很少（小于10）的地图点，要跟丢的节奏，既做跟踪又做重定位

                    // MM=Motion Model,通过运动模型进行跟踪的结果
                    bool bOKMM = false;
                    // 通过重定位方法来跟踪的结果
                    bool bOKReloc = false;
                    // 运动模型中构造的地图点
                    vector<MapPoint*> vpMPsMM;
                    // 在追踪运动模型后发现的外点
                    vector<bool> vbOutMM;
                    // 运动模型得到的位姿
                    cv::Mat TcwMM;
                    // myStep 6.3 当运动模型有效的时候,根据运动模型计算位姿
                    if(!mVelocity.empty())
                    {
                        // 将恒速模型跟踪结果暂存到这几个变量中，因为后面重定位会改变这些变量
                        bOKMM = TrackWithMotionModelWithLines(timestamp);
                        vpMPsMM = mCurrentFrame.mvpMapPoints;
                        vbOutMM = mCurrentFrame.mvbOutlier;
                        TcwMM = mCurrentFrame.mTcw.clone();
                    }
                    // Step 6.4 使用重定位的方法来得到当前帧的位姿
                    bOKReloc = Relocalization();

                    if(bOKMM && !bOKReloc)
                    {
                        // 恒速模型成功、重定位失败，重新使用之前暂存的恒速模型结果
                        mCurrentFrame.SetPose(TcwMM);
                        mCurrentFrame.mvpMapPoints = vpMPsMM;
                        mCurrentFrame.mvbOutlier = vbOutMM;

                        if(mbVO)
                        {
                            // 如果当前帧匹配的3D点很少，增加当前可视地图点的被观测次数
                            for(int i =0; i<mCurrentFrame.N; i++)
                            {
                                // 如果这个特征点形成了地图点,并且也不是外点的时候
                                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                                {
                                    // 增加被找到次数
                                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                                }
                            }
                        }
                    }
                    else if(bOKReloc)
                    {
                        // 只要重定位成功整个跟踪过程正常进行（重定位与跟踪，更相信重定位）
                        mbVO = false;
                    }

                    bOK = bOKReloc || bOKMM;
                }
            }
        }
        // 将最新的关键帧作为当前帧的参考关键帧
        // mpReferenceKF先是上一时刻的参考关键帧，如果当前为新关键帧则变成当前关键帧，如果不是新的关键帧则先为上一帧的参考关键帧，而后经过更新局部关键帧重新确定
        if(!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;

        // If we have an initial estimation of the camera pose and matching. Track the local map.
        // Step 7 在跟踪得到当前帧初始姿态后，现在对local map进行跟踪得到更多的匹配，并优化当前位姿
        // 前面只是跟踪一帧得到初始位姿，这里搜索局部关键帧、局部地图点，和当前帧进行投影匹配，得到更多匹配的MapPoints后进行Pose优化
        // 在帧间匹配得到初始的姿态后，现在对local map进行跟踪得到更多的匹配，并优化当前位姿
        // local map:当前帧、当前帧的MapPoints、当前关键帧与其它关键帧共视关系
        // 前面主要是两两跟踪（恒速模型跟踪上一帧、跟踪参考帧），这里搜索局部关键帧后搜集所有局部MapPoints，
        // 然后将局部MapPoints和当前帧进行投影匹配，得到更多匹配的MapPoints后进行Pose优化
        if(!mbOnlyTracking)
        {
            if(bOK)
            {
#ifdef SAVE_TIMES
                std::chrono::steady_clock::time_point time_StartTrackLocalMap = std::chrono::steady_clock::now();
#endif
                // 局部地图跟踪
                bOK = TrackLocalMapWithLines();///myline 重写局部地图跟踪
#ifdef SAVE_TIMES
                std::chrono::steady_clock::time_point time_EndTrackLocalMap = std::chrono::steady_clock::now();

                mTime_LocalMapTrack = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndTrackLocalMap - time_StartTrackLocalMap).count();
#endif
            }
            if(!bOK)
                cout << "Fail to track local map!" << endl;
            
        }
        else
        {
            // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
            // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
            // the camera we will use the local map again.
            if(bOK && !mbVO)
                bOK = TrackLocalMapWithLines();//局部地图跟踪是否成功
        }
        // 到此为止跟踪确定位姿阶段结束，下面开始做收尾工作和为下一帧做准备

        // 查看到此为止时的两个状态变化
        // bOK的历史变化---上一帧跟踪成功---当前帧跟踪成功---局部地图跟踪成功---true                     -->OK   1 跟踪局部地图成功
        //          \               \              \---局部地图跟踪失败---false
        //           \               \---当前帧跟踪失败---false
        //            \---上一帧跟踪失败---重定位成功---局部地图跟踪成功---true                       -->OK  2 重定位
        //                          \           \---局部地图跟踪失败---false
        //                           \---重定位失败---false

        //
        // mState的历史变化---上一帧跟踪成功---当前帧跟踪成功---局部地图跟踪成功---OK                  -->OK  1 跟踪局部地图成功
        //            \               \              \---局部地图跟踪失败---OK                  -->OK  3 正常跟踪
        //             \               \---当前帧跟踪失败---非OK
        //              \---上一帧跟踪失败---重定位成功---局部地图跟踪成功---非OK
        //                            \           \---局部地图跟踪失败---非OK
        //                             \---重定位失败---非OK（传不到这里，因为直接return了）
        // 由上图可知当前帧的状态OK的条件是跟踪局部地图成功，重定位或正常跟踪都可
        // Step 8 根据上面的操作来判断是否追踪成功
        if(bOK)
            // 此时还OK才说明跟踪成功了
            mState = OK;
        // 由上图可知只有当第一阶段跟踪成功，但第二
        else if (mState == OK)
        {
            if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO)
            {
                Verbose::PrintMess("Track lost for less than one second...", Verbose::VERBOSITY_NORMAL);
                if(!pCurrentMap->isImuInitialized() || !pCurrentMap->GetIniertialBA2())
                {
                    cout << "IMU is not or recently initialized. Reseting active map..." << endl;
                    mpSystem->ResetActiveMap();
                }
                // 状态变为最近丢失
                mState=RECENTLY_LOST;
            }
            else
                mState=LOST; // visual to lost

            if(mCurrentFrame.mnId>mnLastRelocFrameId+mMaxFrames)
            {
                mTimeStampLost = mCurrentFrame.mTimeStamp;
            }
        }

        // Save frame if recent relocalization, since they are used for IMU reset (as we are making copy, it shluld be once mCurrFrame is completely modified)
        // 这段貌似没啥作用
        if((mCurrentFrame.mnId<(mnLastRelocFrameId+mnFramesToResetIMU)) && (mCurrentFrame.mnId > mnFramesToResetIMU) && ((mSensor == System::IMU_MONOCULAR) || (mSensor == System::IMU_STEREO)) && pCurrentMap->isImuInitialized())
        {
            // TODO check this situation
            Verbose::PrintMess("Saving pointer to frame. imu needs reset...", Verbose::VERBOSITY_NORMAL);
            Frame* pF = new Frame(mCurrentFrame);
            pF->mpPrevFrame = new Frame(mLastFrame);

            // Load preintegration
            pF->mpImuPreintegratedFrame = new IMU::Preintegrated(mCurrentFrame.mpImuPreintegratedFrame);
        }
        // 下面代码没有用
        if(pCurrentMap->isImuInitialized())
        {
            if(bOK)
            {
                if(mCurrentFrame.mnId==(mnLastRelocFrameId+mnFramesToResetIMU))
                {
                    cout << "RESETING FRAME!!!" << endl;
                    ResetFrameIMU();
                }
                else if(mCurrentFrame.mnId>(mnLastRelocFrameId+30))
                    mLastBias = mCurrentFrame.mImuBias;
            }
        }

        // Update drawer
        // 更新显示线程中的图像、特征点、地图点等信息
        mpFrameDrawer->Update(this);
        if(!mCurrentFrame.mTcw.empty())
            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);
        // 查看到此为止时的两个状态变化
        // bOK的历史变化---上一帧跟踪成功---当前帧跟踪成功---局部地图跟踪成功---true
        //          \               \              \---局部地图跟踪失败---false
        //           \               \---当前帧跟踪失败---false
        //            \---上一帧跟踪失败---重定位成功---局部地图跟踪成功---true
        //                          \           \---局部地图跟踪失败---false
        //                           \---重定位失败---false

        // mState的历史变化---上一帧跟踪成功---当前帧跟踪成功---局部地图跟踪成功---OK
        //            \               \              \---局部地图跟踪失败---非OK（IMU时为RECENTLY_LOST）
        //             \               \---当前帧跟踪失败---非OK(地图超过10个关键帧时 RECENTLY_LOST)
        //              \---上一帧跟踪失败(RECENTLY_LOST)---重定位成功---局部地图跟踪成功---OK
        //               \                           \           \---局部地图跟踪失败---LOST
        //                \                           \---重定位失败---LOST（传不到这里，因为直接return了）
        //                 \--上一帧跟踪失败(LOST)--LOST（传不到这里，因为直接return了）

        // myStep 9 如果跟踪成功 或 最近刚刚跟丢，更新速度，清除无效地图点，按需创建关键帧
        if(bOK || mState==RECENTLY_LOST)
        {
            // Update motion model
            // Step 9.1 更新恒速运动模型 TrackWithMotionModel 中的mVelocity
            if(!mLastFrame.mTcw.empty() && !mCurrentFrame.mTcw.empty())
            {
                cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
                /// mVelocity = Tcl = Tcw * Twl,表示上一帧到当前帧的变换， 其中 Twl = LastTwc
                mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
                mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
                mVelocity = mCurrentFrame.mTcw*LastTwc;
            }
            else {
                // 否则没有速度
                mVelocity = cv::Mat();
            }
            if(mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO)
                mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

            // Clean VO matches
            // Step 9.2 清除观测不到的地图点
            for(int i=0; i<mCurrentFrame.N; i++)
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(pMP)
                    if(pMP->Observations()<1)
                    {
                        mCurrentFrame.mvbOutlier[i] = false;
                        mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                    }
            }
            //myline 清除地图中观测不到的线
            for(int i=0; i<mCurrentFrame.N_l; i++)
            {
                MapLine* pML = mCurrentFrame.mvpMapLines[i];
                if(pML)
                    if(pML->Observations()<1)
                    {
                        mCurrentFrame.mvbOutlier_Line[i] = false;
                        mCurrentFrame.mvpMapLines[i]=static_cast<MapLine*>(NULL);
                    }
            }

            // Delete temporal MapPoints
            // Step 9.3 清除恒速模型跟踪中 UpdateLastFrame中为当前帧临时添加的MapPoints（仅双目和rgbd）
            // 上个步骤中只是在当前帧中将这些MapPoints剔除，这里从MapPoints数据库中删除
            // 临时地图点仅仅是为了提高双目或rgbd摄像头的帧间跟踪效果，用完以后就扔了，没有添加到地图中
            for(list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend =  mlpTemporalPoints.end(); lit!=lend; lit++)
            {
                MapPoint* pMP = *lit;
                delete pMP;
            }
            mlpTemporalPoints.clear();

            // Delete temporal MapLines
            //myline 清除地图线
            for(list<MapLine*>::iterator lit = mlpTemporalLines.begin(), lend =  mlpTemporalLines.end(); lit!=lend; lit++)
            {
                MapLine* pML = *lit;
                delete pML;
            }
            // 这里不仅仅是清除mlpTemporalPoints，通过delete pMP还删除了指针指向的MapPoint
            // 不能够直接执行这个是因为其中存储的都是指针,之前的操作都是为了避免内存泄露
            mlpTemporalLines.clear();

#ifdef SAVE_TIMES
            std::chrono::steady_clock::time_point timeStartNewKF = std::chrono::steady_clock::now();
#endif
            // Check if we need to insert a new keyframe
//            if (NeedNewKeyFrameWithLines() == 1)
//                CreateNewKeyFrameWithLines(false);
//            else if (NeedNewKeyFrameWithLines() == 2)    // note [EAO] create keyframes by the new object.
//            {
//                CreateNewKeyFrameWithLines(true);
//            }
#ifdef SAVE_TIMES
            std::chrono::steady_clock::time_point timeEndNewKF = std::chrono::steady_clock::now();

            mTime_NewKF_Dec = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(timeEndNewKF - timeStartNewKF).count();
#endif

            // Step 9.4 根据条件来判断是否插入关键帧
            // 需要同时满足下面条件1和2
            // 条件1：bNeedKF=true，需要插入关键帧
            // 条件2：bOK=true跟踪成功 或 IMU模式下的RECENTLY_LOST模式且mInsertKFsLost为true

            // Check if we need to insert a new keyframe
            if((NeedNewKeyFrameWithLines() == 1) && (bOK|| (mState==RECENTLY_LOST && (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO))))
            {
                CreateNewKeyFrameWithLines(false);// 创建关键帧，对于双目或RGB-D会产生新的地图点
            }
            else if((NeedNewKeyFrameWithLines() == 2) && (bOK|| (mState==RECENTLY_LOST && (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO))))
            {
                CreateNewKeyFrameWithLines(true);
            }
            // We allow points and lines with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame. Only has effect if lastframe is tracked
            // 作者这里说允许在BA中被Huber核函数判断为外点的传入新的关键帧中，让后续的BA来审判他们是不是真正的外点
            // 但是估计下一帧位姿的时候我们不想用这些外点，所以删掉
            //  Step 9.5 删除那些在BA中检测为外点的地图点
            for(int i=0; i<mCurrentFrame.N;i++)
            {
                if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
            }
            //myline 删除那些在BA中检测为外点的地图线 imp第二次
            for(int i=0; i<mCurrentFrame.N_l;i++)
            {
                if(mCurrentFrame.mvpMapLines[i] && mCurrentFrame.mvbOutlier_Line[i])
                    mCurrentFrame.mvpMapLines[i]=static_cast<MapLine*>(NULL);
            }
        }

        // Reset if the camera get lost soon after initialization
        // myStep 10 如果第二阶段跟踪失败，跟踪状态为LOST
        if(mState==LOST)
        {
            // 如果地图中关键帧小于10，重置当前地图，退出当前跟踪
            if(pCurrentMap->KeyFramesInMap()<=5)
            {
                mpSystem->ResetActiveMap();
                return;
            }
            if ((mSensor == System::IMU_MONOCULAR) || (mSensor == System::IMU_STEREO))
                if (!pCurrentMap->isImuInitialized())
                {
                    // 如果是IMU模式并且还未进行IMU初始化，重置当前地图，退出当前跟踪
                    Verbose::PrintMess("Track lost before IMU initialisation, reseting...", Verbose::VERBOSITY_QUIET);
                    mpSystem->ResetActiveMap();
                    return;
                }
            // 如果地图中关键帧超过10 并且 纯视觉模式 或 虽然是IMU模式但是已经完成IMU初始化了，保存当前地图，创建新的地图
            CreateMapInAtlas();
        }
        // 确保已经设置了参考关键帧
        if(!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;
        // 保存上一帧的数据,当前帧变上一帧
        mLastFrame = Frame(mCurrentFrame);
    }
    // 查看到此为止
    // mState的历史变化---上一帧跟踪成功---当前帧跟踪成功---局部地图跟踪成功---OK
    //            \               \              \---局部地图跟踪失败---非OK（IMU时为RECENTLY_LOST）
    //             \               \---当前帧跟踪失败---非OK(地图超过10个关键帧时 RECENTLY_LOST)
    //              \---上一帧跟踪失败(RECENTLY_LOST)---重定位成功---局部地图跟踪成功---OK
    //               \                           \           \---局部地图跟踪失败---LOST
    //                \                           \---重定位失败---LOST（传不到这里，因为直接return了）
    //                 \--上一帧跟踪失败(LOST)--LOST（传不到这里，因为直接return了）
    // last.记录位姿信息，用于轨迹复现
    // Step 11 记录位姿信息，用于最后保存所有的轨迹
    if(mState==OK || mState==RECENTLY_LOST)
    {
        // Store frame pose information to retrieve the complete camera trajectory afterwards.
        // Step 11：记录位姿信息，用于最后保存所有的轨迹
        if(!mCurrentFrame.mTcw.empty())
        {
            // 计算相对姿态Tcr = Tcw * Twr, Twr = Trw^-1
            cv::Mat Tcr = mCurrentFrame.mTcw*mCurrentFrame.mpReferenceKF->GetPoseInverse();
            mlRelativeFramePoses.push_back(Tcr);
            mlpReferences.push_back(mCurrentFrame.mpReferenceKF);
            mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
            mlbLost.push_back(mState==LOST);
        }
        else
        {
            // This can happen if tracking is lost
            // 如果跟踪失败，则相对位姿使用上一次值
            mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
            mlpReferences.push_back(mlpReferences.back());
            mlFrameTimes.push_back(mlFrameTimes.back());
            mlbLost.push_back(mState==LOST);
        }

    }

}


void Tracking::StereoInitialization()
{
    if(mCurrentFrame.N>500)
    {
        if (mSensor == System::IMU_STEREO)
        {
            if (!mCurrentFrame.mpImuPreintegrated || !mLastFrame.mpImuPreintegrated)
            {
                cout << "not IMU meas" << endl;
                return;
            }

            if (cv::norm(mCurrentFrame.mpImuPreintegratedFrame->avgA-mLastFrame.mpImuPreintegratedFrame->avgA)<0.5)
            {
                cout << "not enough acceleration" << endl;
                return;
            }

            if(mpImuPreintegratedFromLastKF)
                delete mpImuPreintegratedFromLastKF;

            mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(),*mpImuCalib);
            mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;
        }

        // Set Frame pose to the origin (In case of inertial SLAM to imu)
        if (mSensor == System::IMU_STEREO)
        {
            cv::Mat Rwb0 = mCurrentFrame.mImuCalib.Tcb.rowRange(0,3).colRange(0,3).clone();
            cv::Mat twb0 = mCurrentFrame.mImuCalib.Tcb.rowRange(0,3).col(3).clone();
            mCurrentFrame.SetImuPoseVelocity(Rwb0, twb0, cv::Mat::zeros(3,1,CV_32F));
        }
        else
            mCurrentFrame.SetPose(cv::Mat::eye(4,4,CV_32F));

        // Create KeyFrame
        KeyFrame* pKFini = new KeyFrame(mCurrentFrame,mpAtlas->GetCurrentMap(),mpKeyFrameDB);
        pKFini->set_segmentation_mask(mCurrentFrame._img_seg_mask);
        pKFini->set_depth_map(mCurrentFrame._depth_img);
        // Insert KeyFrame in the map
        mpAtlas->AddKeyFrame(pKFini);

        // Create MapPoints and asscoiate to KeyFrame
        if(!mpCamera2){
            for(int i=0; i<mCurrentFrame.N;i++)
            {
                float z = mCurrentFrame.mvDepth[i];
                if(z>0)
                {
                    cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                    MapPoint* pNewMP = new MapPoint(x3D,pKFini,mpAtlas->GetCurrentMap());
                    pNewMP->AddObservation(pKFini,i);
                    pKFini->AddMapPoint(pNewMP,i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpAtlas->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i]=pNewMP;
                }
            }
        } else{
            for(int i = 0; i < mCurrentFrame.Nleft; i++){
                int rightIndex = mCurrentFrame.mvLeftToRightMatch[i];
                if(rightIndex != -1){
                    cv::Mat x3D = mCurrentFrame.mvStereo3Dpoints[i];

                    MapPoint* pNewMP = new MapPoint(x3D,pKFini,mpAtlas->GetCurrentMap());

                    pNewMP->AddObservation(pKFini,i);
                    pNewMP->AddObservation(pKFini,rightIndex + mCurrentFrame.Nleft);

                    pKFini->AddMapPoint(pNewMP,i);
                    pKFini->AddMapPoint(pNewMP,rightIndex + mCurrentFrame.Nleft);

                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpAtlas->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i]=pNewMP;
                    mCurrentFrame.mvpMapPoints[rightIndex + mCurrentFrame.Nleft]=pNewMP;
                }
            }
        }

        Verbose::PrintMess("New Map created with " + to_string(mpAtlas->MapPointsInMap()) + " points", Verbose::VERBOSITY_QUIET);

        mpLocalMapper->InsertKeyFrame(pKFini);

        mLastFrame = Frame(mCurrentFrame);
        mnLastKeyFrameId=mCurrentFrame.mnId;
        mpLastKeyFrame = pKFini;
        mnLastRelocFrameId = mCurrentFrame.mnId;

        mvpLocalKeyFrames.push_back(pKFini);
        mvpLocalMapPoints=mpAtlas->GetAllMapPoints();
        mpReferenceKF = pKFini;
        mCurrentFrame.mpReferenceKF = pKFini;

        mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

        mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.push_back(pKFini);

        mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

        mState=OK;
    }
}


/**
 * @brief 双目和rgbd的地图初始化，比单目简单很多
 *
 * 由于具有深度信息，直接生成MapPoints
 */

void Tracking::StereoInitializationWithLines()
{
    // 初始化要求当前帧的特征点超过500
    if(mCurrentFrame.N + mCurrentFrame.N_l > 500)
    {
        if (mSensor == System::IMU_STEREO)
        {
            if (!mCurrentFrame.mpImuPreintegrated || !mLastFrame.mpImuPreintegrated)
            {
                cout << "not IMU meas" << endl;
                return;
            }

            if (cv::norm(mCurrentFrame.mpImuPreintegratedFrame->avgA-mLastFrame.mpImuPreintegratedFrame->avgA)<0.5)
            {
                cout << "not enough acceleration" << endl;
                return;
            }

            if(mpImuPreintegratedFromLastKF)
                delete mpImuPreintegratedFromLastKF;

            mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(),*mpImuCalib);
            mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;
        }

        // Set Frame pose to the origin (In case of inertial SLAM to imu)
        // 设定初始位姿为单位旋转，0平移，imu模式下设置的是相机位姿
        if (mSensor == System::IMU_STEREO)
        {
            cv::Mat Rwb0 = mCurrentFrame.mImuCalib.Tcb.rowRange(0,3).colRange(0,3).clone();
            cv::Mat twb0 = mCurrentFrame.mImuCalib.Tcb.rowRange(0,3).col(3).clone();
            mCurrentFrame.SetImuPoseVelocity(Rwb0, twb0, cv::Mat::zeros(3,1,CV_32F));
        }
        else
            mCurrentFrame.SetPose(cv::Mat::eye(4,4,CV_32F));

        // Create KeyFrame
        // 将当前帧构造为初始关键帧
        KeyFrame* pKFini = new KeyFrame(mCurrentFrame,mpAtlas->GetCurrentMap(),mpKeyFrameDB);
        //myplan 传递mask图片
        pKFini->set_segmentation_mask(mCurrentFrame._img_seg_mask);
        cv::Mat test = pKFini->get_segmentation_mask();
        // Insert KeyFrame in the map
        // 在地图中添加该初始关键帧
        mpAtlas->AddKeyFrame(pKFini);
//        cout<<"Creatting MP & ML !!!"<<endl;



        // Create MapPoints and asscoiate to KeyFrame
        if(!mpCamera2){

            // 为每个特征点构造MapPoint
            for(int i=0; i<mCurrentFrame.N;i++)
            {
                // 只有具有正深度的点才会被构造地图点
                float z = mCurrentFrame.mvDepth[i];
                if(z>0)
                {

                    // 通过反投影得到该特征点的世界坐标系下3D坐标
                    cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                    MapPoint* pNewMP = new MapPoint(x3D,pKFini,mpAtlas->GetCurrentMap());
                    // 为该MapPoint添加属性：
                    // a.观测到该MapPoint的关键帧
                    // b.该MapPoint的描述子
                    // c.该MapPoint的平均观测方向和深度范围
                    pNewMP->AddObservation(pKFini,i);
                    pKFini->AddMapPoint(pNewMP,i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpAtlas->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i]=pNewMP;
                }
            }
        } else{
            for(int i = 0; i < mCurrentFrame.Nleft; i++){
                int rightIndex = mCurrentFrame.mvLeftToRightMatch[i];
                if(rightIndex != -1){
                    cv::Mat x3D = mCurrentFrame.mvStereo3Dpoints[i];

                    MapPoint* pNewMP = new MapPoint(x3D,pKFini,mpAtlas->GetCurrentMap());

                    pNewMP->AddObservation(pKFini,i);
                    pNewMP->AddObservation(pKFini,rightIndex + mCurrentFrame.Nleft);

                    pKFini->AddMapPoint(pNewMP,i);
                    pKFini->AddMapPoint(pNewMP,rightIndex + mCurrentFrame.Nleft);

                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpAtlas->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i]=pNewMP;
                    mCurrentFrame.mvpMapPoints[rightIndex + mCurrentFrame.Nleft]=pNewMP;
                }
            }
        }
        ///myline 为线添加地图线
        if(!mpCamera2)
        {
            // 为每个特征线构造MapPoint
            for(int i=0; i<mCurrentFrame.N_l; ++i)
            {
//                cout<<"Creatting ML !!!"<<endl;
                // 只有具有正深度的点才会被构造地图点
//                std::cout<<"点的深度值"<<mCurrentFrame.mvDepth_l[i].first<<mCurrentFrame.mvDepth_l[i].second<<std::endl;
                if(mCurrentFrame.mvDepth_l[i].first>0 && mCurrentFrame.mvDepth_l[i].second>0)
                {
                    Eigen::Vector3d sp3D = mCurrentFrame.UnprojectStereoLines(
                                mCurrentFrame.mvKeysUn_Line[i].startPointX,
                                mCurrentFrame.mvKeysUn_Line[i].startPointY,
                                mCurrentFrame.mvDepth_l[i].first);
                    Eigen::Vector3d ep3D = mCurrentFrame.UnprojectStereoLines(
                                mCurrentFrame.mvKeysUn_Line[i].endPointX,
                                mCurrentFrame.mvKeysUn_Line[i].endPointY,
                                mCurrentFrame.mvDepth_l[i].second);
//                   std::cout<<"before StereoInitializationWithLines第2个有参构造"<<std::endl;
                    MapLine* pNewML = new MapLine(sp3D,ep3D,pKFini,mpAtlas->GetCurrentMap());      //
                    pNewML->AddObservation(pKFini,i);                           //
                    pKFini->AddMapLine(pNewML,i);                              //
                    pNewML->ComputeDistinctiveDescriptors();
                    pNewML->UpdateNormalAndDepth();// imp rgbd 模式下这里可能有错
                    mpAtlas->AddMapLine(pNewML);

                    mCurrentFrame.mvpMapLines[i]=pNewML;
                }
            }
        }
        //myplan 加面,刚开始的时不知道是否有关键帧
        if(!mpCamera2)
        {
            //imp
            const auto planar_mapper = new Planar_Mapping_module(mpAtlas, mSensor);
            planar_mapper->initialize_map_with_plane(pKFini);

            delete planar_mapper;
        }
        //myplan 表示初始帧到地面坐标系的变换矩
//        cv::Mat InitToGround = mCurrentFrame.mGroundtruthPose_mat;
//
//        cv::Mat R = InitToGround.rowRange(0, 3).colRange(0, 3);//表示初始帧的旋转矩阵
//        cv::Mat t = InitToGround.rowRange(0, 3).col(3);//表示初始帧的平移向量
//        cv::Mat Rinv = R.t();
//        cv::Mat Ow = -Rinv * t;//是原点在地面坐标系下的坐标
//        cv::Mat GroundToInit = cv::Mat::eye(4, 4, CV_32F);
//        Rinv.copyTo(GroundToInit.rowRange(0, 3).colRange(0, 3));
//        Ow.copyTo(GroundToInit.rowRange(0, 3).col(3));
//
//        bool build_worldframe_on_ground = true;
//        //即需要在地面坐标系下构建世界坐标系
//        if (build_worldframe_on_ground) // transform initial pose and map to ground frame
//        {
//            pKFini->SetPose(pKFini->GetPose() * GroundToInit);
//
//            std::vector<MapPoint*> vpAllMapPoints;
//            vpAllMapPoints=mpAtlas->GetAllMapPoints();
//            for (size_t iMP = 0; iMP < vpAllMapPoints.size(); iMP++)
//            {
//                if (vpAllMapPoints[iMP])
//                {
//                    MapPoint *pMP = vpAllMapPoints[iMP];
//                    pMP->SetWorldPos(InitToGround.rowRange(0, 3).colRange(0, 3) * pMP->GetWorldPos() + InitToGround.rowRange(0, 3).col(3));
//                }
//            }
//        }
        //myplan end


        Verbose::PrintMess("New Map created with " + to_string(mpAtlas->MapPointsInMap()) + " points and "  + to_string(mpAtlas->MapLinesInMap()) + " lines", Verbose::VERBOSITY_QUIET);
        // 在局部地图中添加该初始关键帧
        mpLocalMapper->InsertKeyFrame(pKFini);
        // 更新当前帧为上一帧
        mLastFrame = Frame(mCurrentFrame);//myplan 这里相当于mInitialSecendFrame
        mnLastKeyFrameId=mCurrentFrame.mnId;
        mpLastKeyFrame = pKFini;
        mnLastRelocFrameId = mCurrentFrame.mnId;

        mvpLocalKeyFrames.push_back(pKFini);
        mvpLocalMapPoints=mpAtlas->GetAllMapPoints();
        //myline 添加地图线
        mvpLocalMapLines=mpAtlas->GetAllMapLines();
        mpReferenceKF = pKFini;
        mCurrentFrame.mpReferenceKF = pKFini;
        // 把当前（最新的）局部MapPoints作为ReferenceMapPoints
        mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);
        //myline 添加地图线
        mpAtlas->SetReferenceMapLines(mvpLocalMapLines);

        mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.push_back(pKFini);

        mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);
        // 追踪成功
        mState=OK;
    }
}
/**
 * @brief 单目的地图初始化
 *
 * 并行地计算基础矩阵和单应性矩阵，选取其中一个模型，恢复出最开始两帧之间的相对姿态以及点云
 * 得到初始两帧的匹配、相对运动、初始MapPoints
 *
 * Step 1：（未创建）得到用于初始化的第一帧，初始化需要两帧
 * Step 2：（已创建）如果当前帧特征点数大于100，则得到用于单目初始化的第二帧
 * Step 3：在mInitialFrame与mCurrentFrame中找匹配的特征点对
 * Step 4：如果初始化的两帧之间的匹配点太少，重新初始化
 * Step 5：通过H模型或F模型进行单目初始化，得到两帧间相对运动、初始MapPoints
 * Step 6：删除那些无法进行三角化的匹配点
 * Step 7：将三角化得到的3D点包装成MapPoints
 */

void Tracking::MonocularInitialization()
{

    if(!mpInitializer)
    {
        // Set Reference Frame
        if(mCurrentFrame.mvKeys.size()>100)
        {

            mInitialFrame = Frame(mCurrentFrame);
            mLastFrame = Frame(mCurrentFrame);
            mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
            for(size_t i=0; i<mCurrentFrame.mvKeysUn.size(); i++)
                mvbPrevMatched[i]=mCurrentFrame.mvKeysUn[i].pt;

            if(mpInitializer)
                delete mpInitializer;

            mpInitializer =  new Initializer(mCurrentFrame,1.0,200);

            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);

            if (mSensor == System::IMU_MONOCULAR)
            {
                if(mpImuPreintegratedFromLastKF)
                {
                    delete mpImuPreintegratedFromLastKF;
                }
                mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(),*mpImuCalib);
                mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;

            }
            return;
        }
    }
    else
    {
        if (((int)mCurrentFrame.mvKeys.size()<=100)||((mSensor == System::IMU_MONOCULAR)&&(mLastFrame.mTimeStamp-mInitialFrame.mTimeStamp>1.0)))
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);

            return;
        }

        // Find correspondences
        ORBmatcher matcher(0.9,true);
        int nmatches = matcher.SearchForInitialization(mInitialFrame,mCurrentFrame,mvbPrevMatched,mvIniMatches,100);

        // Check if there are enough correspondences
        if(nmatches<100)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
            return;
        }

        cv::Mat Rcw; // Current Camera Rotation
        cv::Mat tcw; // Current Camera Translation
        vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

        if(mpCamera->ReconstructWithTwoViews(mInitialFrame.mvKeysUn,mCurrentFrame.mvKeysUn,mvIniMatches,Rcw,tcw,mvIniP3D,vbTriangulated))
        {
            for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
            {
                if(mvIniMatches[i]>=0 && !vbTriangulated[i])
                {
                    mvIniMatches[i]=-1;
                    nmatches--;
                }
            }

            // Set Frame Poses
            mInitialFrame.SetPose(cv::Mat::eye(4,4,CV_32F));
            cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
            Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
            tcw.copyTo(Tcw.rowRange(0,3).col(3));
            mCurrentFrame.SetPose(Tcw);

            CreateInitialMapMonocular();

            // Just for video
            // bStepByStep = true;
        }
    }
}



void Tracking::CreateInitialMapMonocular()
{
    // Create KeyFrames
    KeyFrame* pKFini = new KeyFrame(mInitialFrame,mpAtlas->GetCurrentMap(),mpKeyFrameDB);
    KeyFrame* pKFcur = new KeyFrame(mCurrentFrame,mpAtlas->GetCurrentMap(),mpKeyFrameDB);

    if(mSensor == System::IMU_MONOCULAR)
        pKFini->mpImuPreintegrated = (IMU::Preintegrated*)(NULL);


    pKFini->ComputeBoW();
    pKFcur->ComputeBoW();

    // Insert KFs in the map
    mpAtlas->AddKeyFrame(pKFini);
    mpAtlas->AddKeyFrame(pKFcur);

    for(size_t i=0; i<mvIniMatches.size();i++)
    {
        if(mvIniMatches[i]<0)
            continue;

        //Create MapPoint.
        cv::Mat worldPos(mvIniP3D[i]);
        MapPoint* pMP = new MapPoint(worldPos,pKFcur,mpAtlas->GetCurrentMap());

        pKFini->AddMapPoint(pMP,i);
        pKFcur->AddMapPoint(pMP,mvIniMatches[i]);

        pMP->AddObservation(pKFini,i);
        pMP->AddObservation(pKFcur,mvIniMatches[i]);

        pMP->ComputeDistinctiveDescriptors();
        pMP->UpdateNormalAndDepth();

        //Fill Current Frame structure
        mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
        mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

        //Add to Map
        mpAtlas->AddMapPoint(pMP);
    }


    // Update Connections
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();

    std::set<MapPoint*> sMPs;
    sMPs = pKFini->GetMapPoints();

    // Bundle Adjustment
    Verbose::PrintMess("New Map created with " + to_string(mpAtlas->MapPointsInMap()) + " points", Verbose::VERBOSITY_QUIET);
    Optimizer::GlobalBundleAdjustemnt(mpAtlas->GetCurrentMap(),20);

    pKFcur->PrintPointDistribution();
    // Step 5 取场景的中值深度，用于尺度归一化
    // 为什么是 pKFini 而不是 pKCur ? 答：都可以的，内部做了位姿变换了
    float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    float invMedianDepth;
    if(mSensor == System::IMU_MONOCULAR)
        invMedianDepth = 4.0f/medianDepth; // 4.0f
    else
        invMedianDepth = 1.0f/medianDepth;

    if(medianDepth<0 || pKFcur->TrackedMapPoints(1)<50) // TODO Check, originally 100 tracks
    {
        Verbose::PrintMess("Wrong initialization, reseting...", Verbose::VERBOSITY_NORMAL);
        mpSystem->ResetActiveMap();
        return;
    }

    // Scale initial baseline
    cv::Mat Tc2w = pKFcur->GetPose();
    Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*invMedianDepth;
    pKFcur->SetPose(Tc2w);

    // Scale points
    vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
    for(size_t iMP=0; iMP<vpAllMapPoints.size(); iMP++)
    {
        if(vpAllMapPoints[iMP])
        {
            MapPoint* pMP = vpAllMapPoints[iMP];
            pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth);
            pMP->UpdateNormalAndDepth();
        }
    }

    if (mSensor == System::IMU_MONOCULAR)
    {
        pKFcur->mPrevKF = pKFini;
        pKFini->mNextKF = pKFcur;
        pKFcur->mpImuPreintegrated = mpImuPreintegratedFromLastKF;

        mpImuPreintegratedFromLastKF = new IMU::Preintegrated(pKFcur->mpImuPreintegrated->GetUpdatedBias(),pKFcur->mImuCalib);
    }


    mpLocalMapper->InsertKeyFrame(pKFini);
    mpLocalMapper->InsertKeyFrame(pKFcur);
    mpLocalMapper->mFirstTs=pKFcur->mTimeStamp;

    mCurrentFrame.SetPose(pKFcur->GetPose());
    mnLastKeyFrameId=mCurrentFrame.mnId;
    mpLastKeyFrame = pKFcur;
    mnLastRelocFrameId = mInitialFrame.mnId;

    mvpLocalKeyFrames.push_back(pKFcur);
    mvpLocalKeyFrames.push_back(pKFini);
    mvpLocalMapPoints=mpAtlas->GetAllMapPoints();
    mpReferenceKF = pKFcur;
    mCurrentFrame.mpReferenceKF = pKFcur;

    // Compute here initial velocity
    vector<KeyFrame*> vKFs = mpAtlas->GetAllKeyFrames();

    cv::Mat deltaT = vKFs.back()->GetPose()*vKFs.front()->GetPoseInverse();
    mVelocity = cv::Mat();
    Eigen::Vector3d phi = LogSO3(Converter::toMatrix3d(deltaT.rowRange(0,3).colRange(0,3)));


    double aux = (mCurrentFrame.mTimeStamp-mLastFrame.mTimeStamp)/(mCurrentFrame.mTimeStamp-mInitialFrame.mTimeStamp);
    phi *= aux;

    mLastFrame = Frame(mCurrentFrame);

    mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

    mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

    mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.push_back(pKFini);

    mState=OK;

    initID = pKFcur->mnId;
}


void Tracking::CreateMapInAtlas()
{
    mnLastInitFrameId = mCurrentFrame.mnId;
    mpAtlas->CreateNewMap();
    if (mSensor==System::IMU_STEREO || mSensor == System::IMU_MONOCULAR)
        mpAtlas->SetInertialSensor();
    mbSetInit=false;

    mnInitialFrameId = mCurrentFrame.mnId+1;
    mState = NO_IMAGES_YET;

    // Restart the variable with information about the last KF
    mVelocity = cv::Mat();
    mnLastRelocFrameId = mnLastInitFrameId; // The last relocation KF_id is the current id, because it is the new starting point for new map
    Verbose::PrintMess("First frame id in map: " + to_string(mnLastInitFrameId+1), Verbose::VERBOSITY_NORMAL);
    mbVO = false; // Init value for know if there are enough MapPoints in the last KF
    if(mSensor == System::MONOCULAR || mSensor == System::IMU_MONOCULAR)
    {
        if(mpInitializer)
            delete mpInitializer;
        mpInitializer = static_cast<Initializer*>(NULL);
    }

    if((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ) && mpImuPreintegratedFromLastKF)
    {
        delete mpImuPreintegratedFromLastKF;
        mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(),*mpImuCalib);
    }

    if(mpLastKeyFrame)
        mpLastKeyFrame = static_cast<KeyFrame*>(NULL);

    if(mpReferenceKF)
        mpReferenceKF = static_cast<KeyFrame*>(NULL);

    mLastFrame = Frame();
    mCurrentFrame = Frame();
    mvIniMatches.clear();

    mbCreatedMap = true;

}

void Tracking::CheckReplacedInLastFrame()
{
    for(int i =0; i<mLastFrame.N; i++)
    {
        MapPoint* pMP = mLastFrame.mvpMapPoints[i];

        if(pMP)
        {
            MapPoint* pRep = pMP->GetReplaced();
            if(pRep)
            {
                mLastFrame.mvpMapPoints[i] = pRep;
            }
        }
    }
}
//myline 替换含有线的帧
void Tracking::CheckReplacedInLastFrameWithLines()
{
    for(int i =0; i<mLastFrame.N; i++)
    {
        MapPoint* pMP = mLastFrame.mvpMapPoints[i];

        if(pMP)
        {
            MapPoint* pRep = pMP->GetReplaced();
            if(pRep)
            {
                mLastFrame.mvpMapPoints[i] = pRep;
            }
        }
    }
    for(int i =0; i<mLastFrame.N_l; i++)
    {
        MapLine* pML = mLastFrame.mvpMapLines[i];

        if(pML)
        {
            MapLine* pRep = pML->GetReplaced();
            if(pRep)
            {
                mLastFrame.mvpMapLines[i] = pRep;
            }
        }
    }
}


bool Tracking::TrackReferenceKeyFrame()
{
    // Compute Bag of Words vector
    mCurrentFrame.ComputeBoW();

    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.7,true);
    vector<MapPoint*> vpMapPointMatches;

    int nmatches = matcher.SearchByBoW(mpReferenceKF,mCurrentFrame,vpMapPointMatches);

    if(nmatches<15)
    {
        cout << "TRACK_REF_KF: Less than 15 matches!!\n";
        return false;
    }

    mCurrentFrame.mvpMapPoints = vpMapPointMatches;
    mCurrentFrame.SetPose(mLastFrame.mTcw);

    //mCurrentFrame.PrintPointDistribution();


    // cout << " TrackReferenceKeyFrame mLastFrame.mTcw:  " << mLastFrame.mTcw << endl;
    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        //if(i >= mCurrentFrame.Nleft) break;
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                if(i < mCurrentFrame.Nleft){
                    pMP->mbTrackInView = false;
                }
                else{
                    pMP->mbTrackInViewR = false;
                }
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }

    // TODO check these conditions
    if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO)
        return true;
    else
        return nmatchesMap>=10;
}

bool Tracking::TrackReferenceKeyFrameWithLines()
{

    // Compute Bag of Words vector
    // Step 1：将当前帧的描述子转化为BoW向量
    mCurrentFrame.ComputeBoW();

    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.7,true);
    vector<MapPoint*> vpMapPointMatches;
    // Step 2：通过词袋BoW加速当前帧与参考帧之间的特征点匹配
    mCurrentFrame.n_inliers_pt = matcher.SearchByBoW(mpReferenceKF,mCurrentFrame,vpMapPointMatches);
    mCurrentFrame.mvpMapPoints = vpMapPointMatches;
    mCurrentFrame.SetPose(mLastFrame.mTcw);// 用上一次的Tcw设置初值，在PoseOptim

    // Line Matching
    fill(mCurrentFrame.mvpMapLines.begin(),mCurrentFrame.mvpMapLines.end(),static_cast<MapLine*>(NULL));
    std::vector<int> matches_12;
    float minRatio12L = 0.9;
    LineMatcher::match(mpReferenceKF->mDescriptors_l, mCurrentFrame.mDescriptors_Line, minRatio12L, matches_12);

    const vector<MapLine*> vpMapLinesRKF = mpReferenceKF->GetMapLineMatches();
    const double deltaAngle = M_PI/8.0;
    const double deltaWidth = (mCurrentFrame.mnMaxX-mCurrentFrame.mnMinX)*0.1;
    const double deltaHeight = (mCurrentFrame.mnMaxY-mCurrentFrame.mnMinY)*0.1;
    //初始化线的数量
    mCurrentFrame.n_inliers_ls = 0;
    const int nmatches_12 = matches_12.size();
    for (int i1 = 0; i1 < nmatches_12; ++i1)
    {
        const int i2 = matches_12[i1];
        if (i2 < 0) continue;

        if(mCurrentFrame.mvpMapLines[i2])
            if(mCurrentFrame.mvpMapLines[i2]->Observations()>0)
                continue;



        mCurrentFrame.mvpMapLines[i2] = vpMapLinesRKF[i1];
        ++mCurrentFrame.n_inliers_ls;
    }

    mCurrentFrame.n_inliers = mCurrentFrame.n_inliers_ls + mCurrentFrame.n_inliers_pt;

    if(mCurrentFrame.n_inliers<15)
    {
        cout << "TRACK_REF_KF: Less than 15 matches!!\n";
        return false;
    }
    // Step 4:通过优化3D-2D的重投影误差来获得位姿
    if(SLAM==0)
//        if(mSensor==System::STEREO) {
        Optimizer::PoseOptimizationPL(&mCurrentFrame);
//        }
//        else if(mSensor==System::RGBD){
//            Optimizer::PoseOptimizationWithLine(&mCurrentFrame);
//        }
    else if(SLAM==1)
        Optimizer::PoseOptimizationOnlyLine(&mCurrentFrame);
    else if(SLAM==2)
        Optimizer::PoseOptimizationOnlyLineAngles(&mCurrentFrame);
    else if(SLAM==3)
        Optimizer::PoseOptimizationOnlyLineWithAngles(&mCurrentFrame);

    // Discard outliers
    // Step 5：剔除优化后的匹配点中的外点
    //之所以在优化之后才剔除外点，是因为在优化的过程中就有了对这些外点的标记
    int nmatchesMap_p = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                // 清除它在当前帧中存在过的痕迹
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                //myline 参数   imp 第二次修改
                mCurrentFrame.n_inliers_pt--;
                mCurrentFrame.n_inliers--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap_p++;
        }
    }
    //myline 剔除优化后的匹配线中的外点
    int nmatchesMap_l = 0;
    for(int i =0; i<mCurrentFrame.N_l; i++)
    {
        if(mCurrentFrame.mvpMapLines[i])
        {
            if(mCurrentFrame.mvbOutlier_Line[i])
            {
                MapLine* pML = mCurrentFrame.mvpMapLines[i];

                mCurrentFrame.mvpMapLines[i]=static_cast<MapLine*>(NULL);
                mCurrentFrame.mvbOutlier_Line[i]=false;
                pML->mbTrackInView = false;
                pML->mnLastFrameSeen = mCurrentFrame.mnId;
                mCurrentFrame.n_inliers_ls--;
                mCurrentFrame.n_inliers--;
            }
            else if(mCurrentFrame.mvpMapLines[i]->Observations()>0)
                nmatchesMap_l++;
        }
    }

    // TODO check these conditions
    if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO)
        return true;
    else
        std::cout<<"参考关键帧跟踪的点线匹配数"<< nmatchesMap_p + nmatchesMap_l<<std::endl;
        return nmatchesMap_p + nmatchesMap_l>=10;
} 

void Tracking::UpdateLastFrame()
{
    // Update pose according to reference keyframe
    KeyFrame* pRef = mLastFrame.mpReferenceKF;
    cv::Mat Tlr = mlRelativeFramePoses.back();
    mLastFrame.SetPose(Tlr*pRef->GetPose());

    if(mnLastKeyFrameId==mLastFrame.mnId || mSensor==System::MONOCULAR || mSensor==System::IMU_MONOCULAR || !mbOnlyTracking)
        return;

    // Create "visual odometry" MapPoints
    // We sort points according to their measured depth by the stereo/RGB-D sensor
    vector<pair<float,int> > vDepthIdx;
    vDepthIdx.reserve(mLastFrame.N);
    for(int i=0; i<mLastFrame.N;i++)
    {
        float z = mLastFrame.mvDepth[i];
        if(z>0)
        {
            vDepthIdx.push_back(make_pair(z,i));
        }
    }

    if(vDepthIdx.empty())
        return;

    sort(vDepthIdx.begin(),vDepthIdx.end());

    // We insert all close points (depth<mThDepth)
    // If less than 100 close points, we insert the 100 closest ones.
    int nPoints = 0;
    for(size_t j=0; j<vDepthIdx.size();j++)
    {
        int i = vDepthIdx[j].second;

        bool bCreateNew = false;

        MapPoint* pMP = mLastFrame.mvpMapPoints[i];
        if(!pMP)
            bCreateNew = true;
        else if(pMP->Observations()<1)
        {
            bCreateNew = true;
        }

        if(bCreateNew)
        {
            cv::Mat x3D = mLastFrame.UnprojectStereo(i);
            MapPoint* pNewMP = new MapPoint(x3D,mpAtlas->GetCurrentMap(),&mLastFrame,i);

            mLastFrame.mvpMapPoints[i]=pNewMP;

            mlpTemporalPoints.push_back(pNewMP);
            nPoints++;
        }
        else
        {
            nPoints++;
        }

        if(vDepthIdx[j].first>mThDepth && nPoints>100)
        {
            break;
        }
    }
}
    //MYLINE 函数重写
void Tracking::UpdateLastFrameWithLines()
{
    // Update pose according to reference keyframe
    KeyFrame* pRef = mLastFrame.mpReferenceKF;
    cv::Mat Tlr = mlRelativeFramePoses.back();
    mLastFrame.SetPose(Tlr*pRef->GetPose());

    //imp 修改动态线点颜色，动态线为蓝色，动态点也为蓝色 正常点为绿 正常的为绿色 注释掉下面的话
    if(mnLastKeyFrameId==mLastFrame.mnId || mSensor==System::MONOCULAR || mSensor==System::IMU_MONOCULAR || !mbOnlyTracking)
        return;

    // Create "visual odometry" MapPoints
    // We sort points according to their measured depth by the stereo/RGB-D sensor
    vector<pair<float,int> > vDepthIdx;
    vDepthIdx.reserve(mLastFrame.N);
    for(int i=0; i<mLastFrame.N;i++)
    {
        float z = mLastFrame.mvDepth[i];
        if(z>0)
        {
            vDepthIdx.push_back(make_pair(z,i));
        }
    }

    if(vDepthIdx.empty())
        return;
    // 按照深度从小到大排序
    sort(vDepthIdx.begin(),vDepthIdx.end());

    // We insert all close points (depth<mThDepth)
    // If less than 100 close points, we insert the 100 closest ones.
    // Step 2.2：从中找出不是地图点的部分
    int nPoints = 0;
    for(size_t j=0; j<vDepthIdx.size();j++)
    {
        int i = vDepthIdx[j].second;

        bool bCreateNew = false;
        // 如果这个点对应在上一帧中的地图点没有,或者创建后就没有被观测到,那么就生成一个临时的地图点
        MapPoint* pMP = mLastFrame.mvpMapPoints[i];
        if(!pMP)
            bCreateNew = true;
        else if(pMP->Observations()<1)
        {
            bCreateNew = true;
        }

        if(bCreateNew)
        {
            // Step 2.3：需要创建的点，包装为地图点。只是为了提高双目和RGBD的跟踪成功率，
            // 反投影到世界坐标系中
            cv::Mat x3D = mLastFrame.UnprojectStereo(i);
            // 加入上一帧的地图点中
            MapPoint* pNewMP = new MapPoint(x3D,mpAtlas->GetCurrentMap(),&mLastFrame,i);

            mLastFrame.mvpMapPoints[i]=pNewMP;
            // 标记为临时添加的MapPoint，之后在CreateNewKeyFrame之前会全部删除
            //myline imp
            mlpTemporalPoints.push_back(pNewMP);
            nPoints++;
        }
        else
        {
            nPoints++;
        }

        if(vDepthIdx[j].first>mThDepth && nPoints>100)
        {
            break;
        }
    }
    //myline 将线更新
    vDepthIdx.clear();
    vDepthIdx.reserve(mLastFrame.N_l);
    for(int i=0; i<mLastFrame.N_l;i++)
    {
        float sz = mLastFrame.mbf/mLastFrame.mvDisparity_l[i].first;
        float ez = mLastFrame.mbf/mLastFrame.mvDisparity_l[i].second;
        if(sz<0||ez<0) continue;
        float z = sz>ez?sz:ez;
        vDepthIdx.push_back(make_pair(z,i));
    }

    if(vDepthIdx.empty())
        return;

    sort(vDepthIdx.begin(),vDepthIdx.end());

    // We insert all close lines (endpoint's depth<mThDepth)
    // If less than 50 close lines, we insert the 50 closest ones.
    int nLines = 0;
    for(size_t j=0; j<vDepthIdx.size();j++)
    {
        int i = vDepthIdx[j].second;

        bool bCreateNew = false;

        MapLine* pML = mLastFrame.mvpMapLines[i];
        if(!pML)
            bCreateNew = true;
        else if(pML->Observations()<1)
        {
            bCreateNew = true;
        }

        if(bCreateNew)
        {
            Eigen::Vector3d sp3D = mLastFrame.UnprojectStereoLines(
                        mLastFrame.mvKeysUn_Line[i].startPointX,
                        mLastFrame.mvKeysUn_Line[i].startPointY,
                        mLastFrame.mvDepth_l[i].first);
            Eigen::Vector3d ep3D = mLastFrame.UnprojectStereoLines(
                        mLastFrame.mvKeysUn_Line[i].endPointX,
                        mLastFrame.mvKeysUn_Line[i].endPointY,
                        mLastFrame.mvDepth_l[i].second);
            std::cout<<"brefor 第3个mapline的有参构造 "<<std::endl;
            MapLine* pNewML = new MapLine(sp3D,ep3D,mpAtlas->GetCurrentMap(),&mLastFrame,i);

            mLastFrame.mvpMapLines[i]=pNewML;
            mlpTemporalLines.push_back(pNewML);
            nLines++;
        }
        else
        {
            nLines++;
        }

        if(vDepthIdx[j].first>mThDepth && nLines>50)
            break;
    }
}

bool Tracking::TrackWithMotionModel()
{
    ORBmatcher matcher(0.9,true);

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points if in Localization Mode
    UpdateLastFrame();



    if (mpAtlas->isImuInitialized() && (mCurrentFrame.mnId>mnLastRelocFrameId+mnFramesToResetIMU))
    {
        // Predict ste with IMU if it is initialized and it doesnt need reset
        PredictStateIMU();
        return true;
    }
    else
    {
        mCurrentFrame.SetPose(mVelocity*mLastFrame.mTcw);
    }


    fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

    // Project points seen in previous frame
    int th;

    if(mSensor==System::STEREO)
        th=7;
    else
        th=15;

    int nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,th,mSensor==System::MONOCULAR || mSensor==System::IMU_MONOCULAR);

    // If few matches, uses a wider window search
    if(nmatches<20)
    {
        Verbose::PrintMess("Not enough matches, wider window search!!", Verbose::VERBOSITY_NORMAL);
        fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

        nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,2*th,mSensor==System::MONOCULAR || mSensor==System::IMU_MONOCULAR);
        Verbose::PrintMess("Matches with wider search: " + to_string(nmatches), Verbose::VERBOSITY_NORMAL);

    }

    if(nmatches<20)
    {
        Verbose::PrintMess("Not enough matches!!", Verbose::VERBOSITY_NORMAL);
        if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO)
            return true;
        else
            return false;
    }

    // Optimize frame pose with all matches
    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                if(i < mCurrentFrame.Nleft){
                    pMP->mbTrackInView = false;
                }
                else{
                    pMP->mbTrackInViewR = false;
                }
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }

    if(mbOnlyTracking)
    {
        mbVO = nmatchesMap<10;
        return nmatches>20;
    }

    if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO)
        return true;
    else
        return nmatchesMap>=10;
}
/**
 * @brief 根据恒定速度模型用上一帧地图点来对当前帧进行跟踪
 * Step 1：更新上一帧的位姿；对于双目或RGB-D相机，还会根据深度值生成临时地图点
 * Step 2：根据上一帧特征点对应地图点进行投影匹配
 * Step 3：优化当前帧位姿
 * Step 4：剔除地图点中外点
 * @return 如果匹配数大于10，认为跟踪成功，返回true
 */
bool Tracking::TrackWithMotionModelWithLines(const double &timestamp)
{
    // 最小距离 < 0.9*次小距离 匹配成功，检查旋转
    ORBmatcher matcher(0.9,true);

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points if in Localization Mode
    UpdateLastFrameWithLines();


    if (mpAtlas->isImuInitialized() && (mCurrentFrame.mnId>mnLastRelocFrameId+mnFramesToResetIMU))
    {
        // Predict ste with IMU if it is initialized and it doesnt need reset
        // IMU完成初始化 并且 距离重定位挺久不需要重置IMU，用IMU来估计位姿，没有后面的这那那这的
        PredictStateIMU();
        return true;
    }
    else
    {
        // 根据之前估计的速度，用恒速模型得到当前帧的初始位姿。
        mCurrentFrame.SetPose(mVelocity*mLastFrame.mTcw);
    }


    vector<vector<int>> _mat;
    mCurrentFrame.have_detected = false;
    mCurrentFrame.finish_detected = false;
    //TODO 改到这了
    // Yolo

    //第一个数字：类别标签或类别索引。表示检测到的物体属于哪个类别。
    //39 瓶子黄色，56椅子，73书本
    // 例如，73可能代表"car"类别，56可能代表"person"类别。这需要参考特定的物体类别编码或标签映射表才能确定具体的类别名称。
    //后续数字：边界框的坐标和大小。通常包括矩形框的左上角x坐标、左上角y坐标、宽度、高度。
    // 例如，对于第一个检测结果（73 173 138 46 70 0.578918），它表示一个边界框左上角位于(173, 138)，宽度为46，高度为70。
    //最后一个数字：置信度或准确度。表示物体检测器对于该检测结果的置信度或准确度。该值通常在0到1之间，越接近1表示置信度越高。
    ifstream infile("/home/lzh/VINS/walk_xyz/" + to_string(timestamp) + ".txt", ios::in);
    if (!infile.is_open())
    {
        cout << "yolo_detection file open fail" << endl;
//        exit(233);
    } else{
//        cout << "yolo_detection  open !!!!!!!!!!" << endl;
    }
    vector<int> row; //一个行向量，用于保存一行数据的整数。
    int tmp;//是一个临时变量，用于从字符串中提取整数
    string line;//是一个字符串变量，用于保存从文件中读取的每一行数据
    //这个循环将逐行读取文件infile的内容
    //将每一行的数据存储在line变量中
    while (getline(infile, line)) {
        // string to int.
        //读取的每一行数据存储在line中，并将其作为参数创建一个istringstream对象istr。
        istringstream istr(line);
        //将从istr中提取整数，并将提取的整数存储在tmp变量中。循环会一直执行，直到无法继续提取整数
        while (istr >> tmp) {
            //将提取的整数tmp添加到row向量中，相当于将每一行的整数逐个添加到row中
            row.push_back(tmp);
        }
        //将填充好的row向量添加到二维整数向量_mat中，表示添加了一行数据。
        _mat.push_back(row); // vector<vector<int>>.里面一层是row对应txt几行，_mat元素就有多少个
        row.clear();
        istr.clear();
        line.clear();
    }
    infile.close();//关闭输入文件流infile

    //  vector<vector<int>> --> std::vector<BoxSE>.
    std::vector<BoxSE> boxes_offline;
    for (auto &mat_row: _mat) {
        BoxSE box;
        box.m_class = mat_row[0];//txt文件第一个是类别
        box.m_score = mat_row[5];//第六个是置信度
        box.x = mat_row[1];//左上角x
        box.y = mat_row[2];//左上角y
        box.width = mat_row[3];//第四个宽度
        box.height = mat_row[4];//第五个类别框的高
//        std::cout<<"box+class+sorce===="<< box.m_class<< "+"<<box.m_score<<std::endl;
        // box.m_class_name = "";
        boxes_offline.push_back(box);//将txt文件中的信息存在boxes_offline容器中
//        std::cout<<"boxes===="<<boxes_offline<<std::endl;
    }
    //使用 std::sort 函数对 boxes_offline 向量中的 BoxSE 对象进行排序，按照 m_score 的降序排列
    std::sort(boxes_offline.begin(), boxes_offline.end(), [](BoxSE a, BoxSE b) -> bool {
        return a.m_score > b.m_score;
    });
    // save to current frame.
    //保存这个box到成员变量
    mCurrentFrame.boxes = boxes_offline;
//    std::cout<<"mCurrentFrame.boxes1111=="<<mCurrentFrame.boxes<<std::endl;
//    std::cout << "Size of boxes1111===: " << mCurrentFrame.boxes.size() << std::endl;

    // std::vector<BoxSE> --> Eigen::MatrixXd.
    int i = 0;//用来定义追踪box索引
    Eigen::MatrixXd eigenMat;
    eigenMat.resize((int) mCurrentFrame.boxes.size(), 5);//行数为boxes大小，列为5
    //对每个box将box的信息存为Eigen矩阵
    for (auto &box: mCurrentFrame.boxes) {
        // std::cout << box.m_class << " " << box.x << " " << box.y << " "
        //           << box.width << " " << box.height << " " << box.m_score << std::endl;
        /**
        * keyboard  66 199 257 193 51
        * mouse     64 377 320 31 39
        * cup       41 442 293 51 63
        * tvmonitor 62 232 93 156 141
        * remote    65 44 260 38 57
        */
        eigenMat(i, 0) = box.x;
        eigenMat(i, 1) = box.y;
        eigenMat(i, 2) = box.width;
        eigenMat(i, 3) = box.height;
        eigenMat(i, 4) = box.m_score;
        i++;
    }
    // save to current frame.
    //保存到当前帧的成员变量
    mCurrentFrame.boxes_eigen = eigenMat;
//    std::cout<<"mCurrentFrame.boxes_eigen==="<<mCurrentFrame.boxes_eigen<<std::endl;

    // there are objects in current frame?
    if (!mCurrentFrame.boxes.empty()) {
        //已经检测到物体设为true
        mCurrentFrame.have_detected = true;
//        std::cout<<"检测到了"<<std::endl;
    }


    // ******************************
    //      STEP 0. Cube SLAM       *
    // ******************************
    bool bCubeslam = false;
    if((mCurrentFrame.mnId > 10) && bCubeslam)
    {
        Eigen::Matrix3d calib;
        calib << 535.4,  0,  320.1,
                0,  539.2, 247.6,
                0,      0,     1;

        detect_3d_cuboid detect_cuboid_obj;
        detect_cuboid_obj.whether_plot_detail_images = false;
        detect_cuboid_obj.whether_plot_final_images = false;
        detect_cuboid_obj.print_details = false;
        detect_cuboid_obj.set_calibration(calib);
        detect_cuboid_obj.whether_sample_bbox_height = false;
        detect_cuboid_obj.whether_sample_cam_roll_pitch = false;
        detect_cuboid_obj.nominal_skew_ratio = 2;
        detect_cuboid_obj.whether_save_final_images = true;

        std::vector<ObjectSet> frames_cuboids;

        detect_cuboid_obj.detect_cuboid(mCurrentFrame.mColorImage,
                                        mCurrentFrame.mGroundtruthPose_eigen,
                                        mCurrentFrame.boxes_eigen,
                                        mCurrentFrame.all_lines_eigen,
                                        frames_cuboids);

        cv::imwrite("/home/lzh/VINS/SPL_SLAM/Exp_Picture/Cube_Pic/" + to_string(mCurrentFrame.mnId) + ".png", detect_cuboid_obj.cuboids_2d_img);
    }
    // Cube SLAM END ---------------------------------------------------------


    // 清空当前帧的地图点
    fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

    // *****************************
    // STEP 1. construct 2D object *
    // *****************************
    vector<Object_2D *> objs_2d;
    cv::Mat image = mCurrentFrame.mColorImage.clone();
    //TODO 这里的box没数据
//    std::cout << "Size of boxes2222===: " << mCurrentFrame.boxes.size() << std::endl;
//    std::cout<<"mCurrentFrame.boxes2222=="<<mCurrentFrame.boxes<<std::endl;
    for (auto &box : mCurrentFrame.boxes)
    {
        Object_2D *obj = new Object_2D;

        // copy object bounding box and initialize the 3D center.
        obj->CopyBoxes(box);
        obj->sum_pos_3d = cv::Mat::zeros(3, 1, CV_32F);

        objs_2d.push_back(obj);
    }
    // construct 2D object END ---------------

    // Project points seen in previous frame
    int th;

    if(mSensor==System::STEREO)
        th=7;
    else
        th=15;
    // Step 3：用上一帧地图点进行投影匹配，如果匹配点不够，则扩大搜索半径再来一次
    mCurrentFrame.n_inliers_pt = matcher.SearchByProjection(mCurrentFrame,mLastFrame,th,mSensor==System::MONOCULAR || mSensor==System::IMU_MONOCULAR);

    // If few matches, uses a wider window search
    if(mCurrentFrame.n_inliers_pt<20)
    {
        Verbose::PrintMess("Not enough matches, wider window search!!", Verbose::VERBOSITY_NORMAL);
        fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

        mCurrentFrame.n_inliers_pt = matcher.SearchByProjection(mCurrentFrame,mLastFrame,2*th,mSensor==System::MONOCULAR || mSensor==System::IMU_MONOCULAR);
        Verbose::PrintMess("Matches with wider search: " + to_string(mCurrentFrame.n_inliers_pt), Verbose::VERBOSITY_NORMAL);

    }

    // Frame to Frame Line Matching

    // Line Matching Searching by projection (TO DO: Improve)
    /*float th_l = 3.0;
    float ang_th = M_PI/8.0;
    fill(mCurrentFrame.mvpMapLines.begin(),mCurrentFrame.mvpMapLines.end(),static_cast<MapLine*>(NULL));
    mCurrentFrame.n_inliers_ls = LineMatcher::SearchByProjection(mCurrentFrame, mLastFrame, mCurrentFrame.grid_Line, th_l, ang_th);

    // If few matches, uses a wider window search
    if(mCurrentFrame.n_inliers_ls<10)
    {
        Verbose::PrintMess("Not enough Line matches, wider window search!!", Verbose::VERBOSITY_NORMAL);
        fill(mCurrentFrame.mvpMapLines.begin(),mCurrentFrame.mvpMapLines.end(),static_cast<MapLine*>(NULL));

        mCurrentFrame.n_inliers_ls = LineMatcher::SearchByProjection(mCurrentFrame, mLastFrame, mCurrentFrame.grid_Line, th_l+2, ang_th);
        Verbose::PrintMess("Matches with wider search: " + to_string(mCurrentFrame.n_inliers_ls), Verbose::VERBOSITY_NORMAL);
    } */

    std::vector<int> matches_12;
    float minRatio12L = 0.9;
    LineMatcher::match(mLastFrame.mDescriptors_Line, mCurrentFrame.mDescriptors_Line, minRatio12L, matches_12);

    const double deltaAngle = M_PI/8.0;
    const double deltaWidth = (mCurrentFrame.mnMaxX-mCurrentFrame.mnMinX)*0.1;
    const double deltaHeight = (mCurrentFrame.mnMaxY-mCurrentFrame.mnMinY)*0.1;
    mCurrentFrame.n_inliers_ls = 0;
    const int nmatches_12 = matches_12.size();

    for (int i1 = 0; i1 < nmatches_12; ++i1) 
    {
        if(!mLastFrame.mvpMapLines[i1]) continue;
        const int i2 = matches_12[i1];
        if (i2 < 0) continue;

        if(mCurrentFrame.mvpMapLines[i2])
            if(mCurrentFrame.mvpMapLines[i2]->Observations()>0)
                continue;

        // check for orientation and position in image

        // Project In Image
        const cv::Mat Rcw = mCurrentFrame.mTcw.rowRange(0,3).colRange(0,3);
        const cv::Mat tcw = mCurrentFrame.mTcw.rowRange(0,3).col(3);

        Eigen::Vector3d sp = mLastFrame.mvpMapLines[i1]->GetWorldPos().head(3);
        Eigen::Vector3d ep = mLastFrame.mvpMapLines[i1]->GetWorldPos().tail(3);
        cv::Mat x3Dw_sp = Converter::toCvMat(sp);
        cv::Mat x3Dw_ep = Converter::toCvMat(ep);
        cv::Mat x3Dc_sp = Rcw*x3Dw_sp+tcw;
        cv::Mat x3Dc_ep = Rcw*x3Dw_ep+tcw;

        const float invzc_sp = 1.0/x3Dc_sp.at<float>(2);
        const float invzc_ep = 1.0/x3Dc_ep.at<float>(2);

        if(invzc_sp<0 || invzc_ep<0)
            continue;

        cv::Point2f uv_sp = mCurrentFrame.mpCamera->project(x3Dc_sp);
        cv::Point2f uv_ep = mCurrentFrame.mpCamera->project(x3Dc_ep);

        if(uv_sp.x<mCurrentFrame.mnMinX || uv_sp.x>mCurrentFrame.mnMaxX || uv_ep.x<mCurrentFrame.mnMinX || uv_ep.x>mCurrentFrame.mnMaxX)
            continue;
        if(uv_sp.y<mCurrentFrame.mnMinY || uv_sp.y>mCurrentFrame.mnMaxY || uv_ep.y<mCurrentFrame.mnMinY || uv_ep.y>mCurrentFrame.mnMaxY)
            continue;

        // check for orientation
        double theta = mCurrentFrame.mvKeysUn_Line[i2].angle-atan2(uv_ep.y - uv_sp.y, uv_ep.x - uv_sp.x);;
        if(theta<-M_PI) theta+=2*M_PI;
        else if(theta>M_PI) theta-=2*M_PI;
        if(fabs(theta)>deltaAngle) 
        {
            matches_12[i1] = -1;
            continue;
        }
        // check for position in image
        const float sX_curr = mCurrentFrame.mvKeysUn_Line[i2].startPointX;
        const float sX_last = uv_sp.x;
        const float sY_curr = mCurrentFrame.mvKeysUn_Line[i2].startPointY;
        const float sY_last = uv_sp.y;
        const float eX_curr = mCurrentFrame.mvKeysUn_Line[i2].endPointX;
        const float eX_last = uv_ep.x;
        const float eY_curr = mCurrentFrame.mvKeysUn_Line[i2].endPointY;
        const float eY_last = uv_ep.y;
        if(fabs(sX_curr-sX_last)>deltaWidth || fabs(eX_curr-eX_last)>deltaWidth || fabs(sY_curr-sY_last)>deltaHeight || fabs(eY_curr-eY_last)>deltaHeight )
        {
            matches_12[i1] = -1;
            continue;
        }
        
        mCurrentFrame.mvpMapLines[i2] = mLastFrame.mvpMapLines[i1]; 
        ++mCurrentFrame.n_inliers_ls;
    } 

    mCurrentFrame.n_inliers = mCurrentFrame.n_inliers_ls + mCurrentFrame.n_inliers_pt;

    if(mCurrentFrame.n_inliers<20)
    {
        Verbose::PrintMess("Not enough matches!!", Verbose::VERBOSITY_NORMAL);
        if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO)
            return true;
        else
            return false;
    }
//myplan  start ***************************************************************
    // ***************************************
    // STEP 2. associate objects with points *
    // ***************************************
    AssociateObjAndPoints(objs_2d);

    // ***************************************
    // STEP 3. associate objects with lines *
    // ***************************************
    AssociateObjAndLines(objs_2d);//mynote 注释掉

    // ***************************************************
    // STEP 4. compute the mean and standard of points.*
    // Erase outliers (camera frame) by boxplot.*
    // **************************************************
    for (auto &obj : objs_2d)
    {
        // compute the mean and standard.
        obj->ComputeMeanAndStandardFrame();

        // If the object has too few points, ignore.
        if (obj->Obj_c_MapPonits.size() < 8)
            continue;

        // Erase outliers by boxplot.
        obj->RemoveOutliersByBoxPlot(mCurrentFrame);
    }
    // Erase outliers of obj_2d END ----------------------

    // **************************************************************************
    // STEP 5. construct the bounding box by object feature points in the image.*
    // **************************************************************************
    // bounding box detected by yolo |  bounding box constructed by object points.
    //  _______________                 //  _____________
    // |   *        *  |                // |   *        *|
    // |    *  *       |                // |    *  *     |
    // |*      *  *    |                // |*      *  *  |
    // | *   *    *    |                // | *   *    *  |
    // |   *       *   |                // |___*_______*_|
    // |_______________|
    const cv::Mat Rcw = mCurrentFrame.mTcw.rowRange(0, 3).colRange(0, 3);
    const cv::Mat tcw = mCurrentFrame.mTcw.rowRange(0, 3).col(3);
    for (auto &obj : objs_2d)
    {
        // object 3D center (world).
        obj->_Pos = obj->sum_pos_3d / obj->Obj_c_MapPonits.size();
        obj->mCountMappoint = obj->Obj_c_MapPonits.size(); // point number.
        // world -> camera.
        cv::Mat x3Dc = Rcw * obj->_Pos + tcw;
        const float xc = x3Dc.at<float>(0);
        const float yc = x3Dc.at<float>(1);
        const float invzc = 1.0 / x3Dc.at<float>(2);
        // camera -> image.
        float u = mCurrentFrame.fx * xc * invzc + mCurrentFrame.cx;
        float v = mCurrentFrame.fy * yc * invzc + mCurrentFrame.cy;
        obj->point_center_2d = cv::Point2f(u, v);    // 3D center project to image. no use in this opensource version.

        // record the coordinates of each point in the xy(uv) directions.
        vector<float> x_pt;
        vector<float> y_pt;
        for (auto &pMP : obj->Obj_c_MapPonits)
        {
            float u = pMP->feature.pt.x;
            float v = pMP->feature.pt.y;

            x_pt.push_back(u);
            y_pt.push_back(v);
        }

        if (x_pt.size() < 4) // ignore.
            continue;

        // extremum in xy(uv) direction
        sort(x_pt.begin(), x_pt.end());
        sort(y_pt.begin(), y_pt.end());
        float x_min = x_pt[0];
        float x_max = x_pt[x_pt.size() - 1];
        float y_min = y_pt[0];
        float y_max = y_pt[y_pt.size() - 1];

        // make insure in the image.
        if (x_min < 0)
            x_min = 0;
        if (y_min < 0)
            y_min = 0;
        if (x_max > image.cols)
            x_max = image.cols;
        if (y_max > image.rows)
            y_max = image.rows;

        // the bounding box constructed by object feature points.
        obj->mRectFeaturePoints = cv::Rect(x_min, y_min, x_max - x_min, y_max - y_min);
    }
    // construct bounding box by feature points END -----------------------------------

    // **********************************************************************************************
    // STEP 6. remove 2d bad bounding boxes.
    // Due to the complex scene and Yolo error detection, some poor quality objects need to be removed.
    // The strategy can be adjusted and is not unique, such as:
    // 1. objects overlap with too many object;
    // 2. objects with too few points;
    // 3. objects with too few points and on the edge of the image;
    // 4. objects too large and take up more than half of the image;
    // TODO and so on ......
    // **********************************************************************************************
    // overlap with too many objects.
    for (size_t f = 0; f < objs_2d.size(); ++f)
    {
        int num = 0;
        for (size_t l = 0; l < objs_2d.size(); ++l)
        {
            if (f == l)
                continue;

            if (Converter::bboxOverlapratioLatter(objs_2d[f]->mBoxRect, objs_2d[l]->mBoxRect) > 0.05)
                num++;
        }
        // overlap with more than 3 objects.
        if (num > 4)
            objs_2d[f]->bad = true;
    }
    for (size_t f = 0; f < objs_2d.size(); ++f)
    {
        if (objs_2d[f]->bad)
            continue;

        // ignore the error detect by yolo.
        if ((objs_2d[f]->_class_id == 0) || (objs_2d[f]->_class_id == 63) || (objs_2d[f]->_class_id == 15))
            objs_2d[f]->bad = true;

        // too large in the image.
        if ((float)objs_2d[f]->mBoxRect.area() / (float)(image.cols * image.rows) > 0.5)
            objs_2d[f]->bad = true;

        // too few object points.
        if (objs_2d[f]->Obj_c_MapPonits.size() < 5)
            objs_2d[f]->bad = true;

            // object points too few and the object on the edge of the image.
        else if ((objs_2d[f]->Obj_c_MapPonits.size() >= 5) && (objs_2d[f]->Obj_c_MapPonits.size() < 10))
        {
            if ((objs_2d[f]->mBox.x < 20) || (objs_2d[f]->mBox.y < 20) ||
                (objs_2d[f]->mBox.x + objs_2d[f]->mBox.width > image.cols - 20) ||
                (objs_2d[f]->mBox.y + objs_2d[f]->mBox.height > image.rows - 20))
            {
                objs_2d[f]->bad = true;
            }
        }

        // mark the object that on the edge of the image.
        if (((objs_2d[f]->mBox.x < 5) || (objs_2d[f]->mBox.y < 5) ||
             (objs_2d[f]->mBox.x + objs_2d[f]->mBox.width > image.cols - 5) ||
             (objs_2d[f]->mBox.y + objs_2d[f]->mBox.height > image.rows - 5)))
        {
            objs_2d[f]->bOnEdge = true;
        }

        // when the overlap is large, only one object remains.
        for (size_t l = 0; l < objs_2d.size(); ++l)
        {
            if (objs_2d[l]->bad)
                continue;

            if (f == l)
                continue;

            // retain objects which with high probability.
            if (Converter::bboxOverlapratio(objs_2d[f]->mBoxRect, objs_2d[l]->mBoxRect) > 0.3)
            {
                if (objs_2d[f]->mScore < objs_2d[l]->mScore)
                    objs_2d[f]->bad = true;
                else if (objs_2d[f]->mScore >= objs_2d[l]->mScore)
                    objs_2d[l]->bad = true;
            }
            // if one object surrounds another, keep the larger one.
            if (Converter::bboxOverlapratio(objs_2d[f]->mBoxRect, objs_2d[l]->mBoxRect) > 0.05)
            {
                if (Converter::bboxOverlapratioFormer(objs_2d[f]->mBoxRect, objs_2d[l]->mBoxRect) > 0.85)
                    objs_2d[f]->bad = true;
                if (Converter::bboxOverlapratioLatter(objs_2d[f]->mBoxRect, objs_2d[l]->mBoxRect) > 0.85)
                    objs_2d[l]->bad = true;
            }
        }
    }
    // erase the bad object.
    vector<Object_2D *>::iterator it;
    for (it = objs_2d.begin(); it != objs_2d.end(); )
    {
        if ((*it)->bad == true)
            it = objs_2d.erase(it); // erase.
        else
        {
            // if ((*it)->Obj_c_MapPonits.size() >= 5)
            // {
            //     cv::rectangle(image,
            //                     (*it)->mBoxRect,
            //                     cv::Scalar(100, 100, 256),
            //                     2);
            // }

            // cv::putText(image, to_string((*it)->_class_id),
            //             (*it)->box_center_2d,
            //             cv::FONT_HERSHEY_SIMPLEX, 0.5,
            //             cv::Scalar(0, 255, 255), 2);

            // std::string imname_rect = "./box/" + to_string(mCurrentFrame.mTimeStamp) + ".jpg";
            // cv::imwrite(imname_rect, image);

            ++it;
        }
    }
    // remove 2d bad bounding boxes END ------------------------------------------------------

    // *************************************************************
    // STEP 7. copy objects in the last frame after initialization.*
    // *************************************************************
    if ((mbObjectIni == true) && (mCurrentFrame.mnId > mnObjectIniFrameID))
    {
        // copy objects in the last frame.
        mCurrentFrame.mvLastObjectFrame = mLastFrame.mvObjectFrame;

        // copy objects in the penultimate frame.
        if (!mLastFrame.mvLastObjectFrame.empty())
            mCurrentFrame.mvLastLastObjectFrame = mLastFrame.mvLastObjectFrame;
    }
    // copy objects END -------------------------------------------------------

    // *******************************************************************************
    // STEP 8. Merges objects with 5-10 points  between two adjacent frames.
    // Advantage: Small objects with too few points, can be merged to keep them from being eliminated.
    // (The effect is not very significant.)
    // *******************************************************************************
    bool bMergeTwoObj = true;
    if ((!mCurrentFrame.mvLastObjectFrame.empty()) && bMergeTwoObj)
    {
        // object in current frame.
        for (size_t k = 0; k < objs_2d.size(); ++k)
        {
            // ignore objects with more than 10 points.
            if (objs_2d[k]->Obj_c_MapPonits.size() >= 10)
                continue;

            // object in last frame.
            for (size_t l = 0; l < mCurrentFrame.mvLastObjectFrame.size(); ++l)
            {
                // ignore objects with more than 10 points.
                if (mCurrentFrame.mvLastObjectFrame[l]->Obj_c_MapPonits.size() >= 10)
                    continue;

                // merge two objects.
                if (Converter::bboxOverlapratio(objs_2d[k]->mBoxRect, mCurrentFrame.mvLastObjectFrame[l]->mBoxRect) > 0.5)
                {
                    objs_2d[k]->MergeTwoFrameObj(mCurrentFrame.mvLastObjectFrame[l]);
                    break;
                }
            }
        }
    }
    // merge objects in two frame END -----------------------------------------------

    // ************************************
    // STEP 9. Initialize the object map  *
    // ************************************
    if ((mCurrentFrame.mnId > mLastFrame.mnId) && mbObjectIni == false)
        InitObjMap(objs_2d);

    // **************************************************************
    // STEP 10. Data association after initializing the object map. *
    // **************************************************************
    if ((mCurrentFrame.mnId > mnObjectIniFrameID) && (mbObjectIni == true))
    {
        // step 10.1 points of the object that appeared in the last 30 frames
        // are projected into the image to form a projection bounding box.
        for (int i = 0; i < (int)mpAtlas->mvObjectMap.size(); i++)
        {
            if (mpAtlas->mvObjectMap[i]->bBadErase)
                continue;

            // object appeared in the last 30 frames.
            if (mpAtlas->mvObjectMap[i]->mnLastAddID > mCurrentFrame.mnId - 30)
                mpAtlas->mvObjectMap[i]->ComputeProjectRectFrame(image, mCurrentFrame);
            else
            {
                mpAtlas->mvObjectMap[i]->mRectProject = cv::Rect(0, 0, 0, 0);
            }
        }

        // step 10.2 data association.
        for (size_t k = 0; k < objs_2d.size(); ++k)
        {
            // ignore object with less than 5 points.
            if (objs_2d[k]->Obj_c_MapPonits.size() < 5)
            {
                objs_2d[k]->few_mappoint = true;
                objs_2d[k]->current = false;
                continue;
            }

            // note: data association.
            objs_2d[k]->ObjectDataAssociation(mpAtlas, mCurrentFrame, image, mflag);
        }

        // step 10.3 remove objects with too few observations.
        for (int i = (int)mpAtlas->mvObjectMap.size() - 1; i >= 0; i--)
        {
            if(mflag == "NA")
                continue;

            if (mpAtlas->mvObjectMap[i]->bBadErase)
                continue;

            int df = (int)mpAtlas->mvObjectMap[i]->mObjectFrame.size();
            if (df < 10)
            {
                // not been observed in the last 30 frames.
                if (mpAtlas->mvObjectMap[i]->mnLastAddID < (mCurrentFrame.mnId - 30))
                {
                    if (df < 5)
                        mpAtlas->mvObjectMap[i]->bBadErase = true;

                        // if not overlap with other objects, don't remove.
                    else
                    {
                        bool overlap = false;
                        for (int j = (int)mpAtlas->mvObjectMap.size() - 1; j >= 0; j--)
                        {
                            if (mpAtlas->mvObjectMap[j]->bBadErase || (i == j))
                                continue;

                            if (mpAtlas->mvObjectMap[i]->WhetherOverlap(mpAtlas->mvObjectMap[j]))
                            {
                                overlap = true;
                                break;
                            }
                        }
                        if (overlap)
                            mpAtlas->mvObjectMap[i]->bBadErase = true;
                    }
                }
            }
        }

        // step 10.4 Update the co-view relationship between objects. (appears in the same frame).
        for (int i = (int)mpAtlas->mvObjectMap.size() - 1; i >= 0; i--)
        {
            if (mpAtlas->mvObjectMap[i]->mnLastAddID == mCurrentFrame.mnId)
            {
                for (int j = (int)mpAtlas->mvObjectMap.size() - 1; j >= 0; j--)
                {
                    if (i == j)
                        continue;

                    if (mpAtlas->mvObjectMap[j]->mnLastAddID == mCurrentFrame.mnId)
                    {
                        int nObjId = mpAtlas->mvObjectMap[j]->mnId;

                        map<int, int>::iterator sit;
                        sit = mpAtlas->mvObjectMap[i]->mmAppearSametime.find(nObjId);

                        if (sit != mpAtlas->mvObjectMap[i]->mmAppearSametime.end())
                        {
                            int sit_sec = sit->second;
                            mpAtlas->mvObjectMap[i]->mmAppearSametime.erase(nObjId);
                            mpAtlas->mvObjectMap[i]->mmAppearSametime.insert(make_pair(nObjId, sit_sec + 1));
                        }
                        else
                            mpAtlas->mvObjectMap[i]->mmAppearSametime.insert(make_pair(nObjId, 1));   // first co-view.
                    }
                }
            }
        }

        // step 10.5 Merge potential associate objects (see mapping thread).

        // step 10.6 Estimate the orientation of objects.
        for (int i = (int)mpAtlas->mvObjectMap.size() - 1; i >= 0; i--)
        {
            // map object.
            Object_Map* objMap = mpAtlas->mvObjectMap[i];

            if (objMap->bBadErase)
                continue;

            if (objMap->mnLastAddID < mCurrentFrame.mnId - 5)
                continue;

            // estimate only regular objects.
            if (((objMap->mnClass == 73) || (objMap->mnClass == 64) || (objMap->mnClass == 65)
                 || (objMap->mnClass == 66) || (objMap->mnClass == 56)))
            {
                // objects appear in current frame.
                if(objMap->mnLastAddID == mCurrentFrame.mnId)
                {
                    //mynote 求物体的旋转矩阵
                    SampleObjYaw(objMap);   // note: sample object yaw.
                }
            }

            // step 10.7 project quadrics to the image (only for visualization).
            cv::Mat axe = cv::Mat::zeros(3, 1, CV_32F);
            axe.at<float>(0) = mpAtlas->mvObjectMap[i]->mCuboid3D.lenth / 2;
            axe.at<float>(1) = mpAtlas->mvObjectMap[i]->mCuboid3D.width / 2;
            axe.at<float>(2) = mpAtlas->mvObjectMap[i]->mCuboid3D.height / 2;

            // object pose (world).
            cv::Mat Twq = Converter::toCvMat(mpAtlas->mvObjectMap[i]->mCuboid3D.pose);

            // Projection Matrix K[R|t].
            cv::Mat P(3, 4, CV_32F);
            Rcw.copyTo(P.rowRange(0, 3).colRange(0, 3));
            tcw.copyTo(P.rowRange(0, 3).col(3));
            P = mCurrentFrame.mK * P;

            // draw.
            image = DrawQuadricProject( this->mCurrentFrame.mQuadricImage,
                                        P,
                                        axe,
                                        Twq,
                                        mpAtlas->mvObjectMap[i]->mnClass);
        }
    } // myplan data association END ----------------------------------------------------------------




// Step 4：利用3D-2D投影关系，优化当前帧位姿
    // Optimize frame pose with all matches
    if(SLAM==0)
        Optimizer::PoseOptimizationPL(&mCurrentFrame);
    else if(SLAM==1)
        Optimizer::PoseOptimizationOnlyLine(&mCurrentFrame);
    else if(SLAM==2)
        Optimizer::PoseOptimizationOnlyLineAngles(&mCurrentFrame);
    else if(SLAM==3)
        Optimizer::PoseOptimizationOnlyLineWithAngles(&mCurrentFrame);

    // Discard outliers
        // Step 5：剔除地图点中外点
    int nmatchesMap_p = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                // 如果优化后判断某个地图点是外点，清除它的所有关系
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                if(i < mCurrentFrame.Nleft){
                    pMP->mbTrackInView = false;
                }
                else{
                    pMP->mbTrackInViewR = false;
                }
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                mCurrentFrame.n_inliers_pt--;
                mCurrentFrame.n_inliers--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap_p++;
        }
    }
        //myline 剔除优化后的匹配线中的外点
    int nmatchesMap_l = 0;
    for(int i =0; i<mCurrentFrame.N_l; i++)
    {
        if(mCurrentFrame.mvpMapLines[i])
        {
            if(mCurrentFrame.mvbOutlier_Line[i])
            {
                MapLine* pML = mCurrentFrame.mvpMapLines[i];

                mCurrentFrame.mvpMapLines[i]=static_cast<MapLine*>(NULL);
                mCurrentFrame.mvbOutlier_Line[i]=false;
                pML->mbTrackInView = false;
                pML->mnLastFrameSeen = mCurrentFrame.mnId;
                mCurrentFrame.n_inliers_ls--;
                mCurrentFrame.n_inliers--;
            }
            else if(mCurrentFrame.mvpMapLines[i]->Observations()>0)
                nmatchesMap_l++;
        }
    }
        // 纯定位模式下：如果成功追踪的地图点非常少,那么这里的mbVO标志就会置位
    if(mbOnlyTracking)
    {
        mbVO = nmatchesMap_p<10;
        return mCurrentFrame.n_inliers_pt>20;
    }

    if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO)
        return true;
    else
        return nmatchesMap_p + nmatchesMap_l>=10;
}

bool Tracking::TrackLocalMap()
{

    // We have an estimation of the camera pose and some map points tracked in the frame.
    // We retrieve the local map and try to find matches to points in the local map.
    mTrackedFr++;

    UpdateLocalMap();
    SearchLocalPoints();

    // TOO check outliers before PO
    int aux1 = 0, aux2=0;
    for(int i=0; i<mCurrentFrame.N; i++)
        if( mCurrentFrame.mvpMapPoints[i])
        {
            aux1++;
            if(mCurrentFrame.mvbOutlier[i])
                aux2++;
        }

    int inliers;
    if (!mpAtlas->isImuInitialized())
        Optimizer::PoseOptimization(&mCurrentFrame);
    else
    {
        if(mCurrentFrame.mnId<=mnLastRelocFrameId+mnFramesToResetIMU)
        {
            Verbose::PrintMess("TLM: PoseOptimization ", Verbose::VERBOSITY_DEBUG);
            Optimizer::PoseOptimization(&mCurrentFrame);
        }
        else
        {
            // if(!mbMapUpdated && mState == OK) //  && (mnMatchesInliers>30))
            if(!mbMapUpdated) //  && (mnMatchesInliers>30))
            {
                Verbose::PrintMess("TLM: PoseInertialOptimizationLastFrame ", Verbose::VERBOSITY_DEBUG);
                inliers = Optimizer::PoseInertialOptimizationLastFrame(&mCurrentFrame); // , !mpLastKeyFrame->GetMap()->GetIniertialBA1());
            }
            else
            {
                Verbose::PrintMess("TLM: PoseInertialOptimizationLastKeyFrame ", Verbose::VERBOSITY_DEBUG);
                inliers = Optimizer::PoseInertialOptimizationLastKeyFrame(&mCurrentFrame); // , !mpLastKeyFrame->GetMap()->GetIniertialBA1());
            }
        }
    }

    aux1 = 0, aux2 = 0;
    for(int i=0; i<mCurrentFrame.N; i++)
        if( mCurrentFrame.mvpMapPoints[i])
        {
            aux1++;
            if(mCurrentFrame.mvbOutlier[i])
                aux2++;
        }

    mnMatchesInliers = 0;

    // Update MapPoints Statistics
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(!mCurrentFrame.mvbOutlier[i])
            {
                mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                if(!mbOnlyTracking)
                {
                    if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                        mnMatchesInliers++;
                }
                else
                    mnMatchesInliers++;
            }
            else if(mSensor==System::STEREO)
                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
        }
    }

    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    mpLocalMapper->mnMatchesInliers=mnMatchesInliers;
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<50)
        return false;

    if((mnMatchesInliers>10)&&(mState==RECENTLY_LOST))
        return true;


    if (mSensor == System::IMU_MONOCULAR)
    {
        if(mnMatchesInliers<15)
        {
            return false;
        }
        else
            return true;
    }
    else if (mSensor == System::IMU_STEREO)
    {
        if(mnMatchesInliers<15)
        {
            return false;
        }
        else
            return true;
    }
    else
    {
        if(mnMatchesInliers<30)
            return false;
        else
            return true;
    }
}

bool Tracking::TrackLocalMapWithLines()
{
    // We have an estimation of the camera pose and some map points tracked in the frame.
    // We retrieve the local map and try to find matches to points in the local map.
    mTrackedFr++;

    // Step 1：更新局部关键帧 mvpLocalKeyFrames 和局部地图点 mvpLocalMapPoints
    UpdateLocalMapWithLines();
    //图中新增的在视野范围内的地图点，投影到当前帧搜索匹配，得到更多的匹配关系
    SearchLocalPointsAndLines();

    // TOO check outliers before PO
    int aux1 = 0, aux2=0;
    for(int i=0; i<mCurrentFrame.N; i++)
        if( mCurrentFrame.mvpMapPoints[i])
        {
            aux1++;
            if(mCurrentFrame.mvbOutlier[i])
                aux2++;
        }

    int inliers;
    if (!mpAtlas->isImuInitialized())
    {
        if(SLAM==0)
            Optimizer::PoseOptimizationPL(&mCurrentFrame);
        else if(SLAM==1)
            Optimizer::PoseOptimizationOnlyLine(&mCurrentFrame);
        else if(SLAM==2)
            Optimizer::PoseOptimizationOnlyLineAngles(&mCurrentFrame);
        else if(SLAM==3)
            Optimizer::PoseOptimizationOnlyLineWithAngles(&mCurrentFrame);
    }
    else
    {
        if(mCurrentFrame.mnId<=mnLastRelocFrameId+mnFramesToResetIMU)
        {
            Verbose::PrintMess("TLM: PoseOptimization ", Verbose::VERBOSITY_DEBUG);
            Optimizer::PoseOptimization(&mCurrentFrame);

        }
        else
        {
            // if(!mbMapUpdated && mState == OK) //  && (mnMatchesInliers>30))
            if(!mbMapUpdated) //  && (mnMatchesInliers>30))
            {
                Verbose::PrintMess("TLM: PoseInertialOptimizationLastFrame ", Verbose::VERBOSITY_DEBUG);
                inliers = Optimizer::PoseInertialOptimizationLastFrame(&mCurrentFrame); // , !mpLastKeyFrame->GetMap()->GetIniertialBA1());
            }
            else
            {
                Verbose::PrintMess("TLM: PoseInertialOptimizationLastKeyFrame ", Verbose::VERBOSITY_DEBUG);
                inliers = Optimizer::PoseInertialOptimizationLastKeyFrame(&mCurrentFrame); // , !mpLastKeyFrame->GetMap()->GetIniertialBA1());
            }
        }
    }

    aux1 = 0, aux2 = 0;
    for(int i=0; i<mCurrentFrame.N; i++)
        if( mCurrentFrame.mvpMapPoints[i])
        {
            aux1++;
            if(mCurrentFrame.mvbOutlier[i])
                aux2++;
        }

    mnMatchesInliers = 0;

    // Update MapPoints Statistics
    // Step 4：更新当前帧的地图点被观测程度，并统计跟踪局部地图后匹配数目
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(!mCurrentFrame.mvbOutlier[i])
            {
                mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                if(!mbOnlyTracking)
                {
                    if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                        mnMatchesInliers++;
                }
                else
                    mnMatchesInliers++;
            }
            else if(mSensor==System::STEREO)
                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
        }
    }
    //MYLINE 加入线
    mnMatchesInliers_l = 0;
    for(int i=0; i<mCurrentFrame.N_l; i++)
    {
        if(mCurrentFrame.mvpMapLines[i])
        {
            if(!mCurrentFrame.mvbOutlier_Line[i])
            {
                mCurrentFrame.mvpMapLines[i]->IncreaseFound();
                if(!mbOnlyTracking)
                {
                    if(mCurrentFrame.mvpMapLines[i]->Observations()>0)
                        mnMatchesInliers_l++;
                }
                else
                    mnMatchesInliers_l++;
            }
            else if(mSensor==System::STEREO)
                mCurrentFrame.mvpMapLines[i] = static_cast<MapLine*>(NULL);

        }
    }
    //点+线的匹配数量
    mnMatchesInliers = mnMatchesInliers + mnMatchesInliers_l;

    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    mpLocalMapper->mnMatchesInliers=mnMatchesInliers;
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<50)
        return false;

    if((mnMatchesInliers>10)&&(mState==RECENTLY_LOST))
        return true;


    if (mSensor == System::IMU_MONOCULAR)
    {
        if(mnMatchesInliers<15)
        {
            return false;
        }
        else
            return true;
    }
    else if (mSensor == System::IMU_STEREO)
    {
        if(mnMatchesInliers<15)
        {
            return false;
        }
        else
            return true;
    }
    else
    {
        if(mnMatchesInliers<30)
            return false;
        else
            return true;
    }
}


bool Tracking::NeedNewKeyFrame()
{
    if(((mSensor == System::IMU_MONOCULAR) || (mSensor == System::IMU_STEREO)) && !mpAtlas->GetCurrentMap()->isImuInitialized())
    {
        if (mSensor == System::IMU_MONOCULAR && (mCurrentFrame.mTimeStamp-mpLastKeyFrame->mTimeStamp)>=0.25)
            return true;
        else if (mSensor == System::IMU_STEREO && (mCurrentFrame.mTimeStamp-mpLastKeyFrame->mTimeStamp)>=0.25)
            return true;
        else
            return false;
    }

    if(mbOnlyTracking)
        return false;

    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
    {
        return false;
    }

    // Return false if IMU is initialazing
    if (mpLocalMapper->IsInitializing())
        return false;
    const int nKFs = mpAtlas->KeyFramesInMap();

    // Do not insert keyframes if not enough frames have passed from last relocalisation
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && nKFs>mMaxFrames)
    {
        return false;
    }

    // Tracked MapPoints in the reference keyframe
    int nMinObs = 3;
    if(nKFs<=2)
        nMinObs=2;
    int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

    // Local Mapping accept keyframes?
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    // Check how many "close" points are being tracked and how many could be potentially created.
    int nNonTrackedClose = 0;
    int nTrackedClose= 0;

    if(mSensor!=System::MONOCULAR && mSensor!=System::IMU_MONOCULAR)
    {
        int N = (mCurrentFrame.Nleft == -1) ? mCurrentFrame.N : mCurrentFrame.Nleft;
        for(int i =0; i<N; i++)
        {
            if(mCurrentFrame.mvDepth[i]>0 && mCurrentFrame.mvDepth[i]<mThDepth)
            {
                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                    nTrackedClose++;
                else
                    nNonTrackedClose++;

            }
        }
    }

    bool bNeedToInsertClose;
    bNeedToInsertClose = (nTrackedClose<100) && (nNonTrackedClose>70);

    // Thresholds
    float thRefRatio = 0.75f;
    if(nKFs<2)
        thRefRatio = 0.4f;

    if(mSensor==System::MONOCULAR)
        thRefRatio = 0.9f;

    if(mpCamera2) thRefRatio = 0.75f;

    if(mSensor==System::IMU_MONOCULAR)
    {
        if(mnMatchesInliers>350) // Points tracked from the local map
            thRefRatio = 0.75f;
        else
            thRefRatio = 0.90f;
    }

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b = ((mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames) && bLocalMappingIdle);
    //Condition 1c: tracking is weak
    const bool c1c = mSensor!=System::MONOCULAR && mSensor!=System::IMU_MONOCULAR && mSensor!=System::IMU_STEREO && (mnMatchesInliers<nRefMatches*0.25 || bNeedToInsertClose) ;
    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    const bool c2 = (((mnMatchesInliers<nRefMatches*thRefRatio || bNeedToInsertClose)) && mnMatchesInliers>15);

    // Temporal condition for Inertial cases
    bool c3 = false;
    if(mpLastKeyFrame)
    {
        if (mSensor==System::IMU_MONOCULAR)
        {
            if ((mCurrentFrame.mTimeStamp-mpLastKeyFrame->mTimeStamp)>=0.5)
                c3 = true;
        }
        else if (mSensor==System::IMU_STEREO)
        {
            if ((mCurrentFrame.mTimeStamp-mpLastKeyFrame->mTimeStamp)>=0.5)
                c3 = true;
        }
    }

    bool c4 = false;
    if ((((mnMatchesInliers<75) && (mnMatchesInliers>15)) || mState==RECENTLY_LOST) && ((mSensor == System::IMU_MONOCULAR))) // MODIFICATION_2, originally ((((mnMatchesInliers<75) && (mnMatchesInliers>15)) || mState==RECENTLY_LOST) && ((mSensor == System::IMU_MONOCULAR)))
        c4=true;
    else
        c4=false;

    if(((c1a||c1b||c1c) && c2)||c3 ||c4)
    {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        if(bLocalMappingIdle)
        {
            return true;
        }
        else
        {
            mpLocalMapper->InterruptBA();
            if(mSensor!=System::MONOCULAR  && mSensor!=System::IMU_MONOCULAR)
            {
                if(mpLocalMapper->KeyframesInQueue()<3)
                    return true;
                else
                    return false;
            }
            else
                return false;
        }
    }
    else
        return false;
}

//myplan 这里新增物体添加关键帧我觉得没什么用
int Tracking::NeedNewKeyFrameWithLines()
{
    if(((mSensor == System::IMU_MONOCULAR) || (mSensor == System::IMU_STEREO)) && !mpAtlas->GetCurrentMap()->isImuInitialized())
    {
        if (mSensor == System::IMU_MONOCULAR && (mCurrentFrame.mTimeStamp-mpLastKeyFrame->mTimeStamp)>=0.25)
            return true;
        else if (mSensor == System::IMU_STEREO && (mCurrentFrame.mTimeStamp-mpLastKeyFrame->mTimeStamp)>=0.25)
            return true;
        else
            return false;
    }
    // Step 1：纯VO模式下不插入关键帧
    if(mbOnlyTracking)
        return false;

    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    // Step 2：如果局部地图线程被闭环检测使用，则不插入关键帧
    if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
    {
        return false;
    }

    // Return false if IMU is initialazing

    if (mpLocalMapper->IsInitializing())
        return false;
    // 获取当前地图中的关键帧数目
    const int nKFs = mpAtlas->KeyFramesInMap();

    // Do not insert keyframes if not enough frames have passed from last relocalisation
    //  Step 3：如果距离上一次重定位比较近，并且关键帧数目超出最大限制，不插入关键帧
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && nKFs>mMaxFrames)
    {
        return false;
    }

    // Tracked MapPoints and MapLines in the reference keyframe
    int nMinObs = 3;
    if(nKFs<=2)
        nMinObs=2;
    int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs) + mpReferenceKF->TrackedMapLines(nMinObs);

    // Local Mapping accept keyframes?
    // Step 5：查询局部地图线程是否繁忙，当前能否接受新的关键帧
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    // Check how many "close" points are being tracked and how many could be potentially created.
    // Step 6：对于双目或RGBD摄像头，统计成功跟踪的近点的数量，如果跟踪到的近点太少，没有跟踪到的近点较多，可以插入关键帧
    int nTrackedClose= 0; // 双目或RGB-D中成功跟踪的近点（三维点）
    int nNonTrackedClose = 0;// 双目或RGB-D中没有跟踪到的近点

    //MYPLAN 这里进行了修改
    int nMap = 0;
    int nTotal = 0;

    if(mSensor!=System::MONOCULAR && mSensor!=System::IMU_MONOCULAR)
    {
        int N = (mCurrentFrame.Nleft == -1) ? mCurrentFrame.N : mCurrentFrame.Nleft;
        for(int i =0; i<N; i++)
        {
            if(mCurrentFrame.mvDepth[i]>0 && mCurrentFrame.mvDepth[i]<mThDepth)
            {
                nTotal++;
                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                {
                    nTrackedClose++;
                    nMap++;
                }
                else
                    nNonTrackedClose++;

            }
        }
    }
    else
    {
        // There are no visual odometry matches in the monocular case
        nMap = 1;
        nTotal = 1;
    }
    const float ratioMap = (float)nMap / fmax(1.0f, nTotal);

    // Check how many "close" Lines are being tracked and how many could be potentially created.
    int nNonTrackedClose_l = 0;
    int nTrackedClose_l = 0;

    for(int i=0; i<mCurrentFrame.N_l; i++) 
    {
        float sz = mCurrentFrame.mvDepth_l[i].first;
        float ez = mCurrentFrame.mvDepth_l[i].second;
        float z = sz>ez?sz:ez;
        if(sz>0 && ez>0 && z<mThDepth)
        {
            if(mCurrentFrame.mvpMapLines[i] && !mCurrentFrame.mvbOutlier_Line[i])
                nTrackedClose_l++;
            else
                nNonTrackedClose_l++;

        }
    }
    // 双目或RGBD情况下：跟踪到的地图点中近点太少 同时 没有跟踪到的三维点太多，可以插入关键帧了
    // 单目时，为false
    bool bNeedToInsertClose;
    if(SLAM==0)
        bNeedToInsertClose = (nTrackedClose + nTrackedClose_l<140) && (nNonTrackedClose + nNonTrackedClose_l>100);
    else 
        bNeedToInsertClose = nTrackedClose_l<40 && nNonTrackedClose_l>30;
    // Step 7：决策是否需要插入关键帧
    // Threshold
    float thRefRatio = 0.75f;
    if(nKFs<2)
        thRefRatio = 0.4f;

    if(mSensor==System::MONOCULAR)
        thRefRatio = 0.9f;

    if(mpCamera2) thRefRatio = 0.75f;

    if(mSensor==System::IMU_MONOCULAR)
    {
        if(mnMatchesInliers>350) // Points tracked from the local map
            thRefRatio = 0.75f;
        else
            thRefRatio = 0.90f;
    }
    //myplan 新增
    float thMapRatio = 0.35f;
    if (mnMatchesInliers > 300)
        thMapRatio = 0.30f;

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b = ((mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames) && bLocalMappingIdle);
    //Condition 1c: tracking is weak
    const bool c1c = mSensor!=System::MONOCULAR && mSensor!=System::IMU_MONOCULAR && mSensor!=System::IMU_STEREO && (mnMatchesInliers<nRefMatches*0.25 || bNeedToInsertClose|| ratioMap < 0.3f) ;
    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    const bool c2 = (((mnMatchesInliers<nRefMatches*thRefRatio || bNeedToInsertClose || ratioMap < thMapRatio)) && mnMatchesInliers>15);

    // note [EAO] create new keyframe by object.
    //myplan 新增
    bool c1d = false;
    //myplan 新增关键帧策略
//    if (mCurrentFrame.AppearNewObject)
//        c1d = true;

    // Temporal condition for Inertial cases
    bool c3 = false;
    if(mpLastKeyFrame)
    {
        if (mSensor==System::IMU_MONOCULAR)
        {
            if ((mCurrentFrame.mTimeStamp-mpLastKeyFrame->mTimeStamp)>=0.5)
                c3 = true;
        }
        else if (mSensor==System::IMU_STEREO)
        {
            if ((mCurrentFrame.mTimeStamp-mpLastKeyFrame->mTimeStamp)>=0.5)
                c3 = true;
        }
    }

    bool c4 = false;
    if ((((mnMatchesInliers<75) && (mnMatchesInliers>15)) || mState==RECENTLY_LOST) && ((mSensor == System::IMU_MONOCULAR))) // MODIFICATION_2, originally ((((mnMatchesInliers<75) && (mnMatchesInliers>15)) || mState==RECENTLY_LOST) && ((mSensor == System::IMU_MONOCULAR)))
        c4=true;
    else
        c4=false;

    if(((c1a||c1b||c1c) && c2)||c3 ||c4)
    {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        // Step 7.6：local mapping空闲时或者正在做imu初始化时可以直接插入，不空闲的时候要根据情况
        if(bLocalMappingIdle)
        {
            return 1;
        }
        else
        {
            mpLocalMapper->InterruptBA();
            if(mSensor!=System::MONOCULAR  && mSensor!=System::IMU_MONOCULAR)
            {
                if(mpLocalMapper->KeyframesInQueue()<3)
                    return 1;
                else
                    return 0;
            }
            else
                return 0;
        }
    }

        // note [EAO] create new keyframe by object.
        //myplan 新增物体判断
    else if (c1d)
    {
        if (bLocalMappingIdle)
        {
            return 2;
        }
        else
        {
            mpLocalMapper->InterruptBA();
            if (mSensor != System::MONOCULAR)
            {
                if (mpLocalMapper->KeyframesInQueue() < 3)
                    return 2;
                else
                    return 0;
            }
            else
                return 0;
        }
    }//end

    else
        return 0;
}


bool Tracking::NeedNewKeyFrameWithRGBD()//line
{
    if(((mSensor == System::IMU_MONOCULAR) || (mSensor == System::IMU_STEREO)) && !mpAtlas->GetCurrentMap()->isImuInitialized())
    {
        if (mSensor == System::IMU_MONOCULAR && (mCurrentFrame.mTimeStamp-mpLastKeyFrame->mTimeStamp)>=0.25)
            return true;
        else if (mSensor == System::IMU_STEREO && (mCurrentFrame.mTimeStamp-mpLastKeyFrame->mTimeStamp)>=0.25)
            return true;
        else
            return false;
    }

    if(mbOnlyTracking)
        return false;

    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
    {
        return false;
    }

    // Return false if IMU is initialazing
    if (mpLocalMapper->IsInitializing())
        return false;
    const int nKFs = mpAtlas->KeyFramesInMap();

    // Do not insert keyframes if not enough frames have passed from last relocalisation
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && nKFs>mMaxFrames)
    {
        return false;
    }

    // Tracked MapPoints in the reference keyframe
    int nMinObs = 3;
    if(nKFs<=2)
        nMinObs=2;
    int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);
    int nRefMatches_l = mpReferenceKF->TrackedMapLines(2); //new

    // Local Mapping accept keyframes?
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    // Check how many "close" points are being tracked and how many could be potentially created.
    int nNonTrackedClose = 0;
    int nTrackedClose= 0;
    int nNonTrackedClose_l = 0;
    int nTrackedClose_l= 0;

    if(mSensor!=System::MONOCULAR && mSensor!=System::IMU_MONOCULAR)
    {
        int N = (mCurrentFrame.Nleft == -1) ? mCurrentFrame.N : mCurrentFrame.Nleft;
        for(int i =0; i<N; i++)
        {
            if(mCurrentFrame.mvDepth[i]>0 && mCurrentFrame.mvDepth[i]<mThDepth)
            {
                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                    nTrackedClose++;
                else
                    nNonTrackedClose++;

            }
        }
        for(int i =0; i<mCurrentFrame.N_l; i++)
        {
            float minz = mLastFrame.mbf/mLastFrame.mvDisparity_l[i].first;
            float maxz = mLastFrame.mbf/mLastFrame.mvDisparity_l[i].second;
            if(minz>maxz)
            {
                float tmp=minz;
                minz=maxz;
                maxz=tmp;
            }
            if(minz>0 && maxz<mThDepth)
            {
                if(mCurrentFrame.mvpMapLines[i] && !mCurrentFrame.mvbOutlier_Line[i])
                    ++nTrackedClose_l;
                else
                    ++nNonTrackedClose_l;
            }
        }
    }

    bool bNeedToInsertClose = (nTrackedClose<100) && (nNonTrackedClose>70);
    bool bNeedToInsertClose_l = (nTrackedClose_l<20) && (nNonTrackedClose_l>100);

    // Thresholds
    float thRefRatio = 0.75f;
    float thRefRatio_l = 0.75f;
    if(nKFs<2)
    {
        thRefRatio = 0.4f;
        thRefRatio_l = 0.4f;
    }

    if(mSensor==System::MONOCULAR)
        thRefRatio = 0.9f;

    if(mpCamera2) thRefRatio = 0.75f;

    if(mSensor==System::IMU_MONOCULAR)
    {
        if(mnMatchesInliers>350) // Points tracked from the local map
            thRefRatio = 0.75f;
        else
            thRefRatio = 0.90f;
    }

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b = ((mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames) && bLocalMappingIdle);
    //Condition 1c: tracking is weak
    const bool c1c = mSensor!=System::MONOCULAR && mSensor!=System::IMU_MONOCULAR && mSensor!=System::IMU_STEREO && (mnMatchesInliers<nRefMatches*0.25 || bNeedToInsertClose) ;
    const bool c1c_l =  mSensor!=System::MONOCULAR && (mnMatchesInliers_l<nRefMatches_l*0.25 || bNeedToInsertClose_l);
    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    const bool c2 = (((mnMatchesInliers<nRefMatches*thRefRatio || bNeedToInsertClose)) && mnMatchesInliers>15);
    const bool c2_l = ((mnMatchesInliers_l<nRefMatches_l*thRefRatio_l|| bNeedToInsertClose_l) && mnMatchesInliers_l>10);

    // Temporal condition for Inertial cases
    bool c3 = false;
    if(mpLastKeyFrame)
    {
        if (mSensor==System::IMU_MONOCULAR)
        {
            if ((mCurrentFrame.mTimeStamp-mpLastKeyFrame->mTimeStamp)>=0.5)
                c3 = true;
        }
        else if (mSensor==System::IMU_STEREO)
        {
            if ((mCurrentFrame.mTimeStamp-mpLastKeyFrame->mTimeStamp)>=0.5)
                c3 = true;
        }
    }

    bool c4 = false;
    if ((((mnMatchesInliers<75) && (mnMatchesInliers>15)) || mState==RECENTLY_LOST) && ((mSensor == System::IMU_MONOCULAR))) // MODIFICATION_2, originally ((((mnMatchesInliers<75) && (mnMatchesInliers>15)) || mState==RECENTLY_LOST) && ((mSensor == System::IMU_MONOCULAR)))
        c4=true;
    else
        c4=false;

    if(((c1a||c1b||c1c||c1c_l) && (c2||c2_l))||c3 ||c4)
    {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        if(bLocalMappingIdle)
        {
            return true;
        }
        else
        {
            mpLocalMapper->InterruptBA();
            if(mSensor!=System::MONOCULAR  && mSensor!=System::IMU_MONOCULAR)
            {
                if(mpLocalMapper->KeyframesInQueue()<3)
                    return true;
                else
                    return false;
            }
            else
                return false;
        }
    }
    else
        return false;
}

void Tracking::CreateNewKeyFrame()
{
    if(mpLocalMapper->IsInitializing())
        return;

    if(!mpLocalMapper->SetNotStop(true))
        return;

    KeyFrame* pKF = new KeyFrame(mCurrentFrame,mpAtlas->GetCurrentMap(),mpKeyFrameDB);

    if(mpAtlas->isImuInitialized())
        pKF->bImu = true;

    pKF->SetNewBias(mCurrentFrame.mImuBias);
    mpReferenceKF = pKF;
    mCurrentFrame.mpReferenceKF = pKF;

    if(mpLastKeyFrame)
    {
        pKF->mPrevKF = mpLastKeyFrame;
        mpLastKeyFrame->mNextKF = pKF;
    }
    else
        Verbose::PrintMess("No last KF in KF creation!!", Verbose::VERBOSITY_NORMAL);

    // Reset preintegration from last KF (Create new object)
    if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO)
    {
        mpImuPreintegratedFromLastKF = new IMU::Preintegrated(pKF->GetImuBias(),pKF->mImuCalib);
    }

    if(mSensor!=System::MONOCULAR && mSensor != System::IMU_MONOCULAR) // TODO check if incluide imu_stereo
    {
        mCurrentFrame.UpdatePoseMatrices();
        // cout << "create new MPs" << endl;
        // We sort points by the measured depth by the stereo/RGBD sensor.
        // We create all those MapPoints whose depth < mThDepth.
        // If there are less than 100 close points we create the 100 closest.
        int maxPoint = 100;
        if(mSensor == System::IMU_STEREO)
            maxPoint = 100;

        vector<pair<float,int> > vDepthIdx;
        int N = (mCurrentFrame.Nleft != -1) ? mCurrentFrame.Nleft : mCurrentFrame.N;
        vDepthIdx.reserve(mCurrentFrame.N);
        for(int i=0; i<N; i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                vDepthIdx.push_back(make_pair(z,i));
            }
        }

        if(!vDepthIdx.empty())
        {
            sort(vDepthIdx.begin(),vDepthIdx.end());

            int nPoints = 0;
            for(size_t j=0; j<vDepthIdx.size();j++)
            {
                int i = vDepthIdx[j].second;

                bool bCreateNew = false;

                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(!pMP)
                    bCreateNew = true;
                else if(pMP->Observations()<1)
                {
                    bCreateNew = true;
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
                }

                if(bCreateNew)
                {
                    cv::Mat x3D;

                    if(mCurrentFrame.Nleft == -1){
                        x3D = mCurrentFrame.UnprojectStereo(i);
                    }
                    else{
                        x3D = mCurrentFrame.UnprojectStereoFishEye(i);
                    }

                    MapPoint* pNewMP = new MapPoint(x3D,pKF,mpAtlas->GetCurrentMap());
                    pNewMP->AddObservation(pKF,i);

                    //Check if it is a stereo observation in order to not
                    //duplicate mappoints
                    if(mCurrentFrame.Nleft != -1 && mCurrentFrame.mvLeftToRightMatch[i] >= 0){
                        mCurrentFrame.mvpMapPoints[mCurrentFrame.Nleft + mCurrentFrame.mvLeftToRightMatch[i]]=pNewMP;
                        pNewMP->AddObservation(pKF,mCurrentFrame.Nleft + mCurrentFrame.mvLeftToRightMatch[i]);
                        pKF->AddMapPoint(pNewMP,mCurrentFrame.Nleft + mCurrentFrame.mvLeftToRightMatch[i]);
                    }

                    pKF->AddMapPoint(pNewMP,i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpAtlas->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i]=pNewMP;
                    nPoints++;
                }
                else
                {
                    nPoints++; // TODO check ???
                }

                if(vDepthIdx[j].first>mThDepth && nPoints>maxPoint)
                {
                    break;
                }
            }

            Verbose::PrintMess("new mps for stereo KF: " + to_string(nPoints), Verbose::VERBOSITY_NORMAL);

        }
    }


    mpLocalMapper->InsertKeyFrame(pKF);

    mpLocalMapper->SetNotStop(false);

    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKF;
    //cout  << "end creating new KF" << endl;
}

void Tracking::CreateNewKeyFrameWithLines(bool CreateByObjs)
{
    // 如果局部建图线程正在初始化且没做完或关闭了,就无法插入关键帧
    if(mpLocalMapper->IsInitializing())
        return;

    if(!mpLocalMapper->SetNotStop(true))
        return;
    // Step 1：将当前帧构造成关键帧
    KeyFrame* pKF = new KeyFrame(mCurrentFrame,mpAtlas->GetCurrentMap(),mpKeyFrameDB);

    //myplan 传递mask图片  imp第三次修改
    pKF->set_segmentation_mask(mCurrentFrame._img_seg_mask);
    cv::Mat test = pKF->get_segmentation_mask();

    if(mpAtlas->isImuInitialized())
        pKF->bImu = true;

    pKF->SetNewBias(mCurrentFrame.mImuBias);
    // Step 2：将当前关键帧设置为当前帧的参考关键帧
    // 在UpdateLocalKeyFrames函数中会将与当前关键帧共视程度最高的关键帧设定为当前帧的参考关键帧
    mpReferenceKF = pKF;
    mCurrentFrame.mpReferenceKF = pKF;

    // save ovjects to keyframe.
    //myplan 新关键帧由发现新物体生成
    //keyframe created by objects.
    pKF->objects_kf = mCurrentFrame.mvObjectFrame;
    if (CreateByObjs)
        pKF->mbCreatedByObjs = true;

    if(mpLastKeyFrame)
    {
        pKF->mPrevKF = mpLastKeyFrame;
        mpLastKeyFrame->mNextKF = pKF;
    }
    else
        Verbose::PrintMess("No last KF in KF creation!!", Verbose::VERBOSITY_NORMAL);

    // Reset preintegration from last KF (Create new object)
    if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO)
    {
        mpImuPreintegratedFromLastKF = new IMU::Preintegrated(pKF->GetImuBias(),pKF->mImuCalib);
    }
    // 这段代码和 Tracking::UpdateLastFrame 中的那一部分代码功能相同
    // Step 3：对于双目或rgbd摄像头，为当前帧生成新的地图点；单目无操作
    if(mSensor!=System::MONOCULAR && mSensor != System::IMU_MONOCULAR) // TODO check if incluide imu_stereo
    {
        // 根据Tcw计算mRcw、mtcw和mRwc、mOw
        mCurrentFrame.UpdatePoseMatrices();
        // cout << "create new MPs" << endl;
        // We sort points by the measured depth by the stereo/RGBD sensor.
        // We create all those MapPoints whose depth < mThDepth.
        // If there are less than 100 close points we create the 100 closest.
        int maxPoint = 100;
        if(mSensor == System::IMU_STEREO)
            maxPoint = 100;

        vector<pair<float,int> > vDepthIdx;
        int N = (mCurrentFrame.Nleft != -1) ? mCurrentFrame.Nleft : mCurrentFrame.N;
        vDepthIdx.reserve(mCurrentFrame.N);
        for(int i=0; i<N; i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                vDepthIdx.push_back(make_pair(z,i));
            }
        }

        if(!vDepthIdx.empty())
        {
            // Step 3.2：按照深度从小到大排序
            sort(vDepthIdx.begin(),vDepthIdx.end());
            // Step 3.3：从中找出不是地图点的生成临时地图点
            // 处理的近点的个数
            int nPoints = 0;
            for(size_t j=0; j<vDepthIdx.size();j++)
            {
                int i = vDepthIdx[j].second;

                bool bCreateNew = false;
                // 如果这个点对应在上一帧中的地图点没有,或者创建后就没有被观测到,那么就生成一个临时的地图点
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(!pMP)
                    bCreateNew = true;
                else if(pMP->Observations()<1)
                {
                    bCreateNew = true;
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
                }
                // 如果需要就新建地图点，这里的地图点不是临时的，是全局地图中新建地图点，用于跟踪
                if(bCreateNew)
                {
                    cv::Mat x3D;

                    if(mCurrentFrame.Nleft == -1){
                        x3D = mCurrentFrame.UnprojectStereo(i);
                    }
                    else{
                        x3D = mCurrentFrame.UnprojectStereoFishEye(i);
                    }

                    MapPoint* pNewMP = new MapPoint(x3D,pKF,mpAtlas->GetCurrentMap());
                    pNewMP->AddObservation(pKF,i);

                    //Check if it is a stereo observation in order to not
                    //duplicate mappoints
                    if(mCurrentFrame.Nleft != -1 && mCurrentFrame.mvLeftToRightMatch[i] >= 0){
                        mCurrentFrame.mvpMapPoints[mCurrentFrame.Nleft + mCurrentFrame.mvLeftToRightMatch[i]]=pNewMP;
                        pNewMP->AddObservation(pKF,mCurrentFrame.Nleft + mCurrentFrame.mvLeftToRightMatch[i]);
                        pKF->AddMapPoint(pNewMP,mCurrentFrame.Nleft + mCurrentFrame.mvLeftToRightMatch[i]);
                    }

                    pKF->AddMapPoint(pNewMP,i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpAtlas->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i]=pNewMP;
                    nPoints++;
                }
                else
                {
                    nPoints++; // TODO check ???
                }

                if(vDepthIdx[j].first>mThDepth && nPoints>maxPoint)
                {
                    break;
                }
            }

            Verbose::PrintMess("new mps for stereo KF: " + to_string(nPoints), Verbose::VERBOSITY_NORMAL);

        }

        // We sort lines' endpoints by the measured depth by the stereo/RGBD sensor.
        // We create all those MapLine whose depth < mThDepth.
        // If there are less than 50 close lines we create the 50 closest.
        vDepthIdx.clear();
        vDepthIdx.reserve(mCurrentFrame.N_l);
        for(int i=0; i<mCurrentFrame.N_l; i++)
        {
            float sz = mCurrentFrame.mvDepth_l[i].first;
            float ez = mCurrentFrame.mvDepth_l[i].second;
            if(sz<0||ez<0) continue;
            float z = sz>ez?sz:ez;
            vDepthIdx.push_back(make_pair(z,i));
        }

        if(!vDepthIdx.empty())
        {
            sort(vDepthIdx.begin(),vDepthIdx.end());

            int nLines = 0;
            for(size_t j=0; j<vDepthIdx.size();j++)
            {
                int i = vDepthIdx[j].second;

                bool bCreateNew = false;

                MapLine* pML = mCurrentFrame.mvpMapLines[i];
                if(!pML)
                    bCreateNew = true;
                else if(pML->Observations()<1)
                {
                    bCreateNew = true;
                    mCurrentFrame.mvpMapLines[i] = static_cast<MapLine*>(NULL);
                }

                if(bCreateNew)
                {
                    Eigen::Vector3d sp3D = mCurrentFrame.UnprojectStereoLines(
                                mCurrentFrame.mvKeysUn_Line[i].startPointX,
                                mCurrentFrame.mvKeysUn_Line[i].startPointY,
                                mCurrentFrame.mvDepth_l[i].first);
                    Eigen::Vector3d ep3D = mCurrentFrame.UnprojectStereoLines(
                                mCurrentFrame.mvKeysUn_Line[i].endPointX,
                                mCurrentFrame.mvKeysUn_Line[i].endPointY,
                                mCurrentFrame.mvDepth_l[i].second);
//                    std::cout<<"brefor 第2个CreateNewKeyFrameWithLines线的mapline 的有参构造"<<std::endl;
                    //跑起来进入这个
                    MapLine* pNewML = new MapLine(sp3D,ep3D,pKF,mpAtlas->GetCurrentMap());

                    pNewML->AddObservation(pKF,i);
                    pKF->AddMapLine(pNewML,i);
                    pNewML->ComputeDistinctiveDescriptors();
                    pNewML->UpdateNormalAndDepth();
        mpAtlas->AddMapLine(pNewML);

                    mCurrentFrame.mvpMapLines[i]=pNewML;
                    nLines++;
                }
                else
                {
                    nLines++;
                }

                if(vDepthIdx[j].first>mThDepth && nLines>50)
                    break;
            }
        }
    }


    mpLocalMapper->InsertKeyFrame(pKF);

    mpLocalMapper->SetNotStop(false);

    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKF;
    //cout  << "end creating new KF" << endl;
}

void Tracking::SearchLocalPoints()
{
    // Do not search map points already matched
    for(vector<MapPoint*>::iterator vit=mCurrentFrame.mvpMapPoints.begin(), vend=mCurrentFrame.mvpMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP)
        {
            if(pMP->isBad())
            {
                *vit = static_cast<MapPoint*>(NULL);
            }
            else
            {
                pMP->IncreaseVisible();
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                pMP->mbTrackInView = false;
                pMP->mbTrackInViewR = false;
            }
        }
    }

    int nToMatch=0;

    // Project points in frame and check its visibility
    for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;

        if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        if(pMP->isBad())
            continue;
        // Project (this fills MapPoint variables for matching)
        if(mCurrentFrame.isInFrustum(pMP,0.5))
        {
            pMP->IncreaseVisible();
            nToMatch++;
        }
        if(pMP->mbTrackInView)
        {
            mCurrentFrame.mmProjectPoints[pMP->mnId] = cv::Point2f(pMP->mTrackProjX, pMP->mTrackProjY);
        }
    }

    if(nToMatch>0)
    {
        ORBmatcher matcher(0.8);
        int th = 1;
        if(mSensor==System::RGBD)
            th=3;
        if(mpAtlas->isImuInitialized())
        {
            if(mpAtlas->GetCurrentMap()->GetIniertialBA2())
                th=2;
            else
                th=3;
        }
        else if(!mpAtlas->isImuInitialized() && (mSensor==System::IMU_MONOCULAR || mSensor==System::IMU_STEREO))
        {
            th=10;
        }

        // If the camera has been relocalised recently, perform a coarser search
        if(mCurrentFrame.mnId<mnLastRelocFrameId+2)
            th=5;

        if(mState==LOST || mState==RECENTLY_LOST) // Lost for less than 1 second
            th=15; // 15

        int matches = matcher.SearchByProjection(mCurrentFrame, mvpLocalMapPoints, th, mpLocalMapper->mbFarPoints, mpLocalMapper->mThFarPoints);
    }
}

void Tracking::SearchLocalPointsAndLines()
{
    // Do not search map points already matched
    for(vector<MapPoint*>::iterator vit=mCurrentFrame.mvpMapPoints.begin(), vend=mCurrentFrame.mvpMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP)
        {
            if(pMP->isBad())
            {
                *vit = static_cast<MapPoint*>(NULL);
            }
            else
            {
                pMP->IncreaseVisible();
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                pMP->mbTrackInView = false;
                pMP->mbTrackInViewR = false;
            }
        }
    }

    for(vector<MapLine*>::iterator vit=mCurrentFrame.mvpMapLines.begin(), vend=mCurrentFrame.mvpMapLines.end(); vit!=vend; vit++)
    {
        MapLine* pML = *vit;
        if(pML)
        {
            if(pML->isBad())
            {
                *vit = static_cast<MapLine*>(NULL);
            }
            else
            {
                pML->IncreaseVisible();
                pML->mnLastFrameSeen = mCurrentFrame.mnId;
                pML->mbTrackInView = false;
            }
        }
    }

    int nToMatch=0;

    // Project points in frame and check its visibility
    for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;

        if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        if(pMP->isBad())
            continue;
        // Project (this fills MapPoint variables for matching)
        if(mCurrentFrame.isInFrustum(pMP,0.5))
        {
            pMP->IncreaseVisible();
            nToMatch++;
        }
        if(pMP->mbTrackInView)
        {
            mCurrentFrame.mmProjectPoints[pMP->mnId] = cv::Point2f(pMP->mTrackProjX, pMP->mTrackProjY);
        }
    }

    if(nToMatch>0)
    {
        ORBmatcher matcher(0.8);
        int th = 1;
        if(mSensor==System::RGBD)
            th=3;
        if(mpAtlas->isImuInitialized())
        {
            if(mpAtlas->GetCurrentMap()->GetIniertialBA2())
                th=2;
            else
                th=3;
        }
        else if(!mpAtlas->isImuInitialized() && (mSensor==System::IMU_MONOCULAR || mSensor==System::IMU_STEREO))
        {
            th=10;
        }

        // If the camera has been relocalised recently, perform a coarser search
        if(mCurrentFrame.mnId<mnLastRelocFrameId+2)
            th=5;

        if(mState==LOST || mState==RECENTLY_LOST) // Lost for less than 1 second
            th=15; // 15

        mCurrentFrame.n_inliers_pt = matcher.SearchByProjection(mCurrentFrame, mvpLocalMapPoints, th, mpLocalMapper->mbFarPoints, mpLocalMapper->mThFarPoints);
    }

    // Project lines in frame and check its visibility
    nToMatch = 0;

    vector<MapLine*> mvpLocalMapLines_InFrustum;
    mvpLocalMapLines_InFrustum.reserve(mvpLocalMapLines.size());
    for(vector<MapLine*>::iterator vit=mvpLocalMapLines.begin(), vend=mvpLocalMapLines.end(); vit!=vend; vit++)
    {
        MapLine* pML = *vit;
        if(pML->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        if(pML->isBad())
            continue;

        // Project (this fills MapLine variables for matching)
        if(mCurrentFrame.isInFrustum_l(pML,0.5))
        {
            pML->IncreaseVisible();
            nToMatch++;
            mvpLocalMapLines_InFrustum.push_back(pML);   
        }
    }

    std::vector<int> matches_12;
    float minRatio12L = 0.9;
    if(nToMatch>0)
    {
        LineMatcher::match(mvpLocalMapLines_InFrustum,mCurrentFrame, minRatio12L, matches_12);
    }

    const double deltaAngle = M_PI/10.0;
    const double deltaWidth = (mCurrentFrame.mnMaxX-mCurrentFrame.mnMinX)*0.1;
    const double deltaHeight = (mCurrentFrame.mnMaxY-mCurrentFrame.mnMinY)*0.1;
    for (int i1 = 0, nmatches_12 = matches_12.size(); i1 < nmatches_12; ++i1) {     // remove maplines that do not match with orientation and position in image
        int i2 = matches_12[i1];
        if (i2 < 0) continue;

        if(mCurrentFrame.mvpMapLines[i2])
            if(mCurrentFrame.mvpMapLines[i2]->Observations()>0)
                continue;               

        MapLine* pML = mvpLocalMapLines_InFrustum[i1];

        // check for orientation and position in image
        if(true) {
            // check for orientation
            double theta = mCurrentFrame.mvKeysUn_Line[i2].angle - pML->mnTrackangle;
            if(theta<-M_PI) theta+=2*M_PI;
            else if(theta>M_PI) theta-=2*M_PI;
            if(fabs(theta)>deltaAngle) {
                matches_12[i1] = -1;
                continue;
            }

            // check for position in image
            const float sX_curr = mCurrentFrame.mvKeysUn_Line[i2].startPointX;
            const float sX_last = pML->mTrackProjsX;
            const float sY_curr = mCurrentFrame.mvKeysUn_Line[i2].startPointY;
            const float sY_last = pML->mTrackProjsY;
            const float eX_curr = mCurrentFrame.mvKeysUn_Line[i2].endPointX;
            const float eX_last = pML->mTrackProjeX;
            const float eY_curr = mCurrentFrame.mvKeysUn_Line[i2].endPointY;
            const float eY_last = pML->mTrackProjeY;
            if(fabs(sX_curr-sX_last)>deltaWidth || fabs(eX_curr-eX_last)>deltaWidth || fabs(sY_curr-sY_last)>deltaHeight || fabs(eY_curr-eY_last)>deltaHeight )
            {
                matches_12[i1] = -1;
                continue;
            }
        }

        mCurrentFrame.mvpMapLines[i2] = pML;
    }

    mCurrentFrame.n_inliers_ls = 0;
    for(auto it=mCurrentFrame.mvpMapLines.begin(),itend=mCurrentFrame.mvpMapLines.end(); it!=itend; ++it)
        if(*it) ++mCurrentFrame.n_inliers_ls; 

    mCurrentFrame.n_inliers = mCurrentFrame.n_inliers_ls + mCurrentFrame.n_inliers_pt;

}

void Tracking::UpdateLocalMap()
{
    // This is for visualization
    mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

    // Update
    UpdateLocalKeyFrames();
    UpdateLocalPoints();
}

void Tracking::UpdateLocalMapWithLines()
{
    // This is for visualization
    mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);
    mpAtlas->SetReferenceMapLines(mvpLocalMapLines);

    // Update
    UpdateLocalKeyFramesWithLines();
    UpdateLocalPointsAndLines();
}

void Tracking::UpdateLocalPoints()
{
    mvpLocalMapPoints.clear();

    int count_pts = 0;

    for(vector<KeyFrame*>::const_reverse_iterator itKF=mvpLocalKeyFrames.rbegin(), itEndKF=mvpLocalKeyFrames.rend(); itKF!=itEndKF; ++itKF)
    {
        KeyFrame* pKF = *itKF;
        const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

        for(vector<MapPoint*>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {

            MapPoint* pMP = *itMP;
            if(!pMP)
                continue;
            if(pMP->mnTrackReferenceForFrame==mCurrentFrame.mnId)
                continue;
            if(!pMP->isBad())
            {
                count_pts++;
                mvpLocalMapPoints.push_back(pMP);
                pMP->mnTrackReferenceForFrame=mCurrentFrame.mnId;
            }
        }
    }
}

void Tracking::UpdateLocalPointsAndLines()
{
    mvpLocalMapPoints.clear();
    mvpLocalMapLines.clear();

    int count_pts = 0;

    for(vector<KeyFrame*>::const_reverse_iterator itKF=mvpLocalKeyFrames.rbegin(), itEndKF=mvpLocalKeyFrames.rend(); itKF!=itEndKF; ++itKF)
    {
        KeyFrame* pKF = *itKF;
        const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

        for(vector<MapPoint*>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {

            MapPoint* pMP = *itMP;
            if(!pMP)
                continue;
            if(pMP->mnTrackReferenceForFrame==mCurrentFrame.mnId)
                continue;
            if(!pMP->isBad())
            {
                count_pts++;
                mvpLocalMapPoints.push_back(pMP);
                pMP->mnTrackReferenceForFrame=mCurrentFrame.mnId;
            }
        }

        const vector<MapLine*> vpMLs = pKF->GetMapLineMatches();
        for(vector<MapLine*>::const_iterator itML=vpMLs.begin(), itEndMP=vpMLs.end(); itML!=itEndMP; itML++)
        {
            MapLine* pML = *itML;
            if(!pML)
                continue;
            if(pML->mnTrackReferenceForFrame==mCurrentFrame.mnId)
                continue;
            if(!pML->isBad())
            {
                mvpLocalMapLines.push_back(pML);
                pML->mnTrackReferenceForFrame=mCurrentFrame.mnId;
            }
        }
    }
}

void Tracking::UpdateLocalKeyFrames()
{
    // Each map point vote for the keyframes in which it has been observed
    map<KeyFrame*,int> keyframeCounter;
    if(!mpAtlas->isImuInitialized() || (mCurrentFrame.mnId<mnLastRelocFrameId+2))
    {
        for(int i=0; i<mCurrentFrame.N; i++)
        {
            MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
            if(pMP)
            {
                if(!pMP->isBad())
                {
                    const map<KeyFrame*,tuple<int,int>> observations = pMP->GetObservations();
                    for(map<KeyFrame*,tuple<int,int>>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                        keyframeCounter[it->first]++;
                }
                else
                {
                    mCurrentFrame.mvpMapPoints[i]=NULL;
                }
            }
        }
    }
    else
    {
        for(int i=0; i<mLastFrame.N; i++)
        {
            // Using lastframe since current frame has not matches yet
            if(mLastFrame.mvpMapPoints[i])
            {
                MapPoint* pMP = mLastFrame.mvpMapPoints[i];
                if(!pMP)
                    continue;
                if(!pMP->isBad())
                {
                    const map<KeyFrame*,tuple<int,int>> observations = pMP->GetObservations();
                    for(map<KeyFrame*,tuple<int,int>>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                        keyframeCounter[it->first]++;
                }
                else
                {
                    // MODIFICATION
                    mLastFrame.mvpMapPoints[i]=NULL;
                }
            }
        }
    }


    int max=0;
    KeyFrame* pKFmax= static_cast<KeyFrame*>(NULL);

    mvpLocalKeyFrames.clear();
    mvpLocalKeyFrames.reserve(3*keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    for(map<KeyFrame*,int>::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
    {
        KeyFrame* pKF = it->first;

        if(pKF->isBad())
            continue;

        if(it->second>max)
        {
            max=it->second;
            pKFmax=pKF;
        }

        mvpLocalKeyFrames.push_back(pKF);
        pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
    }

    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        // Limit the number of keyframes
        if(mvpLocalKeyFrames.size()>80) // 80
            break;

        KeyFrame* pKF = *itKF;

        const vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);


        for(vector<KeyFrame*>::const_iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
        {
            KeyFrame* pNeighKF = *itNeighKF;
            if(!pNeighKF->isBad())
            {
                if(pNeighKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pNeighKF);
                    pNeighKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        const set<KeyFrame*> spChilds = pKF->GetChilds();
        for(set<KeyFrame*>::const_iterator sit=spChilds.begin(), send=spChilds.end(); sit!=send; sit++)
        {
            KeyFrame* pChildKF = *sit;
            if(!pChildKF->isBad())
            {
                if(pChildKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pChildKF);
                    pChildKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        KeyFrame* pParent = pKF->GetParent();
        if(pParent)
        {
            if(pParent->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
            {
                mvpLocalKeyFrames.push_back(pParent);
                pParent->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                break;
            }
        }
    }

    // Add 10 last temporal KFs (mainly for IMU)
    if((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO) &&mvpLocalKeyFrames.size()<80)
    {
        //cout << "CurrentKF: " << mCurrentFrame.mnId << endl;
        KeyFrame* tempKeyFrame = mCurrentFrame.mpLastKeyFrame;

        const int Nd = 20;
        for(int i=0; i<Nd; i++){
            if (!tempKeyFrame)
                break;
            //cout << "tempKF: " << tempKeyFrame << endl;
            if(tempKeyFrame->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
            {
                mvpLocalKeyFrames.push_back(tempKeyFrame);
                tempKeyFrame->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                tempKeyFrame=tempKeyFrame->mPrevKF;
            }
        }
    }

    if(pKFmax)
    {
        mpReferenceKF = pKFmax;
        mCurrentFrame.mpReferenceKF = mpReferenceKF;
    }
}

void Tracking::UpdateLocalKeyFramesWithLines()
{
    // Each map point and map line vote for the keyframes in which it has been observed
    map<KeyFrame*,int> keyframeCounter;
    if(!mpAtlas->isImuInitialized() || (mCurrentFrame.mnId<mnLastRelocFrameId+2))
    {
        for(int i=0; i<mCurrentFrame.N; i++)
        {
            MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
            if(pMP)
            {
                if(!pMP->isBad())
                {
                    const map<KeyFrame*,tuple<int,int>> observations = pMP->GetObservations();
                    for(map<KeyFrame*,tuple<int,int>>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                        keyframeCounter[it->first]++;
                }
                else
                {
                    mCurrentFrame.mvpMapPoints[i]=NULL;
                }
            }
        }
        for(int i=0; i<mCurrentFrame.N_l; i++)
        {
            if(mCurrentFrame.mvpMapLines[i])
            {
                MapLine* pML = mCurrentFrame.mvpMapLines[i];
                if(!pML->isBad())
                {
                    const map<KeyFrame*,size_t> observations = pML->GetObservations();
                    for(map<KeyFrame*,size_t>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                        keyframeCounter[it->first]++;
                }
                else
                {
                    mCurrentFrame.mvpMapLines[i]=NULL;
                }
            }
        }
    }
    else
    {
        for(int i=0; i<mLastFrame.N; i++)
        {
            // Using lastframe since current frame has not matches yet
            if(mLastFrame.mvpMapPoints[i])
            {
                MapPoint* pMP = mLastFrame.mvpMapPoints[i];
                if(!pMP)
                    continue;
                if(!pMP->isBad())
                {
                    const map<KeyFrame*,tuple<int,int>> observations = pMP->GetObservations();
                    for(map<KeyFrame*,tuple<int,int>>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                        keyframeCounter[it->first]++;
                }
                else
                {
                    // MODIFICATION
                    mLastFrame.mvpMapPoints[i]=NULL;
                }
            }
        }
        for(int i=0; i<mLastFrame.N_l; i++)
        {
            if(mLastFrame.mvpMapLines[i])
            {
                MapLine* pML = mLastFrame.mvpMapLines[i];
                if(!pML)
                    continue;
                if(!pML->isBad())
                {
                    const map<KeyFrame*,size_t> observations = pML->GetObservations();
                    for(map<KeyFrame*,size_t>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                        keyframeCounter[it->first]++;
                }
                else
                {
                    mLastFrame.mvpMapLines[i]=NULL;
                }
            }
        }
        
    }


    int max=0;
    KeyFrame* pKFmax= static_cast<KeyFrame*>(NULL);

    mvpLocalKeyFrames.clear();
    mvpLocalKeyFrames.reserve(3*keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    for(map<KeyFrame*,int>::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
    {
        KeyFrame* pKF = it->first;

        if(pKF->isBad())
            continue;

        if(it->second>max)
        {
            max=it->second;
            pKFmax=pKF;
        }

        mvpLocalKeyFrames.push_back(pKF);
        pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
    }

    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        // Limit the number of keyframes
        if(mvpLocalKeyFrames.size()>80) // 80
            break;

        KeyFrame* pKF = *itKF;

        const vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);


        for(vector<KeyFrame*>::const_iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
        {
            KeyFrame* pNeighKF = *itNeighKF;
            if(!pNeighKF->isBad())
            {
                if(pNeighKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pNeighKF);
                    pNeighKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        const set<KeyFrame*> spChilds = pKF->GetChilds();
        for(set<KeyFrame*>::const_iterator sit=spChilds.begin(), send=spChilds.end(); sit!=send; sit++)
        {
            KeyFrame* pChildKF = *sit;
            if(!pChildKF->isBad())
            {
                if(pChildKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pChildKF);
                    pChildKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        KeyFrame* pParent = pKF->GetParent();
        if(pParent)
        {
            if(pParent->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
            {
                mvpLocalKeyFrames.push_back(pParent);
                pParent->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                break;
            }
        }
    }

    // Add 10 last temporal KFs (mainly for IMU)
    if((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO) &&mvpLocalKeyFrames.size()<80)
    {
        //cout << "CurrentKF: " << mCurrentFrame.mnId << endl;
        KeyFrame* tempKeyFrame = mCurrentFrame.mpLastKeyFrame;

        const int Nd = 20;
        for(int i=0; i<Nd; i++){
            if (!tempKeyFrame)
                break;
            //cout << "tempKF: " << tempKeyFrame << endl;
            if(tempKeyFrame->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
            {
                mvpLocalKeyFrames.push_back(tempKeyFrame);
                tempKeyFrame->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                tempKeyFrame=tempKeyFrame->mPrevKF;
            }
        }
    }

    if(pKFmax)
    {
        mpReferenceKF = pKFmax;
        mCurrentFrame.mpReferenceKF = mpReferenceKF;
    }
}

bool Tracking::Relocalization()
{
    Verbose::PrintMess("Starting relocalization", Verbose::VERBOSITY_NORMAL);
    // Compute Bag of Words Vector
    mCurrentFrame.ComputeBoW();

    // Relocalization is performed when tracking is lost
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame, mpAtlas->GetCurrentMap());

    if(vpCandidateKFs.empty()) {
        Verbose::PrintMess("There are not candidates", Verbose::VERBOSITY_NORMAL);
        return false;
    }

    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.75,true);

    vector<MLPnPsolver*> vpMLPnPsolvers;
    vpMLPnPsolvers.resize(nKFs);

    vector<vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates=0;

    for(int i=0; i<nKFs; i++)
    {
        KeyFrame* pKF = vpCandidateKFs[i];
        if(pKF->isBad())
            vbDiscarded[i] = true;
        else
        {
            int nmatches = matcher.SearchByBoW(pKF,mCurrentFrame,vvpMapPointMatches[i]);
            if(nmatches<15)
            {
                vbDiscarded[i] = true;
                continue;
            }
            else
            {
                MLPnPsolver* pSolver = new MLPnPsolver(mCurrentFrame,vvpMapPointMatches[i]);
                pSolver->SetRansacParameters(0.99,10,300,6,0.5,5.991);  //This solver needs at least 6 points
                vpMLPnPsolvers[i] = pSolver;
            }
        }
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool bMatch = false;
    ORBmatcher matcher2(0.9,true);

    while(nCandidates>0 && !bMatch)
    {
        for(int i=0; i<nKFs; i++)
        {
            if(vbDiscarded[i])
                continue;

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            MLPnPsolver* pSolver = vpMLPnPsolvers[i];
            cv::Mat Tcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if(!Tcw.empty())
            {
                Tcw.copyTo(mCurrentFrame.mTcw);

                set<MapPoint*> sFound;

                const int np = vbInliers.size();

                for(int j=0; j<np; j++)
                {
                    if(vbInliers[j])
                    {
                        mCurrentFrame.mvpMapPoints[j]=vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    }
                    else
                        mCurrentFrame.mvpMapPoints[j]=NULL;
                }

                int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                if(nGood<10)
                    continue;

                for(int io =0; io<mCurrentFrame.N; io++)
                    if(mCurrentFrame.mvbOutlier[io])
                        mCurrentFrame.mvpMapPoints[io]=static_cast<MapPoint*>(NULL);

                // If few inliers, search by projection in a coarse window and optimize again
                if(nGood<50)
                {
                    int nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,10,100);

                    if(nadditional+nGood>=50)
                    {
                        nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        if(nGood>30 && nGood<50)
                        {
                            sFound.clear();
                            for(int ip =0; ip<mCurrentFrame.N; ip++)
                                if(mCurrentFrame.mvpMapPoints[ip])
                                    sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                            nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,3,64);

                            // Final optimization
                            if(nGood+nadditional>=50)
                            {
                                nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                                for(int io =0; io<mCurrentFrame.N; io++)
                                    if(mCurrentFrame.mvbOutlier[io])
                                        mCurrentFrame.mvpMapPoints[io]=NULL;
                            }
                        }
                    }
                }


                // If the pose is supported by enough inliers stop ransacs and continue
                if(nGood>=50)
                {
                    bMatch = true;
                    break;
                }
            }
        }
    }

    if(!bMatch)
    {
        return false;
    }
    else
    {
        mnLastRelocFrameId = mCurrentFrame.mnId;
        cout << "Relocalized!!" << endl;
        return true;
    }

}

void Tracking::Reset(bool bLocMap)
{
    Verbose::PrintMess("System Reseting", Verbose::VERBOSITY_NORMAL);

    if(mpViewer)
    {
        mpViewer->RequestStop();
        while(!mpViewer->isStopped())
            usleep(3000);
    }

    // Reset Local Mapping
    if (!bLocMap)
    {
        Verbose::PrintMess("Reseting Local Mapper...", Verbose::VERBOSITY_NORMAL);
        mpLocalMapper->RequestReset();
        Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);
    }


    // Reset Loop Closing
    Verbose::PrintMess("Reseting Loop Closing...", Verbose::VERBOSITY_NORMAL);
    mpLoopClosing->RequestReset();
    Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

    // Reset Semi Dense Mapping
    //myplan
    cout << "Reseting Semi Dense Mapping...";
    mpSemiDenseMapping->RequestReset();
    cout << " done" << endl;

    // Clear BoW Database
    Verbose::PrintMess("Reseting Database...", Verbose::VERBOSITY_NORMAL);
    mpKeyFrameDB->clear();
    Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

    // Clear Map (this erase MapPoints and KeyFrames)
    mpAtlas->clearAtlas();
    mpAtlas->CreateNewMap();
    if (mSensor==System::IMU_STEREO || mSensor == System::IMU_MONOCULAR)
        mpAtlas->SetInertialSensor();
    mnInitialFrameId = 0;

    KeyFrame::nNextId = 0;
    KeyFrame::nNextMappingId = 0;//myplan
    Frame::nNextId = 0;
    mState = NO_IMAGES_YET;

    if(mpInitializer)
    {
        delete mpInitializer;
        mpInitializer = static_cast<Initializer*>(NULL);
    }
    mbSetInit=false;

    mlRelativeFramePoses.clear();
    mlpReferences.clear();
    mlFrameTimes.clear();
    mlbLost.clear();
    mCurrentFrame = Frame();
    mnLastRelocFrameId = 0;
    mLastFrame = Frame();
    mpReferenceKF = static_cast<KeyFrame*>(NULL);
    mpLastKeyFrame = static_cast<KeyFrame*>(NULL);
    mvIniMatches.clear();

    if(mpViewer)
        mpViewer->Release();

    Verbose::PrintMess("   End reseting! ", Verbose::VERBOSITY_NORMAL);
}

void Tracking::ResetActiveMap(bool bLocMap)
{
    Verbose::PrintMess("Active map Reseting", Verbose::VERBOSITY_NORMAL);
    if(mpViewer)
    {
        mpViewer->RequestStop();
        while(!mpViewer->isStopped())
            usleep(3000);
    }

    Map* pMap = mpAtlas->GetCurrentMap();

    if (!bLocMap)
    {
        Verbose::PrintMess("Reseting Local Mapper...", Verbose::VERBOSITY_NORMAL);
        mpLocalMapper->RequestResetActiveMap(pMap);
        Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);
    }

    // Reset Loop Closing
    Verbose::PrintMess("Reseting Loop Closing...", Verbose::VERBOSITY_NORMAL);
    mpLoopClosing->RequestResetActiveMap(pMap);
    Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

    // Clear BoW Database
    Verbose::PrintMess("Reseting Database", Verbose::VERBOSITY_NORMAL);
    mpKeyFrameDB->clearMap(pMap); // Only clear the active map references
    Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

    // Clear Map (this erase MapPoints and KeyFrames)
    mpAtlas->clearMap();


    //KeyFrame::nNextId = mpAtlas->GetLastInitKFid();
    //Frame::nNextId = mnLastInitFrameId;
    mnLastInitFrameId = Frame::nNextId;
    mnLastRelocFrameId = mnLastInitFrameId;
    mState = NO_IMAGES_YET; //NOT_INITIALIZED;

    if(mpInitializer)
    {
        delete mpInitializer;
        mpInitializer = static_cast<Initializer*>(NULL);
    }

    list<bool> lbLost;
    // lbLost.reserve(mlbLost.size());
    unsigned int index = mnFirstFrameId;
    cout << "mnFirstFrameId = " << mnFirstFrameId << endl;
    for(Map* pMap : mpAtlas->GetAllMaps())
    {
        if(pMap->GetAllKeyFrames().size() > 0)
        {
            if(index > pMap->GetLowerKFID())
                index = pMap->GetLowerKFID();
        }
    }

    //cout << "First Frame id: " << index << endl;
    int num_lost = 0;
    cout << "mnInitialFrameId = " << mnInitialFrameId << endl;

    for(list<bool>::iterator ilbL = mlbLost.begin(); ilbL != mlbLost.end(); ilbL++)
    {
        if(index < mnInitialFrameId)
            lbLost.push_back(*ilbL);
        else
        {
            lbLost.push_back(true);
            num_lost += 1;
        }

        index++;
    }
    cout << num_lost << " Frames set to lost" << endl;

    mlbLost = lbLost;

    mnInitialFrameId = mCurrentFrame.mnId;
    mnLastRelocFrameId = mCurrentFrame.mnId;

    mCurrentFrame = Frame();
    mLastFrame = Frame();
    mpReferenceKF = static_cast<KeyFrame*>(NULL);
    mpLastKeyFrame = static_cast<KeyFrame*>(NULL);
    mvIniMatches.clear();

    if(mpViewer)
        mpViewer->Release();

    Verbose::PrintMess("   End reseting! ", Verbose::VERBOSITY_NORMAL);
}

vector<MapPoint*> Tracking::GetLocalMapMPS()
{
    return mvpLocalMapPoints;
}

void Tracking::ChangeCalibration(const string &strSettingPath)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    Frame::mbInitialComputations = true;
}

void Tracking::InformOnlyTracking(const bool &flag)
{
    mbOnlyTracking = flag;
}

void Tracking::UpdateFrameIMU(const float s, const IMU::Bias &b, KeyFrame* pCurrentKeyFrame)
{
    Map * pMap = pCurrentKeyFrame->GetMap();
    unsigned int index = mnFirstFrameId;
    list<ORB_SLAM3::KeyFrame*>::iterator lRit = mlpReferences.begin();
    list<bool>::iterator lbL = mlbLost.begin();
    for(list<cv::Mat>::iterator lit=mlRelativeFramePoses.begin(),lend=mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lbL++)
    {
        if(*lbL)
            continue;

        KeyFrame* pKF = *lRit;

        while(pKF->isBad())
        {
            pKF = pKF->GetParent();
        }

        if(pKF->GetMap() == pMap)
        {
            (*lit).rowRange(0,3).col(3)=(*lit).rowRange(0,3).col(3)*s;
        }
    }

    mLastBias = b;

    mpLastKeyFrame = pCurrentKeyFrame;

    mLastFrame.SetNewBias(mLastBias);
    mCurrentFrame.SetNewBias(mLastBias);

    cv::Mat Gz = (cv::Mat_<float>(3,1) << 0, 0, -IMU::GRAVITY_VALUE);

    cv::Mat twb1;
    cv::Mat Rwb1;
    cv::Mat Vwb1;
    float t12;

    while(!mCurrentFrame.imuIsPreintegrated())
    {
        usleep(500);
    }


    if(mLastFrame.mnId == mLastFrame.mpLastKeyFrame->mnFrameId)
    {
        mLastFrame.SetImuPoseVelocity(mLastFrame.mpLastKeyFrame->GetImuRotation(),
                                      mLastFrame.mpLastKeyFrame->GetImuPosition(),
                                      mLastFrame.mpLastKeyFrame->GetVelocity());
    }
    else
    {
        twb1 = mLastFrame.mpLastKeyFrame->GetImuPosition();
        Rwb1 = mLastFrame.mpLastKeyFrame->GetImuRotation();
        Vwb1 = mLastFrame.mpLastKeyFrame->GetVelocity();
        t12 = mLastFrame.mpImuPreintegrated->dT;

        mLastFrame.SetImuPoseVelocity(Rwb1*mLastFrame.mpImuPreintegrated->GetUpdatedDeltaRotation(),
                                      twb1 + Vwb1*t12 + 0.5f*t12*t12*Gz+ Rwb1*mLastFrame.mpImuPreintegrated->GetUpdatedDeltaPosition(),
                                      Vwb1 + Gz*t12 + Rwb1*mLastFrame.mpImuPreintegrated->GetUpdatedDeltaVelocity());
    }

    if (mCurrentFrame.mpImuPreintegrated)
    {
        twb1 = mCurrentFrame.mpLastKeyFrame->GetImuPosition();
        Rwb1 = mCurrentFrame.mpLastKeyFrame->GetImuRotation();
        Vwb1 = mCurrentFrame.mpLastKeyFrame->GetVelocity();
        t12 = mCurrentFrame.mpImuPreintegrated->dT;

        mCurrentFrame.SetImuPoseVelocity(Rwb1*mCurrentFrame.mpImuPreintegrated->GetUpdatedDeltaRotation(),
                                      twb1 + Vwb1*t12 + 0.5f*t12*t12*Gz+ Rwb1*mCurrentFrame.mpImuPreintegrated->GetUpdatedDeltaPosition(),
                                      Vwb1 + Gz*t12 + Rwb1*mCurrentFrame.mpImuPreintegrated->GetUpdatedDeltaVelocity());
    }

    mnFirstImuFrameId = mCurrentFrame.mnId;
}


cv::Mat Tracking::ComputeF12(KeyFrame *&pKF1, KeyFrame *&pKF2)
{
    cv::Mat R1w = pKF1->GetRotation();
    cv::Mat t1w = pKF1->GetTranslation();
    cv::Mat R2w = pKF2->GetRotation();
    cv::Mat t2w = pKF2->GetTranslation();

    cv::Mat R12 = R1w*R2w.t();
    cv::Mat t12 = -R1w*R2w.t()*t2w+t1w;

    cv::Mat t12x = Converter::tocvSkewMatrix(t12);

    const cv::Mat &K1 = pKF1->mK;
    const cv::Mat &K2 = pKF2->mK;


    return K1.t().inv()*t12x*R12*K2.inv();
}


void Tracking::CreateNewMapPoints()
{
    // Retrieve neighbor keyframes in covisibility graph
    const vector<KeyFrame*> vpKFs = mpAtlas->GetAllKeyFrames();

    ORBmatcher matcher(0.6,false);

    cv::Mat Rcw1 = mpLastKeyFrame->GetRotation();
    cv::Mat Rwc1 = Rcw1.t();
    cv::Mat tcw1 = mpLastKeyFrame->GetTranslation();
    cv::Mat Tcw1(3,4,CV_32F);
    Rcw1.copyTo(Tcw1.colRange(0,3));
    tcw1.copyTo(Tcw1.col(3));
    cv::Mat Ow1 = mpLastKeyFrame->GetCameraCenter();

    const float &fx1 = mpLastKeyFrame->fx;
    const float &fy1 = mpLastKeyFrame->fy;
    const float &cx1 = mpLastKeyFrame->cx;
    const float &cy1 = mpLastKeyFrame->cy;
    const float &invfx1 = mpLastKeyFrame->invfx;
    const float &invfy1 = mpLastKeyFrame->invfy;

    const float ratioFactor = 1.5f*mpLastKeyFrame->mfScaleFactor;

    int nnew=0;

    // Search matches with epipolar restriction and triangulate
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF2 = vpKFs[i];
        if(pKF2==mpLastKeyFrame)
            continue;

        // Check first that baseline is not too short
        cv::Mat Ow2 = pKF2->GetCameraCenter();
        cv::Mat vBaseline = Ow2-Ow1;
        const float baseline = cv::norm(vBaseline);

        if((mSensor!=System::MONOCULAR)||(mSensor!=System::IMU_MONOCULAR))
        {
            if(baseline<pKF2->mb)
            continue;
        }
        else
        {
            const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);
            const float ratioBaselineDepth = baseline/medianDepthKF2;

            if(ratioBaselineDepth<0.01)
                continue;
        }

        // Compute Fundamental Matrix
        cv::Mat F12 = ComputeF12(mpLastKeyFrame,pKF2);

        // Search matches that fullfil epipolar constraint
        vector<pair<size_t,size_t> > vMatchedIndices;
        matcher.SearchForTriangulation(mpLastKeyFrame,pKF2,F12,vMatchedIndices,false);

        cv::Mat Rcw2 = pKF2->GetRotation();
        cv::Mat Rwc2 = Rcw2.t();
        cv::Mat tcw2 = pKF2->GetTranslation();
        cv::Mat Tcw2(3,4,CV_32F);
        Rcw2.copyTo(Tcw2.colRange(0,3));
        tcw2.copyTo(Tcw2.col(3));

        const float &fx2 = pKF2->fx;
        const float &fy2 = pKF2->fy;
        const float &cx2 = pKF2->cx;
        const float &cy2 = pKF2->cy;
        const float &invfx2 = pKF2->invfx;
        const float &invfy2 = pKF2->invfy;

        // Triangulate each match
        const int nmatches = vMatchedIndices.size();
        for(int ikp=0; ikp<nmatches; ikp++)
        {
            const int &idx1 = vMatchedIndices[ikp].first;
            const int &idx2 = vMatchedIndices[ikp].second;

            const cv::KeyPoint &kp1 = mpLastKeyFrame->mvKeysUn[idx1];
            const float kp1_ur=mpLastKeyFrame->mvuRight[idx1];
            bool bStereo1 = kp1_ur>=0;

            const cv::KeyPoint &kp2 = pKF2->mvKeysUn[idx2];
            const float kp2_ur = pKF2->mvuRight[idx2];
            bool bStereo2 = kp2_ur>=0;

            // Check parallax between rays
            cv::Mat xn1 = (cv::Mat_<float>(3,1) << (kp1.pt.x-cx1)*invfx1, (kp1.pt.y-cy1)*invfy1, 1.0);
            cv::Mat xn2 = (cv::Mat_<float>(3,1) << (kp2.pt.x-cx2)*invfx2, (kp2.pt.y-cy2)*invfy2, 1.0);

            cv::Mat ray1 = Rwc1*xn1;
            cv::Mat ray2 = Rwc2*xn2;
            const float cosParallaxRays = ray1.dot(ray2)/(cv::norm(ray1)*cv::norm(ray2));

            float cosParallaxStereo = cosParallaxRays+1;
            float cosParallaxStereo1 = cosParallaxStereo;
            float cosParallaxStereo2 = cosParallaxStereo;

            if(bStereo1)
                cosParallaxStereo1 = cos(2*atan2(mpLastKeyFrame->mb/2,mpLastKeyFrame->mvDepth[idx1]));
            else if(bStereo2)
                cosParallaxStereo2 = cos(2*atan2(pKF2->mb/2,pKF2->mvDepth[idx2]));

            cosParallaxStereo = min(cosParallaxStereo1,cosParallaxStereo2);

            cv::Mat x3D;
            if(cosParallaxRays<cosParallaxStereo && cosParallaxRays>0 && (bStereo1 || bStereo2 || cosParallaxRays<0.9998))
            {
                // Linear Triangulation Method
                cv::Mat A(4,4,CV_32F);
                A.row(0) = xn1.at<float>(0)*Tcw1.row(2)-Tcw1.row(0);
                A.row(1) = xn1.at<float>(1)*Tcw1.row(2)-Tcw1.row(1);
                A.row(2) = xn2.at<float>(0)*Tcw2.row(2)-Tcw2.row(0);
                A.row(3) = xn2.at<float>(1)*Tcw2.row(2)-Tcw2.row(1);

                cv::Mat w,u,vt;
                cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);

                x3D = vt.row(3).t();

                if(x3D.at<float>(3)==0)
                    continue;

                // Euclidean coordinates
                x3D = x3D.rowRange(0,3)/x3D.at<float>(3);

            }
            else if(bStereo1 && cosParallaxStereo1<cosParallaxStereo2)
            {
                x3D = mpLastKeyFrame->UnprojectStereo(idx1);
            }
            else if(bStereo2 && cosParallaxStereo2<cosParallaxStereo1)
            {
                x3D = pKF2->UnprojectStereo(idx2);
            }
            else
                continue; //No stereo and very low parallax

            cv::Mat x3Dt = x3D.t();

            //Check triangulation in front of cameras
            float z1 = Rcw1.row(2).dot(x3Dt)+tcw1.at<float>(2);
            if(z1<=0)
                continue;

            float z2 = Rcw2.row(2).dot(x3Dt)+tcw2.at<float>(2);
            if(z2<=0)
                continue;

            //Check reprojection error in first keyframe
            const float &sigmaSquare1 = mpLastKeyFrame->mvLevelSigma2[kp1.octave];
            const float x1 = Rcw1.row(0).dot(x3Dt)+tcw1.at<float>(0);
            const float y1 = Rcw1.row(1).dot(x3Dt)+tcw1.at<float>(1);
            const float invz1 = 1.0/z1;

            if(!bStereo1)
            {
                float u1 = fx1*x1*invz1+cx1;
                float v1 = fy1*y1*invz1+cy1;
                float errX1 = u1 - kp1.pt.x;
                float errY1 = v1 - kp1.pt.y;
                if((errX1*errX1+errY1*errY1)>5.991*sigmaSquare1)
                    continue;
            }
            else
            {
                float u1 = fx1*x1*invz1+cx1;
                float u1_r = u1 - mpLastKeyFrame->mbf*invz1;
                float v1 = fy1*y1*invz1+cy1;
                float errX1 = u1 - kp1.pt.x;
                float errY1 = v1 - kp1.pt.y;
                float errX1_r = u1_r - kp1_ur;
                if((errX1*errX1+errY1*errY1+errX1_r*errX1_r)>7.8*sigmaSquare1)
                    continue;
            }

            //Check reprojection error in second keyframe
            const float sigmaSquare2 = pKF2->mvLevelSigma2[kp2.octave];
            const float x2 = Rcw2.row(0).dot(x3Dt)+tcw2.at<float>(0);
            const float y2 = Rcw2.row(1).dot(x3Dt)+tcw2.at<float>(1);
            const float invz2 = 1.0/z2;
            if(!bStereo2)
            {
                float u2 = fx2*x2*invz2+cx2;
                float v2 = fy2*y2*invz2+cy2;
                float errX2 = u2 - kp2.pt.x;
                float errY2 = v2 - kp2.pt.y;
                if((errX2*errX2+errY2*errY2)>5.991*sigmaSquare2)
                    continue;
            }
            else
            {
                float u2 = fx2*x2*invz2+cx2;
                float u2_r = u2 - mpLastKeyFrame->mbf*invz2;
                float v2 = fy2*y2*invz2+cy2;
                float errX2 = u2 - kp2.pt.x;
                float errY2 = v2 - kp2.pt.y;
                float errX2_r = u2_r - kp2_ur;
                if((errX2*errX2+errY2*errY2+errX2_r*errX2_r)>7.8*sigmaSquare2)
                    continue;
            }

            //Check scale consistency
            cv::Mat normal1 = x3D-Ow1;
            float dist1 = cv::norm(normal1);

            cv::Mat normal2 = x3D-Ow2;
            float dist2 = cv::norm(normal2);

            if(dist1==0 || dist2==0)
                continue;

            const float ratioDist = dist2/dist1;
            const float ratioOctave = mpLastKeyFrame->mvScaleFactors[kp1.octave]/pKF2->mvScaleFactors[kp2.octave];

            if(ratioDist*ratioFactor<ratioOctave || ratioDist>ratioOctave*ratioFactor)
                continue;

            // Triangulation is succesfull
            MapPoint* pMP = new MapPoint(x3D,mpLastKeyFrame,mpAtlas->GetCurrentMap());

            pMP->AddObservation(mpLastKeyFrame,idx1);
            pMP->AddObservation(pKF2,idx2);

            mpLastKeyFrame->AddMapPoint(pMP,idx1);
            pKF2->AddMapPoint(pMP,idx2);

            pMP->ComputeDistinctiveDescriptors();

            pMP->UpdateNormalAndDepth();

            mpAtlas->AddMapPoint(pMP);
            nnew++;
        }
    }
    TrackReferenceKeyFrame();
}

void Tracking::NewDataset()
{
    mnNumDataset++;
}

int Tracking::GetNumberDataset()
{
    return mnNumDataset;
}

int Tracking::GetMatchesInliers()
{
    return mnMatchesInliers;
}
//myplan 新增函数
// BRIEF [EAO] associate objects with points.
void Tracking::AssociateObjAndPoints(vector<Object_2D *> objs_2d)
    {
        const cv::Mat Rcw = mCurrentFrame.mTcw.rowRange(0, 3).colRange(0, 3);
        const cv::Mat tcw = mCurrentFrame.mTcw.rowRange(0, 3).col(3);

        for (int i = 0; i < mCurrentFrame.N; i++)
        {
            if (mCurrentFrame.mvpMapPoints[i])
            {
                MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];

                if (!pMP->isBad())
                {
                    for (size_t k = 0; k < objs_2d.size(); ++k)
                    {
                        if (objs_2d[k]->mBoxRect.contains(mCurrentFrame.mvKeysUn[i].pt))// in rect.
                        {
                            cv::Mat PointPosWorld = pMP->GetWorldPos();                 // world frame.
                            cv::Mat PointPosCamera = Rcw * PointPosWorld + tcw;         // camera frame.

                            pMP->object_view = true;                  // the point is associated with an object.
                            pMP->frame_id.insert(mCurrentFrame.mnId); // no use.
                            pMP->feature = mCurrentFrame.mvKeysUn[i]; // coordinate in current frame.

                            // object points.
                            objs_2d[k]->Obj_c_MapPonits.push_back(pMP);

                            // summation the position of points.
                            objs_2d[k]->sum_pos_3d += PointPosWorld;
                        }
                    }
                }
            }
        }
    } // AssociateObjAndPoints() END -----------------------------------

// BRIEF [EAO] associate objects with lines.
//myplan 新增函数
void Tracking::AssociateObjAndLines(vector<Object_2D *> objs_2d)
    {
        // all lines in current frame.
        Eigen::MatrixXd AllLinesEigen = mCurrentFrame.all_lines_eigen;

        // step 1 make sure edges start from left to right.
        align_left_right_edges(AllLinesEigen);

        for(int i = 0; i < objs_2d.size(); i++)
        {
            Object_2D* obj = objs_2d[i];

            // step 2. expand the bounding box.
            double dLeftExpand = max(0.0, obj->mBox.x - 15.0);
            double dRightExpand = min(mCurrentFrame.mColorImage.cols, obj->mBox.x + obj->mBox.width + 15);
            double dTopExpand = max(0.0, obj->mBox.y - 15.0);
            double dBottomExpand = min(mCurrentFrame.mColorImage.rows, obj->mBox.y + obj->mBox.height + 15);
            Vector2d ExpanLeftTop = Vector2d(dLeftExpand, dTopExpand);			// lefttop.
            Vector2d ExpanRightBottom = Vector2d(dRightExpand, dBottomExpand);  // rightbottom.

            // step 3. associate object with lines.
            Eigen::MatrixXd ObjectLines(AllLinesEigen.rows(),AllLinesEigen.cols());
            int nInsideLinesNum = 0;
            for (int line_id = 0; line_id < AllLinesEigen.rows(); line_id++)
            {
                // check endpoints of the lines, whether inside the box.
                if (check_inside_box(   AllLinesEigen.row(line_id).head<2>(),
                                        ExpanLeftTop,
                                        ExpanRightBottom ))
                {
                    if(check_inside_box(AllLinesEigen.row(line_id).tail<2>(),
                                        ExpanLeftTop,
                                        ExpanRightBottom ))
                    {
                        ObjectLines.row(nInsideLinesNum) = AllLinesEigen.row(line_id);
                        nInsideLinesNum++;
                    }
                }
            }

            // step 4. merge lines.
            double pre_merge_dist_thre = 20;
            double pre_merge_angle_thre = 5;
            double edge_length_threshold = 30;
            MatrixXd ObjectLinesAfterMerge;
            merge_break_lines(	ObjectLines.topRows(nInsideLinesNum),
                                  ObjectLinesAfterMerge, 		// output lines after merge.
                                  pre_merge_dist_thre,		// the distance threshold between two line, 20 pixels.
                                  pre_merge_angle_thre, 		// angle threshold between two line, 5°.
                                  edge_length_threshold);		// length threshold, 30 pixels.

            // step 5. save object lines.
            obj->mObjLinesEigen = ObjectLinesAfterMerge;
            mCurrentFrame.vObjsLines.push_back(ObjectLinesAfterMerge);
        }
    } // AssociateObjAndLines() END ----------------------------------.
// BRIEF [EAO] Initialize the object map.
//myplan 新增函数
void Tracking::InitObjMap(vector<Object_2D *> objs_2d)
{
    int nGoodObjId = -1;        // object id.
    for (auto &obj : objs_2d)
    {
        // Initialize the object map need enough points.
        if (obj->Obj_c_MapPonits.size() < 10)
        {
            obj->few_mappoint = true;
            obj->current = false;
            continue;
        }

        nGoodObjId++;

        mbObjectIni = true;
        mnObjectIniFrameID = mCurrentFrame.mnId;

        // Create an object in the map.
        Object_Map *ObjectMapSingle = new Object_Map;
        ObjectMapSingle->mObjectFrame.push_back(obj);   // 2D objects in each frame associated with this 3D map object.
        ObjectMapSingle->mnId = nGoodObjId;             // 3d objects in the map.
        ObjectMapSingle->mnClass = obj->_class_id;      // object class.
        ObjectMapSingle->mnConfidence = 1;              // object confidence = mObjectFrame.size().
        ObjectMapSingle->mbFirstObserve = true;                 // the object was observed for the first time.
        ObjectMapSingle->mnAddedID = mCurrentFrame.mnId;        // added id.
        ObjectMapSingle->mnLastAddID = mCurrentFrame.mnId;      // last added id.
        ObjectMapSingle->mnLastLastAddID = mCurrentFrame.mnId;  // last last added id.
        ObjectMapSingle->mLastRect = obj->mBoxRect;             // last rect.
        // ObjectMapSingle->mPredictRect = obj->mBoxRect;       // for iou.
        ObjectMapSingle->msFrameId.insert(mCurrentFrame.mnId);  // no used in this version.
        ObjectMapSingle->mSumPointsPos = obj->sum_pos_3d;       // accumulated coordinates of object points.
        ObjectMapSingle->mCenter3D = obj->_Pos;                 // 3d centre.
        obj->mAssMapObjCenter = obj->_Pos;                      // for optimization, no used in this version.

        // add properties of the point and save it to the object.
        for (size_t i = 0; i < obj->Obj_c_MapPonits.size(); i++)
        {
            MapPoint *pMP = obj->Obj_c_MapPonits[i];

            pMP->object_id = ObjectMapSingle->mnId;
            pMP->object_class = ObjectMapSingle->mnClass;
            pMP->object_id_vector.insert(make_pair(ObjectMapSingle->mnId, 1)); // the point is first observed by the object.

            if (ObjectMapSingle->mbFirstObserve == true)
                pMP->First_obj_view = true;

            // save to the object.
            ObjectMapSingle->mvpMapObjectMappoints.push_back(pMP);
        }

        // 2d object.
        obj->mnId = ObjectMapSingle->mnId;
        obj->mnWhichTime = ObjectMapSingle->mnConfidence;
        obj->current = true;

        // save this 2d object to current frame (associates with a 3d object in the map).
        mCurrentFrame.mvObjectFrame.push_back(obj);
        //mCurrentFrame.mvLastObjectFrame.push_back(obj);
        //mCurrentFrame.mvLastLastObjectFrame.push_back(obj);

        // todo: save to key frame.

        // updata map.
        ObjectMapSingle->ComputeMeanAndStandard();
        mpAtlas->mvObjectMap.push_back(ObjectMapSingle);
    }
} // initialize the object map. END -----------------------------------------------------
// BRIEF [EAO] Estimate object orientation.
//myplan 新增函数
void Tracking::SampleObjYaw(Object_Map* objMap)
{
    // demo 1: compare the results without estimating the orientation.
    if((mflag == "None") || (mflag == "iForest"))
        return;

    int numMax = 0;
    float fError = 0.0;
    float fErrorYaw;
    float minErrorYaw = 360.0;
    float sampleYaw = 0.0;
    int nAllLineNum = objMap->mObjectFrame.back()->mObjLinesEigen.rows();

    for(int i = 0; i < 30; i++) {
        // initial angle.
        float roll, pitch, yaw;
        roll = 0.0;
        pitch = 0.0;
        yaw = 0.0;
        float error = 0.0;
        float errorYaw = 0.0;

        // 1 -> 15: -45° - 0°
        // 16 -> 30: 0° - 45°
        if (i < 15)
            yaw = (0.0 - i * 3.0) / 180.0 * M_PI;
        else
            yaw = (0.0 + (i - 15) * 3.0) / 180.0 * M_PI;

        // object pose in object frame. (Ryaw)
        float cp = cos(pitch);
        float sp = sin(pitch);
        float sr = sin(roll);
        float cr = cos(roll);
        float sy = sin(yaw);
        float cy = cos(yaw);
        Eigen::Matrix<double, 3, 3> REigen;


//**************************************************************************
////个矩阵将用来存储 good_points 中的点的坐标。
//        cv::Mat pts1 = objMap->mSumPointsPos.clone(); // 创建pts的一个副本，以免修改原始数据
//        //个矩阵将用来存储 good_points 中的点的坐标。
//        Eigen::Vector3d pts2 = Converter::toEigenMatrixXd(pts1);
//        std::vector<Eigen::Vector3d> good_points;
//        for(int i = 0; i < pts1.rows; ++i) {
//            cv::Mat row = pts1.row(i);
//            Eigen::Vector3d pt = Eigen::Vector3d(row.at<float>(0), row.at<float>(1), row.at<float>(2));
//            good_points.push_back(pt);
//        }
//        //个矩阵将用来存储 所有 good_points 中的点的坐标。
//        Eigen::MatrixX3d pts(good_points.size(), 3);
//        //向量将用来计算 good_points 中的点的中心坐标。
//        Eigen::Vector3d center(0, 0, 0);
//        for (size_t i = 0; i < good_points.size(); ++i) {
//            pts.row(i) = good_points[i].transpose();
//            center += good_points[i].transpose();
//        }
//        center /= pts.rows();
//        //算 pts 中每个点的坐标相对于中心的坐标
//        Eigen::MatrixX3d pts_centered = pts.rowwise() - center.transpose();
//        //计算 pts_centered 的协方差矩
//        Eigen::Matrix3d M = pts_centered.transpose() * pts_centered;
//        //，以得到平均协方差
//        M /= pts_centered.cols();
////        std::cout << "M\n" << M << "\n";
//        //对协方差矩阵 M 进行特征值分解。这是 PCA 的关键步骤，
//        // 可以得到数据的主要变化方向，这些方向将作为椭球的主轴。
//        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(M);
//        //mynote 旋转矩阵
//        Eigen::Matrix3d R = solver.eigenvectors();
//        cv::Mat Ryaw = Converter::toCvMat(R);
//   ***********************************************************************
        //myplan 注释了这里

        REigen << cp * cy, (sr * sp * cy) - (cr * sy), (cr * sp * cy) + (sr * sy),
                cp * sy, (sr * sp * sy) + (cr * cy), (cr * sp * sy) - (sr * cy),
                -sp, sr * cp, cr * cp;
        cv::Mat Ryaw = Converter::toCvMat(REigen);

        // 8 vertices of the 3D box, world --> object frame.
        cv::Mat corner_1 =
                Converter::toCvMat(objMap->mCuboid3D.corner_1_w) - Converter::toCvMat(objMap->mCuboid3D.cuboidCenter);
        cv::Mat corner_2 =
                Converter::toCvMat(objMap->mCuboid3D.corner_2_w) - Converter::toCvMat(objMap->mCuboid3D.cuboidCenter);
        cv::Mat corner_3 =
                Converter::toCvMat(objMap->mCuboid3D.corner_3_w) - Converter::toCvMat(objMap->mCuboid3D.cuboidCenter);
        cv::Mat corner_4 =
                Converter::toCvMat(objMap->mCuboid3D.corner_4_w) - Converter::toCvMat(objMap->mCuboid3D.cuboidCenter);
        cv::Mat corner_5 =
                Converter::toCvMat(objMap->mCuboid3D.corner_5_w) - Converter::toCvMat(objMap->mCuboid3D.cuboidCenter);
        cv::Mat corner_6 =
                Converter::toCvMat(objMap->mCuboid3D.corner_6_w) - Converter::toCvMat(objMap->mCuboid3D.cuboidCenter);
        cv::Mat corner_7 =
                Converter::toCvMat(objMap->mCuboid3D.corner_7_w) - Converter::toCvMat(objMap->mCuboid3D.cuboidCenter);
        cv::Mat corner_8 =
                Converter::toCvMat(objMap->mCuboid3D.corner_8_w) - Converter::toCvMat(objMap->mCuboid3D.cuboidCenter);

        // rotate in object frame  + object frame --> world frame.
        corner_1 = Ryaw * corner_1 + Converter::toCvMat(objMap->mCuboid3D.cuboidCenter);
        corner_2 = Ryaw * corner_2 + Converter::toCvMat(objMap->mCuboid3D.cuboidCenter);
        corner_3 = Ryaw * corner_3 + Converter::toCvMat(objMap->mCuboid3D.cuboidCenter);
        corner_4 = Ryaw * corner_4 + Converter::toCvMat(objMap->mCuboid3D.cuboidCenter);
        corner_5 = Ryaw * corner_5 + Converter::toCvMat(objMap->mCuboid3D.cuboidCenter);
        corner_6 = Ryaw * corner_6 + Converter::toCvMat(objMap->mCuboid3D.cuboidCenter);
        corner_7 = Ryaw * corner_7 + Converter::toCvMat(objMap->mCuboid3D.cuboidCenter);
        corner_8 = Ryaw * corner_8 + Converter::toCvMat(objMap->mCuboid3D.cuboidCenter);

        // step 1. project 8 vertices to image.
        cv::Point2f point1, point2, point3, point4, point5, point6, point7, point8;
        point1 = WorldToImg(corner_1);
        point2 = WorldToImg(corner_2);
        point3 = WorldToImg(corner_3);
        point4 = WorldToImg(corner_4);
        point5 = WorldToImg(corner_5);
        point6 = WorldToImg(corner_6);
        point7 = WorldToImg(corner_7);
        point8 = WorldToImg(corner_8);

        // step 2. angle of 3 edges(lenth, width, height).
        float angle1;
        float angle2;
        float angle3;
        // left -> right.
        if (point6.x > point5.x)
            angle1 = atan2(point6.y - point5.y, point6.x - point5.x);
        else
            angle1 = atan2(point5.y - point6.y, point5.x - point6.x);
        float lenth1 = sqrt(
                (point6.y - point5.y) * (point6.y - point5.y) + (point6.x - point5.x) * (point6.x - point5.x));

        if (point7.x > point6.x)
            angle2 = atan2(point7.y - point6.y, point7.x - point6.x);
        else
            angle2 = atan2(point6.y - point7.y, point6.x - point7.x);
        float lenth2 = sqrt(
                (point7.y - point6.y) * (point7.y - point6.y) + (point7.x - point6.x) * (point7.x - point6.x));

        if (point6.x > point2.x)
            angle3 = atan2(point6.y - point2.y, point6.x - point2.x);
        else
            angle3 = atan2(point2.y - point6.y, point2.x - point6.x);
        float lenth3 = sqrt(
                (point6.y - point2.y) * (point6.y - point2.y) + (point6.x - point2.x) * (point6.x - point2.x));

        // step 3. compute angle between detected lines and cube edges.
        int num = 0;
//IMP 修改了这里
        //****************************************************************************************
//        std::vector < ORB_SLAM3::Plane * > planes;
//        std::vector < Plane * > all_landmark_planes = mpAtlas->get_all_landmark_planes();
//        std::vector <Vector3d> all_plane_normals;
//        for (auto plane: all_landmark_planes) {
//
//            Vector3d plane_norm = plane->get_normal();//平面的法向量
//            all_plane_normals.push_back(plane_norm);
//        }
//        for (auto all_plane_normal: all_plane_normals)
//        {
//            float angle = atan2(all_plane_normal[1], all_plane_normal[0]);//每个平面的角度

        //****************************************************************************************

            for (int line_id = 0; line_id < objMap->mObjectFrame.back()->mObjLinesEigen.rows(); line_id++) {
        //            // angle of detected lines.
                double x1 = objMap->mObjectFrame.back()->mObjLinesEigen(line_id, 0);
                double y1 = objMap->mObjectFrame.back()->mObjLinesEigen(line_id, 1);
                double x2 = objMap->mObjectFrame.back()->mObjLinesEigen(line_id, 2);
                double y2 = objMap->mObjectFrame.back()->mObjLinesEigen(line_id, 3);
                    float angle = atan2(y2 - y1, x2 - x1);//返回线的弧度

                // lenth.
                float lenth = sqrt((y2 - y1) * (y2 - y1) + (x2 - x1) * (x2 - x1));

                // angle between line and 3 edges.
                float dis_angle1 = abs(angle * 180 / M_PI - angle1 * 180 / M_PI);
                float dis_angle2 = abs(angle * 180 / M_PI - angle2 * 180 / M_PI);
                float dis_angle3 = abs(angle * 180 / M_PI - angle3 * 180 / M_PI);

                float th = 5.0;             // threshold of the angle.
                if (objMap->mnClass == 56)   // chair.
                {
                    if ((dis_angle2 < th) || (dis_angle3 < th))
                        num++;
                    if (dis_angle1 < th) {
                        num += 3;
                    }
                } else {
                    // the shortest edge is lenth1.
                    if (min(min(lenth1, lenth2), lenth3) == lenth1) {
                        // error with other two edges.
                        if ((dis_angle2 < th) || (dis_angle3 < th)) {
                            num++;
                            if (dis_angle2 < th)
                                error += dis_angle2;
                            if (dis_angle3 < th)
                                error += dis_angle3;
                        }

                        // angle error.
                        errorYaw += min(dis_angle2, dis_angle3);
                    }
                    // the shortest edge is lenth2.
                    if (min(min(lenth1, lenth2), lenth3) == lenth2) {
                        if ((dis_angle1 < th) || (dis_angle3 < th)) {
                            num++;
                            if (dis_angle1 < th)
                                error += dis_angle1;
                            if (dis_angle3 < th)
                                error += dis_angle3;
                        }
                        errorYaw += min(dis_angle3, dis_angle1);
                    }
                    // the shortest edge is lenth3.
                    if (min(min(lenth1, lenth2), lenth3) == lenth3) {
                        if ((dis_angle1 < th) || (dis_angle2 < th)) {
                            num++;
                            if (dis_angle1 < th)
                                error += dis_angle1;
                            if (dis_angle2 < th)
                                error += dis_angle2;
                        }
                        errorYaw += min(dis_angle2, dis_angle1);
                }
            }
        }
//    }
        if(num == 0)
        {
            num = 1;
            errorYaw = 10.0;
        }

        // record the angle with max number parallel lines.
        if(num > numMax)
        {
            numMax = num;
            sampleYaw = yaw;

            fError = error; // no used in this version.
            // average angle error.
            fErrorYaw = (errorYaw/(float)num)/10.0;
        }
    }

    // step 4. scoring.
    float fScore;
    fScore = ((float)numMax / (float)nAllLineNum) * (1.0 - 0.1 * fErrorYaw);
    if(isinf(fScore))
        fScore = 0.0;

    // measurement： yaw, times, score, angle, angle error.
    Vector5f AngleTimesAndScore;
    AngleTimesAndScore[0] = sampleYaw;
    AngleTimesAndScore[1] = 1.0;
    AngleTimesAndScore[2] = fScore;
    AngleTimesAndScore[3] = fError;     // no used in this version.
    AngleTimesAndScore[4] = fErrorYaw;

    // update multi-frame measurement.
    bool bNewMeasure = true;
    for (auto &row : objMap->mvAngleTimesAndScore)
    {
        if(row[0] == AngleTimesAndScore[0])
        {
            row[1] += 1.0;
            row[2] = AngleTimesAndScore[2] * (1/row[1]) + row[2] * (1 - 1/row[1]);
            row[3] = AngleTimesAndScore[3] * (1/row[1]) + row[3] * (1 - 1/row[1]);
            row[4] = AngleTimesAndScore[4] * (1/row[1]) + row[4] * (1 - 1/row[1]);

            bNewMeasure = false;
        }
    }
    if(bNewMeasure == true)
    {
        objMap->mvAngleTimesAndScore.push_back(AngleTimesAndScore);
    }

    // step 5. rank.
    index_eao = 1;
    std::sort(objMap->mvAngleTimesAndScore.begin(),objMap->mvAngleTimesAndScore.end(),VIC);
    // for (auto &row : objMap->mvAngleTimesAndScore)
    // {
    //     std::cout << row[0] * 180.0 / M_PI  << "\t" <<  row[1] << "\t" <<  row[2] << std::endl;
    // }
    // the best yaw.
    int best_num = 0;
    float best_score = 0;
    for(int i = 0; i < std::min(3, (int)objMap->mvAngleTimesAndScore.size()); i++)
    {
        float fScore = objMap->mvAngleTimesAndScore[i][2];
        if(fScore >= best_score)
        {
            best_score = fScore;
            best_num = i;
        }
    }

    // step 6. update object yaw.//0 3 4
    objMap->mCuboid3D.rotY = objMap->mvAngleTimesAndScore[best_num][0];
    objMap->mCuboid3D.mfErrorParallel = objMap->mvAngleTimesAndScore[best_num][3];
    objMap->mCuboid3D.mfErroeYaw = objMap->mvAngleTimesAndScore[best_num][4];
} // SampleObjYaw() END -------------------------------------------------------------------------

// BRIEF [EAO] project quadrics from world to image.
////myplan 新增函数
cv::Mat Tracking::DrawQuadricProject(cv::Mat &im,
                                     const cv::Mat &P,   // projection matrix.
                                     const cv::Mat &axe, // axis length.
                                     const cv::Mat &Twq, // object pose.
                                     int nClassid,
                                     bool isGT,
                                     int nLatitudeNum,
                                     int nLongitudeNum)
{
    // color.
    std::vector<cv::Scalar> colors = {  cv::Scalar(135,0,248),
                                        cv::Scalar(255,0,253),
                                        cv::Scalar(4,254,119),
                                        cv::Scalar(255,126,1),
                                        cv::Scalar(0,112,255),
                                        cv::Scalar(0,250,250),
    };

    // draw params
    cv::Scalar sc = colors[nClassid % 6];

    int nLineWidth = 2;

    // generate angluar grid -> xyz grid (vertical half sphere)
    vector<float> vfAngularLatitude;  // (-90, 90)
    vector<float> vfAngularLongitude; // [0, 180]
    cv::Mat pointGrid(nLatitudeNum + 2, nLongitudeNum + 1, CV_32FC4);

    for (int i = 0; i < nLatitudeNum + 2; i++)
    {
        float fThetaLatitude = -M_PI_2 + i * M_PI / (nLatitudeNum + 1);
        cv::Vec4f *p = pointGrid.ptr<cv::Vec4f>(i);
        for (int j = 0; j < nLongitudeNum + 1; j++)
        {
            float fThetaLongitude = j * M_PI / nLongitudeNum;
            p[j][0] = axe.at<float>(0, 0) * cos(fThetaLatitude) * cos(fThetaLongitude);
            p[j][1] = axe.at<float>(1, 0) * cos(fThetaLatitude) * sin(fThetaLongitude);
            p[j][2] = axe.at<float>(2, 0) * sin(fThetaLatitude);
            p[j][3] = 1.;
        }
    }

    // draw latitude
    for (int i = 0; i < pointGrid.rows; i++)
    {
        cv::Vec4f *p = pointGrid.ptr<cv::Vec4f>(i);
        // [0, 180]
        for (int j = 0; j < pointGrid.cols - 1; j++)
        {
            cv::Mat spherePt0 = (cv::Mat_<float>(4, 1) << p[j][0], p[j][1], p[j][2], p[j][3]);
            cv::Mat spherePt1 = (cv::Mat_<float>(4, 1) << p[j + 1][0], p[j + 1][1], p[j + 1][2], p[j + 1][3]);
            cv::Mat conicPt0 = P * Twq * spherePt0;
            cv::Mat conicPt1 = P * Twq * spherePt1;
            cv::Point pt0(conicPt0.at<float>(0, 0) / conicPt0.at<float>(2, 0), conicPt0.at<float>(1, 0) / conicPt0.at<float>(2, 0));
            cv::Point pt1(conicPt1.at<float>(0, 0) / conicPt1.at<float>(2, 0), conicPt1.at<float>(1, 0) / conicPt1.at<float>(2, 0));
            cv::line(im, pt0, pt1, sc, nLineWidth); // [0, 180]
        }
        // [180, 360]
        for (int j = 0; j < pointGrid.cols - 1; j++)
        {
            cv::Mat spherePt0 = (cv::Mat_<float>(4, 1) << -p[j][0], -p[j][1], p[j][2], p[j][3]);
            cv::Mat spherePt1 = (cv::Mat_<float>(4, 1) << -p[j + 1][0], -p[j + 1][1], p[j + 1][2], p[j + 1][3]);
            cv::Mat conicPt0 = P * Twq * spherePt0;
            cv::Mat conicPt1 = P * Twq * spherePt1;
            cv::Point pt0(conicPt0.at<float>(0, 0) / conicPt0.at<float>(2, 0), conicPt0.at<float>(1, 0) / conicPt0.at<float>(2, 0));
            cv::Point pt1(conicPt1.at<float>(0, 0) / conicPt1.at<float>(2, 0), conicPt1.at<float>(1, 0) / conicPt1.at<float>(2, 0));
            cv::line(im, pt0, pt1, sc, nLineWidth); // [180, 360]
        }
    }

    // draw longitude
    cv::Mat pointGrid_t = pointGrid.t();
    for (int i = 0; i < pointGrid_t.rows; i++)
    {
        cv::Vec4f *p = pointGrid_t.ptr<cv::Vec4f>(i);
        // [0, 180]
        for (int j = 0; j < pointGrid_t.cols - 1; j++)
        {
            cv::Mat spherePt0 = (cv::Mat_<float>(4, 1) << p[j][0], p[j][1], p[j][2], p[j][3]);
            cv::Mat spherePt1 = (cv::Mat_<float>(4, 1) << p[j + 1][0], p[j + 1][1], p[j + 1][2], p[j + 1][3]);
            cv::Mat conicPt0 = P * Twq * spherePt0;
            cv::Mat conicPt1 = P * Twq * spherePt1;
            cv::Point pt0(conicPt0.at<float>(0, 0) / conicPt0.at<float>(2, 0), conicPt0.at<float>(1, 0) / conicPt0.at<float>(2, 0));
            cv::Point pt1(conicPt1.at<float>(0, 0) / conicPt1.at<float>(2, 0), conicPt1.at<float>(1, 0) / conicPt1.at<float>(2, 0));
            cv::line(im, pt0, pt1, sc, nLineWidth); // [0, 180]
        }
        // [180, 360]
        for (int j = 0; j < pointGrid_t.cols - 1; j++)
        {
            cv::Mat spherePt0 = (cv::Mat_<float>(4, 1) << -p[j][0], -p[j][1], p[j][2], p[j][3]);
            cv::Mat spherePt1 = (cv::Mat_<float>(4, 1) << -p[j + 1][0], -p[j + 1][1], p[j + 1][2], p[j + 1][3]);
            cv::Mat conicPt0 = P * Twq * spherePt0;
            cv::Mat conicPt1 = P * Twq * spherePt1;
            cv::Point pt0(conicPt0.at<float>(0, 0) / conicPt0.at<float>(2, 0), conicPt0.at<float>(1, 0) / conicPt0.at<float>(2, 0));
            cv::Point pt1(conicPt1.at<float>(0, 0) / conicPt1.at<float>(2, 0), conicPt1.at<float>(1, 0) / conicPt1.at<float>(2, 0));
            cv::line(im, pt0, pt1, sc, nLineWidth); // [180, 360]
        }
    }

    return im;
} // DrawQuadricProject() END  -----------------------------------------------------------------------------------------------------

// BRIEF [EAO] project points to image.
//myplan 新增函数
cv::Point2f Tracking::WorldToImg(cv::Mat &PointPosWorld)
{
    // world.
    const cv::Mat Rcw = mCurrentFrame.mTcw.rowRange(0, 3).colRange(0, 3);
    const cv::Mat tcw = mCurrentFrame.mTcw.rowRange(0, 3).col(3);

    // camera.
    cv::Mat PointPosCamera = Rcw * PointPosWorld + tcw;

    const float xc = PointPosCamera.at<float>(0);
    const float yc = PointPosCamera.at<float>(1);
    const float invzc = 1.0 / PointPosCamera.at<float>(2);

    // image.
    float u = mCurrentFrame.fx * xc * invzc + mCurrentFrame.cx;
    float v = mCurrentFrame.fy * yc * invzc + mCurrentFrame.cy;

    return cv::Point2f(u, v);
} // WorldToImg(cv::Mat &PointPosWorld) END ------------------------------

void Tracking::printMatrix(const std::vector<std::vector<double>>& matrix) {
    for (int i = 0; i < matrix.size(); i++) {
        for (int j = 0; j < matrix[i].size(); j++) {
            std::cout << matrix[i][j] << " ";
        }
        std::cout << std::endl;  // line break for each row
    }
}
void Tracking::get_plane_norm()
{
    t_all_plane_normals=mpMapDrawer->all_plane_normals;
}


} //namespace ORB_SLAM
