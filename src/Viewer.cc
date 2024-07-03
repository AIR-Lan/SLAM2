


#include "Viewer.h"
#include <pangolin/pangolin.h>

#include <mutex>

#include <fstream>

namespace ORB_SLAM3
{

Viewer::Viewer(System* pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Tracking *pTracking, const string &strSettingPath):
    both(false), mpSystem(pSystem), mpFrameDrawer(pFrameDrawer),mpMapDrawer(pMapDrawer), mpTracker(pTracking),
    mbFinishRequested(false), mbFinished(true), mbStopped(true), mbStopRequested(false)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    bool is_correct = ParseViewerParamFile(fSettings);

    if(!is_correct)
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

    mbStopTrack = false;
}

bool Viewer::ParseViewerParamFile(cv::FileStorage &fSettings)
{
    bool b_miss_params = false;

    float fps = fSettings["Camera.fps"];
    if(fps<1)
        fps=30;
    mT = 1e3/fps;

    cv::FileNode node = fSettings["Camera.width"];
    if(!node.empty())
    {
        mImageWidth = node.real();
    }
    else
    {
        std::cerr << "*Camera.width parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Camera.height"];
    if(!node.empty())
    {
        mImageHeight = node.real();
    }
    else
    {
        std::cerr << "*Camera.height parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.ViewpointX"];
    if(!node.empty())
    {
        mViewpointX = node.real();
    }
    else
    {
        std::cerr << "*Viewer.ViewpointX parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.ViewpointY"];
    if(!node.empty())
    {
        mViewpointY = node.real();
    }
    else
    {
        std::cerr << "*Viewer.ViewpointY parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.ViewpointZ"];
    if(!node.empty())
    {
        mViewpointZ = node.real();
    }
    else
    {
        std::cerr << "*Viewer.ViewpointZ parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.ViewpointF"];
    if(!node.empty())
    {
        mViewpointF = node.real();
    }
    else
    {
        std::cerr << "*Viewer.ViewpointF parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    return !b_miss_params;
}

void Viewer::Run()
{
    mbFinished = false;
    mbStopped = false;
    showpoints = true;

    if(mpTracker->mSensor == mpSystem->STEREO|| mpTracker->mSensor == mpSystem->RGBD)
    {
        if(mpTracker->SLAM!=0)
            showpoints=false;
    }



    pangolin::CreateWindowAndBind("SPL-SLAM: Map Viewer",1024,768);
    cv::Mat image(768, 1024, CV_8UC3);//创建存储map的mat

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",true,true);
    pangolin::Var<bool> menuCamView("menu.Camera View",false,false);
    pangolin::Var<bool> menuTopView("menu.Top View",false,false);
    // pangolin::Var<bool> menuSideView("menu.Side View",false,false);
    pangolin::Var<bool> IsYoloObject("menu.IsYoloObject",true,true);//change yolo
    pangolin::Var<bool> menuShowPoints("menu.Show Points",showpoints,true);
    pangolin::Var<bool> menuShowLines("menu.Show Lines",true,true);//myline 显示线
    pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);
    pangolin::Var<bool> menuShowGraph("menu.Show Graph",false,true);
    pangolin::Var<bool> menuShowInertialGraph("menu.Show Inertial Graph",true,true);
    pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode",false,true);
    pangolin::Var<bool> menuReset("menu.Reset",false,false);
    pangolin::Var<bool> menuStepByStep("menu.Step By Step",false,true);  // false, true
    pangolin::Var<bool> menuStep("menu.Step",false,false);

    pangolin::Var<bool> menuShowSemiDense("menu.Show SemiDense",true,true);
    pangolin::Var<bool> menuShowModel("menu.Show Model", false,true);
    pangolin::Var<bool> menuShowTexture("menu.Show Texture", false,true);
    pangolin::Var<bool> menuShowCubeObj("menu.Show CubeObj",true,true);
    pangolin::Var<bool> menuShowQuadricObj("menu.Show QuadricObj",true,true);

    pangolin::Var<bool> menuShowBottle("menu.Show Bottles",true,true);
    pangolin::Var<bool> menuShowChair("menu.Show Chairs",true,true);
    pangolin::Var<bool> menuShowTvmonitors("menu.Show Tvmonitors",true,true);
    pangolin::Var<bool> menuShowKeyboard("menu.Show Keyboard",true,true);
    pangolin::Var<bool> menuShowMouse("menu.Show Mouse",true,true);
    pangolin::Var<bool> menuShowBook("menu.Show Books",true,true);
    pangolin::Var<bool> menuShowBear("menu.Show Bear",true,true);
    pangolin::Var<double> menuSigmaTH("menu.Sigma",0.02,1e-10,0.05,false);
    //myplan
    pangolin::Var<bool> menuShowPlanes("menu.Show Planes", true, true);

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,1000),
                pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0)
                );

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    pangolin::OpenGlMatrix Twc, Twr;
    Twc.SetIdentity();
    pangolin::OpenGlMatrix Ow; // Oriented with g in the z axis
    Ow.SetIdentity();
    pangolin::OpenGlMatrix Twwp; // Oriented with g in the z axis, but y and x from camera
    Twwp.SetIdentity();
    cv::namedWindow("SPL-SLAM: Current Frame");

    bool bFollow = true;
    bool bLocalizationMode = false;
    bool bStepByStep = false;
    bool bCameraView = true;

    if(mpTracker->mSensor == mpSystem->MONOCULAR || mpTracker->mSensor == mpSystem->STEREO || mpTracker->mSensor == mpSystem->RGBD)
    {
        menuShowGraph = true;
    }

    while(1)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);//// ... 渲染SPL-SLAM Viewer的内容


        mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc,Ow,Twwp);

        if(mbStopTrack)
        {
            menuStepByStep = true;
            mbStopTrack = false;
        }

        if(menuFollowCamera && bFollow)
        {
            if(bCameraView)
                s_cam.Follow(Twc);
            else
                s_cam.Follow(Ow);
        }
        else if(menuFollowCamera && !bFollow)
        {
            if(bCameraView)
            {
                s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,1000));
                s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0));
                s_cam.Follow(Twc);
            }
            else
            {
                s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(1024,768,3000,3000,512,389,0.1,1000));
                s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(0,0.01,10, 0,0,0,0.0,0.0, 1.0));
                s_cam.Follow(Ow);
            }
            bFollow = true;
        }
        else if(!menuFollowCamera && bFollow)
        {
            bFollow = false;
        }

        if(menuCamView)
        {
            menuCamView = false;
            bCameraView = true;
            s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,10000));
            s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0));
            s_cam.Follow(Twc);
        }

        if(menuTopView && mpMapDrawer->mpAtlas->isImuInitialized())
        {
            menuTopView = false;
            bCameraView = false;
            /*s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(1024,768,3000,3000,512,389,0.1,1000));
            s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(0,0.01,10, 0,0,0,0.0,0.0, 1.0));*/
            s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(1024,768,3000,3000,512,389,0.1,10000));
            s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(0,0.01,50, 0,0,0,0.0,0.0, 1.0));
            s_cam.Follow(Ow);
        }

        /*if(menuSideView && mpMapDrawer->mpAtlas->isImuInitialized())
        {
            s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(1024,768,3000,3000,512,389,0.1,10000));
            s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(0.0,0.1,30.0,0,0,0,0.0,0.0,1.0));
            s_cam.Follow(Twwp);
        }*/


        if(menuLocalizationMode && !bLocalizationMode)
        {
            mpSystem->ActivateLocalizationMode();
            bLocalizationMode = true;
        }
        else if(!menuLocalizationMode && bLocalizationMode)
        {
            mpSystem->DeactivateLocalizationMode();
            bLocalizationMode = false;
        }

        if(menuStepByStep && !bStepByStep)
        {
            mpTracker->SetStepByStep(true);
            bStepByStep = true;
        }
        else if(!menuStepByStep && bStepByStep)
        {
            mpTracker->SetStepByStep(false);
            bStepByStep = false;
        }

        if(menuStep)
        {
            mpTracker->mbStep = true;
            menuStep = false;
        }


        d_cam.Activate(s_cam);
        glClearColor(1.0f,1.0f,1.0f,1.0f);
        mpMapDrawer->DrawCurrentCamera(Twc);
        if(menuShowKeyFrames || menuShowGraph || menuShowInertialGraph)
            mpMapDrawer->DrawKeyFrames(menuShowKeyFrames,menuShowGraph, menuShowInertialGraph);
        if(menuShowPoints)
            mpMapDrawer->DrawMapPoints();

        if(IsYoloObject) //change 添加5行if else
        {
            mpTracker->isYoloObject=true;
        }
        else
        {
            mpTracker->isYoloObject=false;
        }

        if(mpTracker->mSensor == mpSystem->STEREO || mpTracker->mSensor == mpSystem->IMU_STEREO|| mpTracker->mSensor == mpSystem->RGBD)
        {
            if(menuShowLines)
            {
                mpMapDrawer->DrawMapLines();
            }
            if(menuShowPlanes)
            {
                //myplan 在地图上画面
                mpMapDrawer->draw_landmarks_plane();
            }
            // step draw objects.
            if(menuShowCubeObj || menuShowQuadricObj)
            {
                mpMapDrawer->DrawObject(menuShowCubeObj, menuShowQuadricObj,
                                        mflag,
                                        menuShowBottle, menuShowChair, menuShowTvmonitors,
                                        menuShowKeyboard,menuShowMouse,menuShowBook,menuShowBear);
            }
            if(menuShowSemiDense)
                mpMapDrawer->DrawSemiDense(menuSigmaTH);
            if(menuShowModel && menuShowTexture) {
                mpMapDrawer->DrawModel();
            }
            else if (menuShowModel && !menuShowTexture) {
                mpMapDrawer->DrawTriangles(Twc);
            }
            else if (!menuShowModel && menuShowTexture) {
                mpMapDrawer->DrawFrame();
            }

        }
        // 读取帧缓冲区中的像素数据到cv::Mat对象中
        glReadPixels(0, 0, 1024, 768, GL_BGR, GL_UNSIGNED_BYTE, image.data);

        // 翻转图像（因为OpenGL的坐标原点在左下角，而OpenCV的坐标原点在左上角）
        cv::flip(image, image, 0);

        /// imp 保存MAP图像到指定路径
//        cv::imwrite("/home/lzh/VINS/ORB3_LINE_LK_RGBD_Stereo/Exp_Picture/pl_all/map.jpg", image);
        
//***************************上面是画map的，下面图Frame***************************//
        pangolin::FinishFrame();

        cv::Mat im;
        cv::Mat toShow;
        //myline 画出线
        if(mpTracker->mSensor == mpSystem->STEREO || mpTracker->mSensor == mpSystem->IMU_STEREO|| mpTracker->mSensor == mpSystem->RGBD)
            im = mpFrameDrawer->DrawFrameWithLines(true);
        else
            im = mpFrameDrawer->DrawFrame(true);

        if(both){
            cv::Mat imRight = mpFrameDrawer->DrawRightFrame();
            cv::hconcat(im,imRight,toShow);
        }
        else{
            toShow = im;
        }
        //change 绘制目标框和类别框
        if(IsYoloObject) { // change 添加9行
            mpTracker->isYoloObject = true;
            {
                std::unique_lock<std::mutex> lock(mMutexPAFinsh);
                int x = 0; int y = 0; int z = 0;
                for (auto vit = mmDetectMap.begin(); vit != mmDetectMap.end(); vit++)
                {
                    //上绘制一个矩形框，矩形框的位置由三个属性指定，框线宽度为2
                    cv::rectangle(toShow, vit->bbox, cv::Scalar(vit->color_r ,vit->color_g, vit->color_b), 2);
                    draw_text(toShow,mClassnames[vit->class_idx] + ": " + cv::format("%.2f", vit->score),cv::Scalar(vit->color_r ,vit->color_g, vit->color_b),vit->bbox.tl());

                }
            }
        }
        /// imp 将当前Frame的保存
//        cv::imwrite("/home/lzh/VINS/SPL_SLAM/Exp_Picture/pl_all/frame.jpg", im);
        cv::imshow("SPL-SLAM: Current Frame",toShow);
        cv::waitKey(mT);

        if(menuReset)
        {
            menuShowGraph = true;
            menuShowInertialGraph = true;
            menuShowKeyFrames = true;
            menuShowPoints = true;
            //myline 函数
            if(mpTracker->mSensor == mpSystem->STEREO || mpTracker->mSensor == mpSystem->IMU_STEREO|| mpTracker->mSensor == mpSystem->RGBD)
                menuShowLines = true;
            menuLocalizationMode = false;
            if(bLocalizationMode)
                mpSystem->DeactivateLocalizationMode();
            bLocalizationMode = false;
            bFollow = true;
            menuFollowCamera = true;
            //mpSystem->Reset();
            mpSystem->ResetActiveMap();
            menuReset = false;
        }

        if(Stop())
        {
            while(isStopped())
            {
                usleep(3000);
            }
        }

        if(CheckFinish())
            break;
    }

    SetFinish();
}

void Viewer::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool Viewer::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void Viewer::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool Viewer::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void Viewer::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(!mbStopped)
        mbStopRequested = true;
}

bool Viewer::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool Viewer::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);

    if(mbFinishRequested)
        return false;
    else if(mbStopRequested)
    {
        mbStopped = true;
        mbStopRequested = false;
        return true;
    }

    return false;

}

void Viewer::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
}

void Viewer::SetTrackingPause()
{
    mbStopTrack = true;
}
void Viewer::draw_text(cv::Mat &img, const std::string &str, //change yolo 添加这个函数
                       const cv::Scalar &color, cv::Point pos, bool reverse)
{
    ///修改class类文本在图像中显示时所占用的大小。
    /// 1 字体比例因子，用于调整字体大小。
    ///1: 字体线条宽度，指定文本的线条粗细。
    auto t_size = cv::getTextSize(str, cv::FONT_HERSHEY_SIMPLEX, 1, 1, nullptr);
    cv::Point bottom_left, upper_right;
    if (reverse) {
        upper_right = pos;
        bottom_left = cv::Point(upper_right.x - t_size.width, upper_right.y + t_size.height);
    } else {
        bottom_left = pos;
        //修改这个两个都加0.5
        upper_right = cv::Point(bottom_left.x + 0.5*t_size.width, bottom_left.y - 0.5*t_size.height);
    }

    cv::rectangle(img, bottom_left, upper_right, color, -1);
    ///修改class类将文本绘制到其中的图像。
    cv::putText(img, str, bottom_left, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255) ,1);
}



}
