


#ifndef MAPDRAWER_H
#define MAPDRAWER_H

#include"Atlas.h"
#include"MapPoint.h"
//myline 添加头文件
#include"MapLine.h"
#include"KeyFrame.h"
#include<pangolin/pangolin.h>

#include<mutex>
#include "Converter.h"

namespace ORB_SLAM3
{
class Convert;
class MapDrawer
{
public:
    MapDrawer(Atlas* pAtlas, const string &strSettingPath);

    Atlas* mpAtlas;

    void DrawMapPoints();
    void DrawMapLines();
    void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph, const bool bDrawInertialGraph);
    void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);
    void SetCurrentCameraPose(const cv::Mat &Tcw);
    void SetReferenceKeyFrame(KeyFrame *pKF);
    void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M, pangolin::OpenGlMatrix &MOw);
    void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M, pangolin::OpenGlMatrix &MOw, pangolin::OpenGlMatrix &MTwwp);
//myplan
    void DrawSemiDense(const double sigma);
    void DrawModel();
    void DrawTriangles(pangolin::OpenGlMatrix &Twc);
    // BRIEF [EAO-SLAM] draw objects.
    void DrawObject(const bool bCubeObj, const bool QuadricObj,
                    const string &flag,
                    const bool bShowBottle,  const bool bShowChair, const bool bShowTvmonitors,
                    const bool bShowKeyboard,const bool bShowMouse, const bool bShowBook,   const bool bShowBear);
    void DrawFrame();

    // myplan
//    bool seg_or_not() const;
    void draw_landmarks_plane();
    unsigned int get_landmark_planes(std::vector<ORB_SLAM3::Plane *> &all_landmark_planes);
    // FW: draw planes
    struct PlaneColor
    {
        double _r;
        double _g;
        double _b;

        PlaneColor(double r, double g, double b) : _r(r), _g(g), _b(b)
        {
        }
    };
    std::vector<PlaneColor> get_available_color();
    float _transparency_alpha = 0.7;//0.7
    double _square_size = 0.15;//TODO 改变面大小
    bool _draw_plane_normal = false;//imp TODO FALSE
    bool _draw_dense_pointcloud = false;
    //myplan
    std::vector<Vector3d> all_plane_normals;


private:

    //myplan
    std::vector<PlaneColor> _mPlaneColors;
    bool ParseViewerParamFile(cv::FileStorage &fSettings);

    float mKeyFrameSize;
    float mKeyFrameLineWidth;
    float mGraphLineWidth;
    float mPointSize;
    float mCameraSize;
    float mCameraLineWidth;

    cv::Mat mCameraPose;

    std::mutex mMutexCamera;

    float mfFrameColors[6][3] = {{0.0f, 0.0f, 1.0f},
                                {0.8f, 0.4f, 1.0f},
                                {1.0f, 0.2f, 0.4f},
                                {0.6f, 0.0f, 1.0f},
                                {1.0f, 1.0f, 0.0f},
                                {0.0f, 1.0f, 1.0f}};
};

} //namespace ORB_SLAM

#endif // MAPDRAWER_H
