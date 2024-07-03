

#include "FrameDrawer.h"
#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include<mutex>

namespace ORB_SLAM3
{

FrameDrawer::FrameDrawer(Atlas* pAtlas):both(false),mpAtlas(pAtlas)
{
    mState=Tracking::SYSTEM_NOT_READY;
    mIm = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));
    mImRight = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));
}

cv::Mat FrameDrawer::DrawFrame(bool bOldFeatures)
{
    // std::cout << "0" << std::endl;
    cv::Mat im;
    vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
    vector<int> vMatches; // Initialization: correspondeces with reference keypoints
    vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
    vector<bool> vbVO, vbMap; // Tracked MapPoints in current frame
    vector<pair<cv::Point2f, cv::Point2f> > vTracks;
    int state; // Tracking state

    //
    Frame currentFrame;
    vector<MapPoint*> vpLocalMap;
    vector<cv::KeyPoint> vMatchesKeys;
    vector<MapPoint*> vpMatchedMPs;
    vector<cv::KeyPoint> vOutlierKeys;
    vector<MapPoint*> vpOutlierMPs;
    map<long unsigned int, cv::Point2f> mProjectPoints;
    map<long unsigned int, cv::Point2f> mMatchedInImage;

    //Copy variables within scoped mutex
    {
        unique_lock<mutex> lock(mMutex);
        state=mState;
        if(mState==Tracking::SYSTEM_NOT_READY)
            mState=Tracking::NO_IMAGES_YET;

        mIm.copyTo(im);

        if(mState==Tracking::NOT_INITIALIZED)
        {
            vCurrentKeys = mvCurrentKeys;
            vIniKeys = mvIniKeys;
            vMatches = mvIniMatches;
            vTracks = mvTracks;
        }
        else if(mState==Tracking::OK /*&& bOldFeatures*/)
        {
            vCurrentKeys = mvCurrentKeys;
            vbVO = mvbVO;
            vbMap = mvbMap;

            currentFrame = mCurrentFrame;
            vpLocalMap = mvpLocalMap;
            vMatchesKeys = mvMatchedKeys;
            vpMatchedMPs = mvpMatchedMPs;
            vOutlierKeys = mvOutlierKeys;
            vpOutlierMPs = mvpOutlierMPs;
            mProjectPoints = mmProjectPoints;
            mMatchedInImage = mmMatchedInImage;

        }
        else if(mState==Tracking::LOST)
        {
            vCurrentKeys = mvCurrentKeys;
        }
    }

    if(im.channels()<3) //this should be always true
        cvtColor(im,im,CV_GRAY2BGR);

    //Draw
    if(state==Tracking::NOT_INITIALIZED)
    {
        for(unsigned int i=0; i<vMatches.size(); i++)
        {
            if(vMatches[i]>=0)
            {
                cv::line(im,vIniKeys[i].pt,vCurrentKeys[vMatches[i]].pt,
                        cv::Scalar(0,255,0));
            }
        }
        for(vector<pair<cv::Point2f, cv::Point2f> >::iterator it=vTracks.begin(); it!=vTracks.end(); it++)
            cv::line(im,(*it).first,(*it).second, cv::Scalar(0,255,0),5);

    }
    else if(state==Tracking::OK && bOldFeatures) //TRACKING
    {
        mnTracked=0;
        mnTrackedVO=0;
        const float r = 5;
        int n = vCurrentKeys.size();
        for(int i=0;i<n;i++)
        {
            if(vbVO[i] || vbMap[i])
            {
                cv::Point2f pt1,pt2;
                pt1.x=vCurrentKeys[i].pt.x-r;
                pt1.y=vCurrentKeys[i].pt.y-r;
                pt2.x=vCurrentKeys[i].pt.x+r;
                pt2.y=vCurrentKeys[i].pt.y+r;

                // This is a match to a MapPoint in the map
                if(vbMap[i])
                {
                    cv::rectangle(im,pt1,pt2,cv::Scalar(0,255,0));
                    cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(0,255,0),-1);
                    mnTracked++;
                }
                else // This is match to a "visual odometry" MapPoint created in the last frame
                {
                    cv::rectangle(im,pt1,pt2,cv::Scalar(255,0,0));
                    cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(255,0,0),-1);
                    mnTrackedVO++;
                }
            }
            /*else
            {
                cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(0,0,255),-1);
            }*/
        }
        // std::cout << "2.3" << std::endl;
    }
    else if(state==Tracking::OK && !bOldFeatures)
    {
        mnTracked=0;
        int nTracked2 = 0;
        mnTrackedVO=0;
        int n = vCurrentKeys.size();

        // cout << "----------------------" << endl;
        // cout << "Number of matches in old method: " << n << endl;

        for(int i=0; i < n; ++i)
        {

            // This is a match to a MapPoint in the map
            if(vbMap[i])
            {
                mnTracked++;
            }
        }

        n = mProjectPoints.size();
        //cout << "Number of projected points: " << n << endl;
        n = mMatchedInImage.size();
        //cout << "Number of matched points: " << n << endl;
        map<long unsigned int, cv::Point2f>::iterator it_match = mMatchedInImage.begin();
        while(it_match != mMatchedInImage.end())
        {
            long unsigned int mp_id = it_match->first;
            cv::Point2f p_image = it_match->second;

            if(mProjectPoints.find(mp_id) != mProjectPoints.end())
            {
                cv::Point2f p_proj = mMatchedInImage[mp_id];
                cv::line(im, p_proj, p_image, cv::Scalar(0, 255, 0), 2);
                nTracked2++;
            }
            else
            {
                cv::circle(im,p_image,2,cv::Scalar(0,0,255),-1);
            }


            it_match++;
            //it_proj = mProjectPoints.erase(it_proj);
        }
        //for(int i=0; i < n; ++i)
        //{
            /*if(!vpMatchedMPs[i])
                continue;*/

            //cv::circle(im,vProjectPoints[i],2,cv::Scalar(255,0,0),-1);
            /*cv::Point2f point3d_proy;
            float u, v;
            bool bIsInImage = currentFrame.ProjectPointDistort(vpMatchedMPs[i] , point3d_proy, u, v);
            if(bIsInImage)
            {
                //cout << "-Point is out of the image" << point3d_proy.x << ", " << point3d_proy.y << endl;
                cv::circle(im,vMatchesKeys[i].pt,2,cv::Scalar(255,0,0),-1);
                continue;
            }

            //cout << "+Point CV " << point3d_proy.x << ", " << point3d_proy.y << endl;
            //cout << "+Point coord " << u << ", " << v << endl;
            cv::Point2f point_im = vMatchesKeys[i].pt;

            cv::line(im,cv::Point2f(u, v), point_im,cv::Scalar(0, 255, 0), 1);*/

        //}

        /*cout << "Number of tracker in old method: " << mnTracked << endl;
        cout << "Number of tracker in new method: " << nTracked2 << endl;*/

        n = vOutlierKeys.size();
        //cout << "Number of outliers: " << n << endl;
        for(int i=0; i < n; ++i)
        {
            cv::Point2f point3d_proy;
            float u, v;
            currentFrame.ProjectPointDistort(vpOutlierMPs[i] , point3d_proy, u, v);

            cv::Point2f point_im = vOutlierKeys[i].pt;

            cv::line(im,cv::Point2f(u, v), point_im,cv::Scalar(0, 0, 255), 1);
        }

//        for(int i=0;i<n;i++)
//        {
//            if(vbVO[i] || vbMap[i])
//            {
//                cv::Point2f pt1,pt2;
//                pt1.x=vCurrentKeys[i].pt.x-r;
//                pt1.y=vCurrentKeys[i].pt.y-r;
//                pt2.x=vCurrentKeys[i].pt.x+r;
//                pt2.y=vCurrentKeys[i].pt.y+r;

//                // This is a match to a MapPoint in the map
//                if(vbMap[i])
//                {
//                    cv::rectangle(im,pt1,pt2,cv::Scalar(0,255,0));
//                    cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(0,255,0),-1);
//                    mnTracked++;
//                }
//                else // This is match to a "visual odometry" MapPoint created in the last frame
//                {
//                    cv::rectangle(im,pt1,pt2,cv::Scalar(255,0,0));
//                    cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(255,0,0),-1);
//                    mnTrackedVO++;
//                }
//            }
//        }
    }
    // std::cout << "3" << std::endl;

    cv::Mat imWithInfo;
    DrawTextInfo(im,state, imWithInfo);

    return imWithInfo;
}

cv::Mat FrameDrawer::DrawFrameWithLines(bool bOldFeatures)
{
    // std::cout << "0" << std::endl;
    cv::Mat im;
    vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
    vector<int> vMatches; // Initialization: correspondeces with reference keypoints
    vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
    vector<cv::line_descriptor::KeyLine> vCurrentKeys_Line; // KeyLines in current frame
    vector<bool> vbVO, vbMap, vbVO_l, vbMap_l; // Tracked MapPoints and MapLines in current frame
    vector<pair<cv::Point2f, cv::Point2f> > vTracks;
    int state; // Tracking state

    //
    Frame currentFrame;
    vector<MapPoint*> vpLocalMap;
    vector<cv::KeyPoint> vMatchesKeys;
    vector<MapPoint*> vpMatchedMPs;
    vector<cv::KeyPoint> vOutlierKeys;
    vector<MapPoint*> vpOutlierMPs;
    map<long unsigned int, cv::Point2f> mProjectPoints;
    map<long unsigned int, cv::Point2f> mMatchedInImage;

    //Copy variables within scoped mutex
    {
        unique_lock<mutex> lock(mMutex);
        state=mState;
        if(mState==Tracking::SYSTEM_NOT_READY)
            mState=Tracking::NO_IMAGES_YET;

        mIm.copyTo(im);

        if(mState==Tracking::NOT_INITIALIZED)
        {
            vCurrentKeys = mvCurrentKeys;
            vCurrentKeys_Line = mvCurrentKeys_l;
            vIniKeys = mvIniKeys;
            vMatches = mvIniMatches;
            vTracks = mvTracks;
        }
        else if(mState==Tracking::OK /*&& bOldFeatures*/)
        {
            vCurrentKeys = mvCurrentKeys;
            vbVO = mvbVO;
            vbMap = mvbMap;
            vCurrentKeys_Line = mvCurrentKeys_l;
            vbVO_l = mvbVO_l;
            vbMap_l = mvbMap_l;

            currentFrame = mCurrentFrame;
            vpLocalMap = mvpLocalMap;
            vMatchesKeys = mvMatchedKeys;
            vpMatchedMPs = mvpMatchedMPs;
            vOutlierKeys = mvOutlierKeys;
            vpOutlierMPs = mvpOutlierMPs;
            mProjectPoints = mmProjectPoints;
            mMatchedInImage = mmMatchedInImage;

        }
        else if(mState==Tracking::LOST)
        {
            vCurrentKeys = mvCurrentKeys;
            //myline
            vCurrentKeys_Line = mvCurrentKeys_l;
        }
    }

    if(im.channels()<3) //this should be always true
        cvtColor(im,im,CV_GRAY2BGR);

    //Draw
    if(state==Tracking::NOT_INITIALIZED)
    {
        for(unsigned int i=0; i<vMatches.size(); i++)
        {
            if(vMatches[i]>=0)
            {
                cv::line(im,vIniKeys[i].pt,vCurrentKeys[vMatches[i]].pt,
                        cv::Scalar(0,255,0));
            }
        }
        for(vector<pair<cv::Point2f, cv::Point2f> >::iterator it=vTracks.begin(); it!=vTracks.end(); it++)
            cv::line(im,(*it).first,(*it).second, cv::Scalar(0,255,0),5);

    }
    else if(state==Tracking::OK && bOldFeatures) //TRACKING
    {
        mnTracked=0;
        mnTrackedVO=0;
        mnTracked_l=0;
        mnTrackedVO_l=0;
        const float r = 5;
        int n = vCurrentKeys.size();
        for(int i=0;i<n;i++)
        {
            if(vbVO[i] || vbMap[i])
            {
                cv::Point2f pt1,pt2;
                pt1.x=vCurrentKeys[i].pt.x-r;
                pt1.y=vCurrentKeys[i].pt.y-r;
                pt2.x=vCurrentKeys[i].pt.x+r;
                pt2.y=vCurrentKeys[i].pt.y+r;

                // This is a match to a MapPoint in the map
                if(vbMap[i])
                {
                    cv::rectangle(im,pt1,pt2,cv::Scalar(0,255,0));
                    cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(0,255,0),-1);
                    mnTracked++;
                }
                else // This is match to a "visual odometry" MapPoint created in the last frame
                {
                    cv::rectangle(im,pt1,pt2,cv::Scalar(255,0,0));
                    cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(255,0,0),-1);
                    mnTrackedVO++;
                }
            }
            /*else
            {
                cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(0,0,255),-1);
            }*/
        }

        // mylk 调用画出lk光流场函数
        visualizeOpticalFlow(im, F_prepoint, F_nextpoint, State);
        visualizeDynaOpticalFlow(im,T_M_dyna);
        // std::cout << "2.3" << std::endl;

        // draw line features
        int nl = vCurrentKeys_Line.size();
        for(int i=0; i<nl; ++i)
        {
            if(vbVO_l[i] || vbMap_l[i])
            // if(true)
            {
                cv::Point2f sp, ep;
                sp.x = int(vCurrentKeys_Line[i].startPointX);
                sp.y = int(vCurrentKeys_Line[i].startPointY);
                ep.x = int(vCurrentKeys_Line[i].endPointX);
                ep.y = int(vCurrentKeys_Line[i].endPointY);
                if(vbMap_l[i]) {
                    // //B G R  myline 线的颜色和宽度   姜黄色 IMP
//
//*************************************************************
//                    bool find_flag= false;
//                    for (auto area: mvDynamicArea)
//                    {
//                        if (area.contains(sp) && area.contains(ep))
//
//                        {
//                            find_flag=true;
//
//                        }
//
//                    }
//                    if(find_flag)
//                    {
//                        cv::line(im, sp, ep, cv::Scalar(255,0,255), 2);
//                    }
//                    else
//                        cv::line(im, sp, ep, cv::Scalar(0,255,0), 2);
//*********************************************************************

                        cv::line(im, sp, ep, cv::Scalar(0,255,0), 2);            //  myline 线的颜色和宽度   IMP
                        ++mnTracked_l;
                }
                else {
                    cv::line(im, sp, ep, cv::Scalar(255,0,255), 1.5);                   // Magenta
                    ++mnTrackedVO_l;
                }
//                else {
//                    cv::line(im, sp, ep, cv::Scalar(0,255,255), 1.5);                 // Yellow
//                }
            }
        }
    }
    else if(state==Tracking::OK && !bOldFeatures)
    {
        mnTracked=0;
        mnTracked_l=0;
        int nTracked2 = 0;
        mnTrackedVO=0;
        mnTrackedVO_l=0;
        int n = vCurrentKeys.size();

        // cout << "----------------------" << endl;
        // cout << "Number of matches in old method: " << n << endl;

        for(int i=0; i < n; ++i)
        {

            // This is a match to a MapPoint in the map
            if(vbMap[i])
            {
                mnTracked++;
            }
        }

        n = mProjectPoints.size();
        //cout << "Number of projected points: " << n << endl;
        n = mMatchedInImage.size();
        //cout << "Number of matched points: " << n << endl;
        map<long unsigned int, cv::Point2f>::iterator it_match = mMatchedInImage.begin();
        while(it_match != mMatchedInImage.end())
        {
            long unsigned int mp_id = it_match->first;
            cv::Point2f p_image = it_match->second;

            if(mProjectPoints.find(mp_id) != mProjectPoints.end())
            {
                cv::Point2f p_proj = mMatchedInImage[mp_id];
                cv::line(im, p_proj, p_image, cv::Scalar(0, 255, 0), 2);
                nTracked2++;
            }
            else
            {
                cv::circle(im,p_image,2,cv::Scalar(0,0,255),-1);
            }


            it_match++;
            //it_proj = mProjectPoints.erase(it_proj);
        }


        n = vOutlierKeys.size();
        //cout << "Number of outliers: " << n << endl;
        for(int i=0; i < n; ++i)
        {
            cv::Point2f point3d_proy;
            float u, v;
            currentFrame.ProjectPointDistort(vpOutlierMPs[i] , point3d_proy, u, v);

            cv::Point2f point_im = vOutlierKeys[i].pt;
            // //B G R
            cv::line(im,cv::Point2f(u, v), point_im,cv::Scalar(125,189,248), 2);
        }


        // draw line features
        int nl = vCurrentKeys_Line.size();
        for(int i=0; i<nl; ++i)
        {
            if(vbVO_l[i] || vbMap_l[i])
            // if(true)
            {
                cv::Point2f sp, ep;
                sp.x = int(vCurrentKeys_Line[i].startPointX);
                sp.y = int(vCurrentKeys_Line[i].startPointY);
                ep.x = int(vCurrentKeys_Line[i].endPointX);
                ep.y = int(vCurrentKeys_Line[i].endPointY);
                if(vbMap_l[i]) {
                    // //B G R  myline 线的颜色和宽度   姜黄色
                    cv::line(im, sp, ep, cv::Scalar(125,189,248), 2);                  // Red
                    ++mnTracked_l;
                }
                else {
                    cv::line(im, sp, ep, cv::Scalar(255,0,255), 1.5);                   // Magenta
                    ++mnTrackedVO_l;
                }
//                else {
//                    cv::line(im, sp, ep, cv::Scalar(0,255,255), 1.5);                 // Yellow
//                }
            }
        }

    }
    // std::cout << "3" << std::endl;

    cv::Mat imWithInfo;
    DrawTextInfoWithLines(im,state, imWithInfo);//myline 这里去掉 IMP

    return imWithInfo;
}


cv::Mat FrameDrawer::DrawRightFrame()
{
    cv::Mat im;
    vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
    vector<int> vMatches; // Initialization: correspondeces with reference keypoints
    vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
    vector<bool> vbVO, vbMap; // Tracked MapPoints in current frame
    int state; // Tracking state

    //Copy variables within scoped mutex
    {
        unique_lock<mutex> lock(mMutex);
        state=mState;
        if(mState==Tracking::SYSTEM_NOT_READY)
            mState=Tracking::NO_IMAGES_YET;

        mImRight.copyTo(im);

        if(mState==Tracking::NOT_INITIALIZED)
        {
            vCurrentKeys = mvCurrentKeysRight;
            vIniKeys = mvIniKeys;
            vMatches = mvIniMatches;
        }
        else if(mState==Tracking::OK)
        {
            vCurrentKeys = mvCurrentKeysRight;
            vbVO = mvbVO;
            vbMap = mvbMap;
        }
        else if(mState==Tracking::LOST)
        {
            vCurrentKeys = mvCurrentKeysRight;
        }
    } // destroy scoped mutex -> release mutex

    if(im.channels()<3) //this should be always true
        cvtColor(im,im,CV_GRAY2BGR);

    //Draw
    if(state==Tracking::NOT_INITIALIZED) //INITIALIZING
    {
        for(unsigned int i=0; i<vMatches.size(); i++)
        {
            if(vMatches[i]>=0)
            {
                cv::line(im,vIniKeys[i].pt,vCurrentKeys[vMatches[i]].pt,
                         cv::Scalar(0,255,0));
            }
        }
    }
    else if(state==Tracking::OK) //TRACKING
    {
        mnTracked=0;
        mnTrackedVO=0;
        const float r = 5;
        const int n = mvCurrentKeysRight.size();
        const int Nleft = mvCurrentKeys.size();

        for(int i=0;i<n;i++)
        {
            if(vbVO[i + Nleft] || vbMap[i + Nleft])
            {
                cv::Point2f pt1,pt2;
                pt1.x=mvCurrentKeysRight[i].pt.x-r;
                pt1.y=mvCurrentKeysRight[i].pt.y-r;
                pt2.x=mvCurrentKeysRight[i].pt.x+r;
                pt2.y=mvCurrentKeysRight[i].pt.y+r;

                // This is a match to a MapPoint in the map
                if(vbMap[i + Nleft])
                {
                    cv::rectangle(im,pt1,pt2,cv::Scalar(0,255,0));
                    cv::circle(im,mvCurrentKeysRight[i].pt,2,cv::Scalar(0,255,0),-1);
                    mnTracked++;
                }
                else // This is match to a "visual odometry" MapPoint created in the last frame
                {
                    cv::rectangle(im,pt1,pt2,cv::Scalar(255,0,0));
                    cv::circle(im,mvCurrentKeysRight[i].pt,2,cv::Scalar(255,0,0),-1);
                    mnTrackedVO++;
                }
            }
        }
    }


    cv::Mat imWithInfo;
    DrawTextInfo(im,state, imWithInfo);

    return imWithInfo;
}

void FrameDrawer::DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText)
{
    stringstream s;
    if(nState==Tracking::NO_IMAGES_YET)
        s << " WAITING FOR IMAGES";
    else if(nState==Tracking::NOT_INITIALIZED)
        s << " TRYING TO INITIALIZE ";
    else if(nState==Tracking::OK)
    {
        if(!mbOnlyTracking)
            s << "SLAM MODE |  ";
        else
            s << "LOCALIZATION | ";
        int nMaps = mpAtlas->CountMaps();
        int nKFs = mpAtlas->KeyFramesInMap();
        int nMPs = mpAtlas->MapPointsInMap();
        s << "Maps: " << nMaps << ", KFs: " << nKFs << ", MPs: " << nMPs << ", Matches: " << mnTracked;
        if(mnTrackedVO>0)
            s << ", + VO matches: " << mnTrackedVO;
    }
    else if(nState==Tracking::LOST)
    {
        s << " TRACK LOST. TRYING TO RELOCALIZE ";
    }
    else if(nState==Tracking::SYSTEM_NOT_READY)
    {
        s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
    }

    int baseline=0;
    cv::Size textSize = cv::getTextSize(s.str(),cv::FONT_HERSHEY_PLAIN,1,1,&baseline);

    imText = cv::Mat(im.rows+textSize.height+10,im.cols,im.type());
    im.copyTo(imText.rowRange(0,im.rows).colRange(0,im.cols));
    imText.rowRange(im.rows,imText.rows) = cv::Mat::zeros(textSize.height+10,im.cols,im.type());
    cv::putText(imText,s.str(),cv::Point(5,imText.rows-5),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,255,255),1,8);

}

void FrameDrawer::DrawTextInfoWithLines(cv::Mat &im, int nState, cv::Mat &imText)
{
    stringstream s;
    if(nState==Tracking::NO_IMAGES_YET)
        s << " WAITING FOR IMAGES";
    else if(nState==Tracking::NOT_INITIALIZED)
        s << " TRYING TO INITIALIZE ";
    else if(nState==Tracking::OK)
    {
        if(!mbOnlyTracking)
            s << "SLAM MODE |  ";
        else
            s << "LOCALIZATION | ";
        int nMaps = mpAtlas->CountMaps();
        int nKFs = mpAtlas->KeyFramesInMap();
        int nMPs = mpAtlas->MapPointsInMap();
        int nMLs = mpAtlas->MapLinesInMap();
        s << "Maps: " << nMaps << ", KFs: " << nKFs << ", MPs: " << nMPs << ", PMatches: " << mnTracked << ", MLs: " << nMLs << ", LMatches: " << mnTracked_l;
        if(mnTrackedVO>0)
            s << ", +VO_MatchesP: " << mnTrackedVO;
        if(mnTrackedVO_l>0)
            s << ", +VO_MatchesL: " << mnTrackedVO_l;
    }
    else if(nState==Tracking::LOST)
    {
        s << " TRACK LOST. TRYING TO RELOCALIZE ";
    }
    else if(nState==Tracking::SYSTEM_NOT_READY)
    {
        s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
    }

    int baseline=0;
    cv::Size textSize = cv::getTextSize(s.str(),cv::FONT_HERSHEY_PLAIN,1,1,&baseline);

//    imText = cv::Mat(im.rows+textSize.height+10,im.cols,im.type());
//    im.copyTo(imText.rowRange(0,im.rows).colRange(0,im.cols));
//    imText.rowRange(im.rows,imText.rows) = cv::Mat::zeros(textSize.height+10,im.cols,im.type());
//    cv::putText(imText,s.str(),cv::Point(5,imText.rows-5),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,255,255),1,8);

//    imText = cv::Mat(im.rows+textSize.height,im.cols,im.type());
//    im.copyTo(imText.rowRange(0,im.rows).colRange(0,im.cols));
//    imText.rowRange(im.rows,imText.rows) = cv::Mat::zeros(textSize.height,im.cols,im.type());
//    cv::putText(im, s.str(), cv::Point(5, im.rows-5), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

    imText = cv::Mat(im.rows, im.cols, im.type());
    im.copyTo(imText.rowRange(0, im.rows).colRange(0, im.cols));




//    cv::putText(imText,s.str(),cv::Point(5,imText.rows-5),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,255,255),1,8);

}

void FrameDrawer::Update(Tracking *pTracker)
{
    unique_lock<mutex> lock(mMutex);
    pTracker->mImGray.copyTo(mIm);
    mvCurrentKeys=pTracker->mCurrentFrame.mvKeys;
    //myline 加线
    mvCurrentKeys_l = pTracker->mCurrentFrame.mvKeys_Line;

//    mThDepth = pTracker->mCurrentFrame.mThDepth;
//    mvCurrentDepth = pTracker->mCurrentFrame.mvDepth;

    //mylk 下面四行
    F_prepoint = pTracker->mCurrentFrame.F_prepoint_draw;
    F_nextpoint = pTracker->mCurrentFrame.F_nextpoint_draw;
    State = pTracker->mCurrentFrame.state_draw;
    T_M_dyna = pTracker->mCurrentFrame.T_M;


    if(both){
        mvCurrentKeysRight = pTracker->mCurrentFrame.mvKeysRight;
        pTracker->mImRight.copyTo(mImRight);
        N = mvCurrentKeys.size() + mvCurrentKeysRight.size();
    }
    else{
        N = mvCurrentKeys.size();
        //myline
        N_l = mvCurrentKeys_l.size();
    }

    //cout << "Number of matches in frame: " << N << endl;
    // cout << "Number of matches in frame: " << N << endl;
    mvbVO = vector<bool>(N,false);
    mvbMap = vector<bool>(N,false);
    //myline
    mvbVO_l = vector<bool>(N_l,false);
    mvbMap_l = vector<bool>(N_l,false);
    mbOnlyTracking = pTracker->mbOnlyTracking;

    //Variables for the new visualization
    mCurrentFrame = pTracker->mCurrentFrame;
    mmProjectPoints = mCurrentFrame.mmProjectPoints;
    //mmMatchedInImage = mCurrentFrame.mmMatchedInImage;
    mmMatchedInImage.clear();

    mvpLocalMap = pTracker->GetLocalMapMPS();
    mvMatchedKeys.clear();
    mvMatchedKeys.reserve(N);
    mvpMatchedMPs.clear();
    mvpMatchedMPs.reserve(N);
    mvOutlierKeys.clear();
    mvOutlierKeys.reserve(N);
    mvpOutlierMPs.clear();
    mvpOutlierMPs.reserve(N);
    //mvProjectPoints.clear();
    //mvProjectPoints.reserve(N);

    if(pTracker->mLastProcessedState==Tracking::NOT_INITIALIZED)
    {
        mvIniKeys=pTracker->mInitialFrame.mvKeys;
        mvIniMatches=pTracker->mvIniMatches;
    }
    else if(pTracker->mLastProcessedState==Tracking::OK)
    {
        for(int i=0;i<N;i++)
        {
            MapPoint* pMP = pTracker->mCurrentFrame.mvpMapPoints[i];
            if(pMP)
            {
                if(!pTracker->mCurrentFrame.mvbOutlier[i])
                {
                    if(pMP->Observations()>0)
                        mvbMap[i]=true;
                    else
                        mvbVO[i]=true;

                    //mvpMatchedMPs.push_back(pMP);
                    //mvMatchedKeys.push_back(mvCurrentKeys[i]);
                    mmMatchedInImage[pMP->mnId] = mvCurrentKeys[i].pt;

                    //cv::Point2f point3d_proy;
                    //float u, v;
                    //bool bIsInImage = mCurrentFrame.ProjectPointDistort(pMP, point3d_proy, u, v);
                    //if(bIsInImage)
                    //{
                        //mvMatchedKeys.push_back(mvCurrentKeys[i]);
                        //mvProjectPoints.push_back(cv::Point2f(u, v));
                    //}
                }
                else
                {
                    mvpOutlierMPs.push_back(pMP);
                    mvOutlierKeys.push_back(mvCurrentKeys[i]);
                }
            }
        }
        //myline 更新线
        for(int i=0;i<N_l;i++)
        {
            MapLine* pML = pTracker->mCurrentFrame.mvpMapLines[i];
            if(pML)
            {
                if(!pTracker->mCurrentFrame.mvbOutlier_Line[i])
                {
                    if(pML->Observations()>0)
                        mvbMap_l[i]=true;
                    else
                        mvbVO_l[i]=true;
                }
            }
        }
    }
    mState=static_cast<int>(pTracker->mLastProcessedState);
}

//mylk 函数
void FrameDrawer::visualizeOpticalFlow( cv::Mat& image, const std::vector<cv::Point2f>& points1, const std::vector<cv::Point2f>& points2, const std::vector<uchar>& status) {
    //cv::Mat visImage = image.clone();

    for (int i = 0; i < points1.size(); i++)
    {
        if (status[i] == 1) {
            cv::Point2f pt1 = points1[i];
            cv::Point2f pt2 = points2[i];

            cv::Point2f pta,ptb;
            const float r = 5;
            pta.x=pt2.x-r;
            pta.y=pt2.y-r;
            ptb.x=pt2.x+r;
            ptb.y=pt2.y+r;

            // 计算箭头的目标点
            cv::Point2f arrowEnd = pt2 + (pt2 - pt1) * 3; // 根据需要调整箭头长度

            // 绘制带有箭头的线段 226, 43, 138 紫色
            cv::arrowedLine(image, pt1, arrowEnd, cv::Scalar(28, 26, 226), 1.5);//画出光流箭头


                //cv::line(image, pt1, pt2, cv::Scalar(0, 255, 0), 2);
                //imp 第四次四行
            bool find_flag= false;
            for (auto area: mvDynamicArea)
            {
                if (area.contains(pt2))

                {
                    find_flag=true;

                }

            }
            if(find_flag)
            {
                //    //MYNOTE 这些当前帧的特征点传入到FrameDrwer 画出红色小点
                //    //MYNOTE 意思是前后帧满足lk基础矩阵约束的特征点进行下一步特帧点的跟踪和匹配
                //    //mynote 这里提取出来的特征点还没进行Desp计算，这里只是预处理后，表示可以用来稳定跟踪的特征点，然后跟Desp计算
                //desp计算出 计算特征点的描述子并恢复特征点的坐标
//                cv::rectangle(image, pta, ptb, cv::Scalar(255, 0, 0));//BGR
//                cv::circle(image, pt2, 2, cv::Scalar(255, 0, 0), -1);
            }
            else
                cv::rectangle(image, pta, ptb, cv::Scalar(0, 255, 0));//BGR
            if(find_flag)
            {
//                cv::circle(image, pt2, 2, cv::Scalar(255, 0, 0), -1);
            }
            else
                cv::circle(image, pt2, 2, cv::Scalar(0, 255, 0), -1);//mynote 这里的pt2 不在动态框内的也改成绿色框点了
//                cv::circle(image, pt2, 3, cv::Scalar(28, 26, 226), -1);//-1 实心圆， 这里是画出当前帧稳定特征点，位于动态框之外的可以用来稳定跟踪的特征点，然后跟Desp计算

            cv::circle(image, pt2, 2, cv::Scalar(28, 26, 226), -1);//-1 实心圆， 这里是画出当前帧稳定特征点 //imp 第四次
        }
    }

}
//mylk 函数 画出经过光流基础矩阵约束，极线距离大于阈值的外点，用粉红色大圆圈表示
void FrameDrawer::visualizeDynaOpticalFlow(cv::Mat& image, const std::vector<cv::Point2f>& points)
{
    for (const cv::Point2f& point : points) {

//        bool find_flag= false;
//        for (auto area: mvDynamicArea)
//        {
//            if (area.contains(point))
//
//            {
//                find_flag=true;
//
//            }
//
//        }
//        if(find_flag)
//        {
//            cv::circle(image, point, 5, cv::Scalar(255, 0, 255), -1);//--1实心
//        }
    }
}


} //namespace ORB_SLAM
