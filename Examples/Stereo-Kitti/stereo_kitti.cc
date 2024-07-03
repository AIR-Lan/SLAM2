/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;

///home/lzh/VINS/ORB3_LINE_LK_Stereo/Vocabulary/ORBvoc.txt
///home/lzh/VINS/ORB3_LINE_LK_Stereo/Vocabulary/LSDvoc.txt
///home/lzh/VINS/ORB3_LINE_LK_Stereo/Examples/Stereo-Line/EuRoC.yaml
///media/lzh/新加卷/Ubuntu1804_Copy/kitti_dataset_stereo_rgb/Euroc/MH_04
///home/lzh/VINS/ORB3_LINE_LK_Stereo/Examples/Stereo-Line/EuRoC_TimeStamps/MH04.txt
///home/lzh/VINS/ORB3_LINE_LK_Stereo/Examples/Stereo-Line/EuRoc_Output


//图像加载函数，参数1，图像路径，参数2左图路径，参数3右图路径，参数3语义图片路径vector<string> &vstrSemanticImageFilenames，参数4时间戳路径
void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps);

int main(int argc, char **argv)//程序执行主函数
{
    if(argc != 5)
    {
        cerr << endl << "Usage: ./stereo_kitti path_to_vocabulary path_to_line_voc path_to_settings path_to_sequence" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
//    vector<string> vstrSemanticImageFilenames;
    vector<double> vTimestamps;
    //vstrSemanticImageFilenames
    LoadImages(string(argv[4]), vstrImageLeft, vstrImageRight,  vTimestamps);

    const int nImages = vstrImageLeft.size();//计算图像的数量

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    //slam线程函数
    ///mynote 进去开启追踪，局部建图，回环线程，显示线程
    ORB_SLAM3::System SLAM(argv[1],argv[2],argv[3],ORB_SLAM3::System::STEREO,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    double t_track = 0.f;
    double t_resize = 0.f;

    // Main loop
    cv::Mat imLeft, imRight;
    for(int ni=0; ni<nImages; ni++)
    {
//        std::cout << vstrSemanticImageFilenames[ni] << std::endl; ///加载语义图片
        // Read left and right images from file
        //读取左图右图图片
        imLeft = cv::imread(vstrImageLeft[ni],cv::IMREAD_UNCHANGED); //,cv::IMREAD_UNCHANGED);
        imRight = cv::imread(vstrImageRight[ni],cv::IMREAD_UNCHANGED); //,cv::IMREAD_UNCHANGED);
//        semantic_img = cv::imread(vstrSemanticImageFilenames[ni],cv::IMREAD_UNCHANGED);
//        cv::imshow("seg", semantic_img);
        cv::waitKey(1);
        double tframe = vTimestamps[ni];

        if(imLeft.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(vstrImageLeft[ni]) << endl;
            return 1;
        }

#ifdef COMPILEDWITHC14
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the images to the SLAM system
        //MYSTEP 2 构造frame 开启双目追踪线程
        SLAM.TrackStereo(imLeft,imRight,tframe, vector<ORB_SLAM3::IMU::Point>(), vstrImageLeft[ni]);

#ifdef COMPILEDWITHC14
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

#ifdef REGISTER_TIMES
        t_track = t_resize + std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2 - t1).count();
        SLAM.InsertTrackTime(t_track);
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");

    return 0;
}
//函数内容，参数1图像路径，参数2左图路径，参数3右图路径，参数4vector<string> &vstrSemanticImageFilenames语义图片路径，参数5时间戳
void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps)
{
    ifstream fTimes;//实例化类。用来从文件中读取数据
    string strPathTimeFile = strPathToSequence + "/times.txt";//存储时间戳地址
    fTimes.open(strPathTimeFile.c_str());//打开文件
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);//读取文件里的时间戳数据存到容器中
        }
    }

    string strPrefixLeft = strPathToSequence + "/image_0/";
    string strPrefixRight = strPathToSequence + "/image_1/";
//    string strPrefixSemantic = strPathToSequence + "/semantic_0/";

    const int nTimes = vTimestamps.size();
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);
//    vstrSemanticImageFilenames.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";//左图文件名
        vstrImageRight[i] = strPrefixRight + ss.str() + ".png";//右图文件名
//        vstrSemanticImageFilenames[i] = strPrefixSemantic + ss.str() + ".png";
    }
}
