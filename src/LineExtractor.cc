

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

#include "LineExtractor.h"
#include <random>

//myplan
#include "KeyFrame.h"
#include <Thirdparty/Line3Dpp/clustering.h>
#include <Thirdparty/Line3Dpp/line3D.h>
#include <opencv2/core/eigen.hpp>
#include "Modeler.h"
#include <boost/filesystem.hpp>
#include "Thirdparty/EDTest/EDLib.h"

using namespace cv;
using namespace line_descriptor;
using namespace std;



namespace ORB_SLAM3
{


Lineextractor::Lineextractor(int _lsd_nfeatures, int _lsd_refine, float _lsd_scale, int _nlevels, float _scale, int _extractor)
    :lsd_nfeatures(_lsd_nfeatures), lsd_refine(_lsd_refine), lsd_scale(_lsd_scale), nlevels(_nlevels), scale(_scale), extractor(_extractor)
{

}



//img：输入图像，用于线段特征提取。
//mask：掩码图像，指定感兴趣区域。
//keylines：线段特征的输出向量，用于存储提取到的线段特征。
//descriptors_line：线段特征的描述子的输出矩阵，用于存储计算得到的线段特征的描述子


void Lineextractor::operator()( const cv::Mat& img, const cv::Mat& mask, 
            std::vector<cv::line_descriptor::KeyLine>& keylines, cv::Mat& descriptors_line)
{
    // Line Length Threshold
    min_line_length = 0.025;//设置线段长度阈值
    if(extractor==0) // LSD Extractor
    {
        // Detect line features
        Ptr<BinaryDescriptor>   lbd = BinaryDescriptor::createBinaryDescriptor();//创建 LSD 二进制描述子对象，
        Ptr<line_descriptor::LSDDetectorC> lsd = line_descriptor::LSDDetectorC::createLSDDetectorC();//创建 LSD 提取器
        keylines.clear();
        // lsd parameters
        //并设置 LSD 提取器的参数
        lsd_sigma_scale = 0.6;//：尺度因子的乘数，用于计算高斯差分函数的标准差
        lsd_quant = 2.0;//线段方向量化的量化步长。
        lsd_ang_th = 22.5;//线段角度的阈值，超过该阈值的线段将被忽略
        lsd_log_eps = 1.0;//用于计算线段长度的对数差的阈值。
        lsd_density_th = 0.6;//线段密度的阈值，超过该阈值的线段将被忽略。
        lsd_n_bins = 1024;//方向量化的直方图的数量。
        line_descriptor::LSDDetectorC::LSDOptions opts;
        opts.refine       = lsd_refine;//是否对线段进行细化处理的标志。
        opts.scale        = lsd_scale;//线段方向量化的尺度参数。
        opts.sigma_scale  = lsd_sigma_scale;//高斯差分函数的标准差。
        opts.quant        = lsd_quant;//：线段方向量化的量化步长。
        opts.ang_th       = lsd_ang_th;//：线段方向量化的量化步长。
        opts.log_eps      = lsd_log_eps;//线段长度的对数差的阈值。
        opts.density_th   = lsd_density_th;//：线段密度的阈值。
        opts.n_bins       = lsd_n_bins;//方向量化的直方图的数量。
        //：线段的最小长度阈值，根据图像的尺寸来设置
        opts.min_length   = min_line_length*(std::min(img.cols,img.rows)); 
        lsd->detect( img, keylines, scale, nlevels, opts);

//         filter keyline // **********************************************

        std::cout << "before " << keylines.size() << std::endl;
        std::cout << "lsd_nfeatures " << lsd_nfeatures << std::endl;

        auto new_keylines = keylines;
        new_keylines.clear();
        for (auto line : keylines)
        {
            auto start_point = cv::Point2i(line.startPointX, line.startPointY);
            auto end_point = cv::Point2i(line.endPointX, line.endPointY);
            bool find_flag= false;
            for (auto area : mvDynamicArea)
            {
                if (area.contains(start_point) && area.contains(end_point))
                {
                    find_flag = true;
                }
            }
            if (!find_flag)
            {
                new_keylines.push_back(line);
            }
        }
        keylines = new_keylines;
        std::cout << "AFTER " << keylines.size() << std::endl;

        lsd_nfeatures = keylines.size();



        //********************过滤长度过长的************************************//
//                std::cout << "before " << keylines.size() << std::endl;
//        auto new_keylines = keylines;
//        new_keylines.clear();
//        for (auto line : keylines)
//        {
//            bool dis_flag= false;
//            auto sl_x = line.startPointX;
//            auto sl_y=line.startPointY;
//            auto el_x=line.endPointX;
//            auto el_y=line.endPointY;
//            float dis_min=30;
//            float dis_max=1000;
//            float dis_l=std::sqrt((sl_x-el_x)*(sl_x-el_x)+(sl_y-el_y)*(sl_y-el_y));
//            if(dis_min<dis_l && dis_l<dis_max)
//            {
//                dis_flag= true;
//            }
//            if (dis_flag)
//            {
//
//                new_keylines.push_back(line);
//
//            }
//        }
//        keylines = new_keylines;
//        std::cout << "AFTER " << keylines.size() << std::endl;
//        lsd_nfeatures = keylines.size();



        //通过判断提取到的线段数量是否超过设定的最大线段数量
        if( int(keylines.size())>=lsd_nfeatures && lsd_nfeatures!=0  )//mynote 这里用大于等于如果需要去除动态线
        {
            // sort keylines by their response or by their length
            //据线段的响应值（response）或者长度（length）对线段进行排序
            sort( keylines.begin(), keylines.end(), sort_lines_by_response() );
            //sort( keylines.begin(), keylines.end(), sort_lines_by_length() );
            //然后，根据设定的最大线段数量（lsd_nfeatures）对线段进行裁剪，保留前面的线段
            keylines.resize(lsd_nfeatures);
            // reassign index
            //在裁剪后的线段列表中，重新为每个线段分配一个新的索引
            for( int i = 0; i < lsd_nfeatures; i++  )
                    keylines[i].class_id = i;
        }

        nlevels_l=nlevels;//存储不同层级
        mvLevelSigma2_l.resize(nlevels);//金字塔尺度参数
        mvLevelSigma2_l[0]=1.0f;//缩放系数
        mvInvLevelSigma2_l.resize(nlevels);//缩放系数的倒数
        for (int i = 0; i < nlevels; i++) 
        {
            mvImagePyramid_l.push_back(lsd->gaussianPyrs[i]);
            mvScaleFactor_l.push_back(lsd->mvScaleFactor[i]);
            mvInvScaleFactor_l.push_back(lsd->mvInvScaleFactor[i]);
            if (i>0)  
                mvLevelSigma2_l[i]=mvScaleFactor_l[i]*mvScaleFactor_l[i];
            mvInvLevelSigma2_l[i]=1.0f/mvLevelSigma2_l[i];
        }
        //算线段的描述符，计算结果存储在descriptors_line中。
        lbd->compute( img, keylines, descriptors_line);
    }
    else if(extractor==1) // ED Extractor
    {
        // Detect line features
        Ptr<BinaryDescriptor>   lbd = BinaryDescriptor::createBinaryDescriptor();
        Ptr<line_descriptor::LSDDetectorC> lsd = line_descriptor::LSDDetectorC::createLSDDetectorC();
        keylines.clear();
        double min_length = min_line_length*(std::min(img.cols,img.rows)); 
        lsd->detect_ED( img, keylines, scale, nlevels, min_length);
        // filter keyline
        if( int(keylines.size())>lsd_nfeatures && lsd_nfeatures!=0  )
        {
            // sort keylines by their response or by their length
            sort( keylines.begin(), keylines.end(), sort_lines_by_response() );
            //sort( keylines.begin(), keylines.end(), sort_lines_by_length() );
            keylines.resize(lsd_nfeatures);
            // reassign index
            for( int i = 0; i < lsd_nfeatures; i++  )
                    keylines[i].class_id = i;
        }

        nlevels_l=nlevels;
        mvLevelSigma2_l.resize(nlevels);
        mvLevelSigma2_l[0]=1.0f;
        mvInvLevelSigma2_l.resize(nlevels);
        for (int i = 0; i < nlevels; i++) 
        {
            mvImagePyramid_l.push_back(lsd->gaussianPyrs[i]);
            mvScaleFactor_l.push_back(lsd->mvScaleFactor[i]);
            mvInvScaleFactor_l.push_back(lsd->mvInvScaleFactor[i]);
            if (i>0)  
                mvLevelSigma2_l[i]=mvScaleFactor_l[i]*mvScaleFactor_l[i];
            mvInvLevelSigma2_l[i]=1.0f/mvLevelSigma2_l[i];
        } 

        lbd->compute( img, keylines, descriptors_line);        
    }
}





}//namespace ORB_SLAM
