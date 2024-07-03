

#include "planar_mapping_module.h"
#include "Frame.h"
#include "KeyFrame.h"
#include "Map.h"
#include "Atlas.h"
#include "MapPoint.h"
#include "landmark_plane.h"

#include <random>

#include <opencv2/core/core.hpp>
#include<mutex>
// added libs for Graph-cut RANSAC
#include <opencv2/calib3d.hpp>

#include "types.h"


#include "Converter.h"

//myplan 这个函数用来开启面建图的函数
//#include "gridStructure.h"

#include "GCRANSAC/types.h"
#include "GCRANSAC/model.h"
#include "GCRANSAC/flann_neighborhood_graph.h"
#include "GCRANSAC/uniform_sampler.h"
#include "GCRANSAC/sampler.h"
#include "GCRANSAC/preemption_sprt.h"
#include "GCRANSAC/GCRANSAC.h"
#include "GCRANSAC/scoring_function.h"



namespace ORB_SLAM3
{
    //类构造函数初始化类的对象进行初始化和设置
    Planar_Mapping_module::Planar_Mapping_module(Atlas *pAtlas, const int sensor)
            :  plSensor(sensor),mpAtlas(pAtlas)
//            mpMap(pMap)
    {
//        std::cout<<"plsensor===="<<plSensor<<std::endl;
        // FW: this is a global flag indicates using instance planar segmentation
        // used in initialization, tracking, mapping, map_publisher and visualization
//        pMap->_b_seg_or_not = true;

        // see "./planar_mapping_parameters.yaml"
        load_configuration();
    }
//析构函数
    Planar_Mapping_module::~Planar_Mapping_module()
    {
    }
//方法的目标是使用给定的关键帧初始化地图，并创建一个平面。
    bool Planar_Mapping_module::initialize_map_with_plane(KeyFrame* pKFini)
    {
        if (pKFini->isBad())//是否去掉关键帧的标志位
        {
            return false;
        }

        return process_new_kf(pKFini);
    }
    //方法的目标是处理新的关键帧。它首先估计地图尺度，
    // 然后根据关键帧创建颜色-平面映射，最后尝试通过RANSAC拟合创建新的平面。
    bool Planar_Mapping_module::process_new_kf(KeyFrame* pKFini)
    {
        if (pKFini->isBad())
        {
            return false;
        }

        std::lock_guard<std::mutex> lock(_mtxPlaneMutex);//imp bug

//        if (_setVerbose)
//        {
//            spdlog::info("-- PlanarMapping -- processing keyframe: id({})", keyfrm->id_);
//        }

        // [1] estimate map scale, and calculate the geometric thresholds for RANSAC
        if (plSensor==0)
        {
            // monocular mode, the scale need to be estimated dynamicly
            //myplan 单目的目前没写
//            estimate_map_scale(pRefKF); // estimate map scale as inverse of median depth of this keyframe
            std::cout<<"单目的没写"<<std::endl;
        }
        else
        {
            // RGB-D mode, the scale is (more or less) fixed
            if (mpAtlas->GetAllKeyFrames().size() < 3)//imp 这里有bug
//            if(1)
//            {
                estimate_map_scale(); // estimate map scale as average of sum of the norm of all the map points coordinates
//            }
        }

        // [2] create an unordered_map between the segmentation label and the potential 3D plane
        // the map points which corresponding to the 2D keypoints are linked to the potential plane for later RANSAC fitting
//        auto before = std::chrono::high_resolution_clock::now();

        eigen_alloc_unord_map<long, ORB_SLAM3::Plane *> colorToPlanes;
//        create_ColorToPlane(pKFini, colorToPlanes);
        if (create_ColorToPlane(pKFini, colorToPlanes))//TODO BUG

        {
//            std::cout<<"创建好了对应颜色块"<<std::endl;
//            auto after = std::chrono::high_resolution_clock::now();
//            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(after - before).count();

//            if (_setVerbose)
//            {
//                spdlog::info("\t \t | process time (create ColorPlane): {}ms", duration);
//            }

            // [3] try to reconstruct 3D plane///todo 这里没有新建面
            return create_new_plane(colorToPlanes);
        }
//        std::cout<<"创建失败对应颜色块"<<std::endl;
        return false;
    }

    // for Monocular


    // for RGB-D
//该方法的目的是估计地图的尺度，即计算所有地图点在世界坐标系中的平均位置，
// 并根据尺度计算一些用于平面建模的阈值参数。
// 这些阈值参数用于后续的地图更新和优化过程中，以控制地图点的添加和优化
    void Planar_Mapping_module::estimate_map_scale()
    {
//        std::cout<<"进入了estimate_map_scale"<<std::endl;
        // The map scale is calculated as the average of sum of world postion of all the map points
//        auto before = std::chrono::high_resolution_clock::now();

        std::vector<ORB_SLAM3::MapPoint *> lms = mpAtlas->GetAllMapPoints();//LANDMARK =ORB 地图点
        double map_points_added = 0.0;
        for (auto const &lm : lms)
        {
            if (!lm->isBad())
            {
                cv::Mat pos_w = lm->GetWorldPos();
                map_points_added += cv::norm(pos_w);
            }
        }

        _map_scale = map_points_added / lms.size();
        _planar_distance_thresh = PLANAR_DISTANCE_CORRECTION * _map_scale;
        _final_error_thresh = FINAL_ERROR_CORRECTION * _map_scale;
//        std::cout<<" _map_scale====="<< _map_scale  <<std::endl;
//        std::cout<<" _planar_distance_thresh====="<< _planar_distance_thresh  <<std::endl;
//        std::cout<<" _final_error_thresh"<<  _final_error_thresh  <<std::endl;



//        auto after = std::chrono::high_resolution_clock::now();
//        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(after - before).count();
//
//        if (_setVerbose)
//        {
//            spdlog::info("\t \t | _map_scale: {}", _map_scale);
//            spdlog::info("\t \t | _planar_distance_thresh: {}", _planar_distance_thresh);
//            spdlog::info("\t \t | _final_error_thr: {}", _final_error_thresh);
//            spdlog::info("\t \t | process time (scale): {}ms", duration);
//        }
    }

    bool Planar_Mapping_module::create_ColorToPlane(ORB_SLAM3::KeyFrame *pKFini, eigen_alloc_unord_map<long, ORB_SLAM3::Plane *> &colorToPlanes)
    {
        if (pKFini->isBad())
        {
            return false;
        }

        cv::Mat segmentation_mask = pKFini->get_segmentation_mask();
        std::set<ORB_SLAM3::MapPoint *> lms = pKFini->GetMapPoints();
//        std::cout<<"create_ColorToPlane传的mask 大小 = "<< segmentation_mask.size()<<std::endl;
//        cv::imshow("test", segmentation_mask);
//        std::cout<<"mask图像的高度=="<< segmentation_mask.rows <<"mask图像的宽"<<segmentation_mask.cols<<std::endl;
        if (segmentation_mask.empty() || lms.empty())//这里mask图像为空 todo BUG
        {
            return false;
        }

        for (auto &lm : lms)
        {
            if (lm->isBad() || lm->get_Owning_Plane())
            {
                continue;
            }
            //
//            std::tuple<int, int> tuple_value = lm->GetIndexInKeyFrame(pKFini);
//            std::cout << "GetIndexInKeyFrame地图点索引+关键帧索引=====: " << std::get<0>(tuple_value) << ", " << std::get<1>(tuple_value ) << std::endl;

//            std::tuple<int, int> test = lm->GetObservations().at(pKFini);
//            std::cout << "GetObservations地图点索引+关键帧索引=====: " << std::get<0>(test) << ", " << std::get<1>(test ) << std::endl;

            std::tuple<int, int> observations = lm->GetObservations().at(pKFini);
            int kpt_id = std::get<0>(observations);//这里kpt_id =-1
//            int kpt_id = std::get<1>(lm->GetObservations());//这里kpt_id =-1
            auto kpt = pKFini->mvKeysUn[kpt_id];

            if (kpt.pt.y < 0 || kpt.pt.y >= segmentation_mask.rows ||
                kpt.pt.x < 0 || kpt.pt.x >= segmentation_mask.cols)
            {
                std::cout << "keypoint out of image range!" << std::endl;
                continue;
            }

//            std::cout<<"mask图像的高度=="<< segmentation_mask.rows <<"mask图像的宽=="<<segmentation_mask.cols<<std::endl;

            // get the color (the semantic label from segmentation) of this pixel
            auto center_color = segmentation_mask.at<cv::Vec3b>((int)kpt.pt.y, (int)kpt.pt.x);
            long pseudo_hash_center = center_color.val[0] + (center_color.val[1] << 8) + (center_color.val[2] << 16);

//            std::cout << "pseudo_hash_center========" << pseudo_hash_center << std::endl;//0
//            std::cout << "center_color.val[0]========" <<center_color.val[0]<< std::endl;//0
//            std::cout << "center_color.val[1]<<8========" <<center_color.val[1] << std::endl;//80
//            std::cout << "center_color.val[2] << 16===========" <<center_color.val[2] << std::endl;//160
//            std::cout << "center_color.val========"<< center_color.val << std::endl;//0
            if (pseudo_hash_center == 0)
            {
                continue;
            }

            if (!_check_3x3_window)
            {
                if (colorToPlanes.empty() || !colorToPlanes.count(pseudo_hash_center))
                {
                    auto plane = new ORB_SLAM3::Plane(pKFini, mpAtlas);
                    colorToPlanes[pseudo_hash_center] = plane;
                }
                else
                {
                    // avoid duplicate
                    if (!colorToPlanes.count(pseudo_hash_center))
                    {
                        auto plane = new ORB_SLAM3::Plane(pKFini, mpAtlas);
                        colorToPlanes[pseudo_hash_center] = plane;
                    }
                }

                colorToPlanes[pseudo_hash_center]->add_landmark(lm);
            }
            else
            {
                //  check a small 3x3 window if all the pixel belongs to the same semantic class
                std::vector<cv::Vec3b> color_list;

                if ((int)kpt.pt.y + 1 > 0 && (int)kpt.pt.y + 1 < segmentation_mask.rows &&
                    (int)kpt.pt.x + 1 > 0 && (int)kpt.pt.x + 1 < segmentation_mask.cols)
                    color_list.push_back(segmentation_mask.at<cv::Vec3b>((int)kpt.pt.y + 1, (int)kpt.pt.x + 1)); // bottom right

                if ((int)kpt.pt.y - 1 > 0 && (int)kpt.pt.y - 1 < segmentation_mask.rows &&
                    (int)kpt.pt.x - 1 > 0 && (int)kpt.pt.x - 1 < segmentation_mask.cols)
                    color_list.push_back(segmentation_mask.at<cv::Vec3b>((int)kpt.pt.y - 1, (int)kpt.pt.x - 1)); // top left

                if ((int)kpt.pt.y + 1 > 0 && (int)kpt.pt.y + 1 < segmentation_mask.rows &&
                    (int)kpt.pt.x - 1 > 0 && (int)kpt.pt.x - 1 < segmentation_mask.cols)
                    color_list.push_back(segmentation_mask.at<cv::Vec3b>((int)kpt.pt.y + 1, (int)kpt.pt.x - 1)); // bottom left

                if ((int)kpt.pt.y - 1 > 0 && (int)kpt.pt.y - 1 < segmentation_mask.rows &&
                    (int)kpt.pt.x + 1 > 0 && (int)kpt.pt.x + 1 < segmentation_mask.cols)
                    color_list.push_back(segmentation_mask.at<cv::Vec3b>((int)kpt.pt.y - 1, (int)kpt.pt.x + 1)); // top right

                if ((int)kpt.pt.y + 1 > 0 && (int)kpt.pt.y + 1 < segmentation_mask.rows &&
                    (int)kpt.pt.x > 0 && (int)kpt.pt.x < segmentation_mask.cols)
                    color_list.push_back(segmentation_mask.at<cv::Vec3b>((int)kpt.pt.y + 1, (int)kpt.pt.x)); // bottom

                if ((int)kpt.pt.y - 1 > 0 && (int)kpt.pt.y - 1 < segmentation_mask.rows &&
                    (int)kpt.pt.x > 0 && (int)kpt.pt.x < segmentation_mask.cols)
                    color_list.push_back(segmentation_mask.at<cv::Vec3b>((int)kpt.pt.y - 1, (int)kpt.pt.x)); // top

                if ((int)kpt.pt.y > 0 && (int)kpt.pt.y < segmentation_mask.rows &&
                    (int)kpt.pt.x - 1 > 0 && (int)kpt.pt.x - 1 < segmentation_mask.cols)
                    color_list.push_back(segmentation_mask.at<cv::Vec3b>((int)kpt.pt.y, (int)kpt.pt.x - 1)); // left

                if ((int)kpt.pt.y > 0 && (int)kpt.pt.y < segmentation_mask.rows &&
                    (int)kpt.pt.x + 1 > 0 && (int)kpt.pt.x + 1 < segmentation_mask.cols)
                    color_list.push_back(segmentation_mask.at<cv::Vec3b>((int)kpt.pt.y, (int)kpt.pt.x + 1)); // right

                bool if_consistant_color = true;
//                if (!color_list.empty())
//                {
//                    for (auto &color : color_list)
//                    {
//                        long pseudo_hash = color.val[0] + (color.val[1] << 8) + (color.val[2] << 16);
//                        if (pseudo_hash == 0 || pseudo_hash_center != pseudo_hash)
//                        {
//                            // non-planar surfaces are black, all the pixel inside 3x3 window should have same label
//                            if_consistant_color = false;
//                            break;
//                        }
//                    }
//                }

                if (if_consistant_color)
                {
                    // create a new plane if a new color (semantic label) found
                    if (colorToPlanes.empty())
                    {
                        auto plane = new ORB_SLAM3::Plane(pKFini, mpAtlas);
                        colorToPlanes[pseudo_hash_center] = plane;
                    }
                    else
                    {
                        // avoid duplicate
                        if (!colorToPlanes.count(pseudo_hash_center))
                        {
                            auto plane = new ORB_SLAM3::Plane(pKFini, mpAtlas);
                            colorToPlanes[pseudo_hash_center] = plane;
                        }
                    }

                    // link map point to the plane, and set ownership
                    colorToPlanes[pseudo_hash_center]->add_landmark(lm);
                }
            }
        }

//        if (_setVerbose)
//        {
//            // spdlog::info("\t \t | number of planes (colors) initialized by this keyframe: {}", colorToPlanes.size());
//        }

        if (colorToPlanes.empty())
        {
            return false;
        }

        return true;
    }



    //MYPLAN
    bool Planar_Mapping_module::create_new_plane(eigen_alloc_unord_map<long, ORB_SLAM3::Plane *> &colorToPlanes)
    {
//        if (_setVerbose)
//        {
//            spdlog::info("-- PlanarMapping -- trying to create plane");
//        }

//        auto before = std::chrono::high_resolution_clock::now();

        bool hasNewPlanes = false;
        for (auto &color_plane_pair : colorToPlanes)
        {
            if (color_plane_pair.second->get_num_landmarks() < MIN_NUMBER_POINTS_BEFORE_RANSAC) // 12
            {
                delete color_plane_pair.second;
                color_plane_pair.second = nullptr;
                continue;
            }

            // Conditional ternary operator
            //myplan 修改这个参数定义不同的图优化策略
            if (estimate_plane_sequential_RANSAC(color_plane_pair.second))
//            if (estimate_plane_sequential_Graph_cut_RANSAC(color_plane_pair.second))
            {
//                ? estimate_plane_sequential_Graph_cut_RANSAC(color_plane_pair.second)
//                 estimate_plane_sequential_RANSAC(color_plane_pair.second);


                hasNewPlanes = true;
                color_plane_pair.second->set_valid(); // set valid, but plane need to be refined

                if (_setVerbose)
                {
                    double a, b, c, d;
                    color_plane_pair.second->get_equation(a, b, c, d);
//                    std::cout<<" new plane detected: ({}, {}, {}, {})"<< a<< b<< c<< d<<std::endl;
                }

                // save to map database
                mpAtlas->add_landmark_plane(color_plane_pair.second);
            }
            else
            {
                color_plane_pair.second->remove_landmarks_ownership();

                // delete pointer first, then set as null
                delete color_plane_pair.second;
                color_plane_pair.second = nullptr;

//                if (_setVerbose)
//                {
//                    spdlog::info("\t \t | estimate plane through RANSAC failed");
//                }
            }
        }
//        std::cout<<"清除颜色===="<<std::endl;
        colorToPlanes.clear();

//        auto after = std::chrono::high_resolution_clock::now();
//        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(after - before).count();

//        if (_setVerbose)
//        {
//            spdlog::info("\t \t | process time (RANSAC+SVD): {}ms", duration);
//        }

        return hasNewPlanes;
    }
//法使用顺序RANSAC算法对平面进行估计。它通过迭代随机选择地标点，估计初始平面参数并计算拟合误差，
// 然后根据设定的阈值和模式进行优化，找到与估计平面最佳匹配的内点集合和参数。
// 如果估计成功，函数返回 true；否则，返回 false。
    bool Planar_Mapping_module::estimate_plane_sequential_RANSAC(ORB_SLAM3::Plane *plane)
    {
        std::vector<ORB_SLAM3::MapPoint *> lms = plane->get_landmarks();

//        if (_setVerbose)
//        {
//            spdlog::info("\t \t | ----");
//            spdlog::info("\t \t | estimate plane RANSAC");
//            spdlog::info("\t \t | number of map points linked to the plane: {} ", lms.size());
//        }

        if (lms.empty())
        {
            return false;
        }

        if (lms.size() < POINTS_PER_RANSAC)
        {
//            if (_setVerbose)
//            {
//                spdlog::info("\t \t | Not enough points!");
//            }

            return false;
        }

        // randomly choose points for RANSAC, save as index
        double best_error = std::numeric_limits<double>::max();
        std::vector<int> inliers_list;
        std::vector<int> best_inliers_list;
        bool best_found = false;
        double a, b, c, d;
        double a_best, b_best, c_best, d_best;
        std::function<int()> rnd = std::bind(std::uniform_int_distribution<>(0, lms.size() - 1), std::mt19937(std::random_device()()));
        for (unsigned int i = 0; i < _iterationsCount; i++)
        {
            std::vector<int> indexes;
            while (indexes.size() < POINTS_PER_RANSAC)
            {
                int index = rnd(); // random index
                if (!lms[index]->isBad())
                {
                    indexes.push_back(index);
                }
            }

            if (indexes.empty())
            {
                break;
            }

            // [1] estimate initial plane parameter from random selected lms
            double residual = estimate_plane_SVD(lms, indexes, a, b, c, d);
            if (residual < best_error)
            {
                best_error = residual;
            }
            a_best = a;
            b_best = b;
            c_best = c;
            d_best = d;
            plane->set_equation(a_best, b_best, c_best, d_best);
            plane->set_best_error(residual);

            // [2] find inlier 3D points by calculating the distance between lms to the plane
            inliers_list.clear();
            for (unsigned int j = 0; j < lms.size(); j++)
            {
                if (lms[j]->isBad())
                {
                    continue;
                }

                // calculate point-plane distance
                double point2plane_error = plane->calculate_distance(Converter::toVector3d(lms[j]->GetWorldPos()));
                if (point2plane_error < _planar_distance_thresh)
                {
                    inliers_list.push_back(j);
                }
            }

            // [3] try to update best parameters which minimize the distance between all plane-linked (3D) points to the estimated plane
            if (_ransacMode == RANSAC_SMALLEST_DISTANCE_ERROR)
            {
                const double inlier_ratio = double(inliers_list.size()) / double(lms.size());

                if (inlier_ratio > _inliers_ratio_thr &&
                    inliers_list.size() >= POINTS_PER_RANSAC)
                {
                    // estimate plane parameters again using all the inliers found before
                    auto error = estimate_plane_SVD(lms, inliers_list, a, b, c, d);

                    // if new error is better than before, then update parameters
                    if (error < best_error)
                    {
                        best_error = error;
                        a_best = a;
                        b_best = b;
                        c_best = c;
                        d_best = d;
                        plane->set_equation(a_best, b_best, c_best, d_best);
                        plane->set_best_error(best_error);

                        // find best inlier 3D points
                        best_inliers_list.clear();
                        for (unsigned int z = 0; z < inliers_list.size(); z++)
                        {
                            int index = inliers_list[z];
                            best_inliers_list.push_back(std::move(index));
                        }

                        best_found = true;

                        // early break if the error is small enough
                        if (error < _final_error_thresh)
                        {
//                            if (_setVerbose)
//                            {
//                                spdlog::info("\t \t | early break in RANSAC when error < _final_error_thresh");
//                            }

                            break;
                        }
                    }
                }
            }
            else if (_ransacMode == RANSAC_HIGHEST_INLIER_RATIO)
            {
                // FW: for now, we try to find plane which minimize the geometric threshold (point2plane distance)
                //  this mode indicates try to find more 3D lms to fit the plane (high inlier ratio), which is not used so far
            }
        } // end of ransac iteration

        if (!best_found)
        {
//            if (_setVerbose)
//            {
//                spdlog::info("\t \t | plane not founded");
//            }

            return false;
        }
        else if (best_error > _final_error_thresh)
        {
//            if (_setVerbose)
//            {
////                spdlog::info("\t \t | error is higher than threshold");
//            }

            return false;
        }

        // [4] assign the best lms to the plane
        std::vector<ORB_SLAM3::MapPoint *> plane_best_inlier_map_points;
        for (unsigned int i = 0; i < best_inliers_list.size(); i++)
        {
            auto lm = lms[best_inliers_list[i]];
            if (lm->isBad())
            {
                continue;
            }

            double dis = plane->calculate_distance((Converter::toVector3d(lm->GetWorldPos())));
            if (dis < _planar_distance_thresh)
            {
                plane_best_inlier_map_points.push_back(lm);
            }
        }

//        if (_setVerbose)
//        {
////            std::cout<<"好的面内点数量为======="<<plane_best_inlier_map_points.size()<<std::endl;
//        }

        plane->remove_landmarks_ownership();
        plane->set_landmarks(plane_best_inlier_map_points);
        plane->set_landmarks_ownership();

        return true;
    }
//评估拟合程度方法使用 RANSAC 算法对平面的参数进行更新。它随机选择一组地标点用于估计初始平面参数，
// 并通过迭代优化来找到与估计平面最佳匹配的内点集合和参数。这样可以提高平面的准确性和鲁棒性。
// 如果更新成功，函数返回 true；否则，返回 false，并将平面标记为需要细化。
    bool Planar_Mapping_module::update_plane_via_RANSAC(Plane *plane)
    {
        std::vector<ORB_SLAM3::MapPoint *> lms = plane->get_landmarks();

        if (lms.empty())
        {
            return false;
        }

        if (lms.size() < POINTS_PER_RANSAC)
        {
            plane->set_invalid();
            return false;
        }

        // randomly choose points for RANSAC, save as index
        double best_error = plane->get_best_error();
        std::vector<int> inliers_list;
        std::vector<int> best_inliers_list;
        bool best_found = false;
        double a, b, c, d;
        double a_best, b_best, c_best, d_best;
        std::function<int()> rnd = std::bind(std::uniform_int_distribution<>(0, lms.size() - 1), std::mt19937(std::random_device()()));
        for (unsigned int i = 0; i < _iterationsCount; i++)
        {
            std::vector<int> indexes;
            // we use more than half of the points to update plane parameters i.e. after merge
            while (indexes.size() < lms.size() * 0.8)
            {
                int index = rnd(); // random index
                if (!lms[index]->get_Owning_Plane())
                {
                    continue;
                }

                if (!lms[index]->isBad())
                {
                    indexes.push_back(index);
                }
            }

            if (indexes.empty())
            {
                break;
            }

            // [1] estimate initial plane parameter from random selected lms
            double residual = estimate_plane_SVD(lms, indexes, a, b, c, d);
            if (residual < best_error)
            {
                best_error = residual;
            }
            a_best = a;
            b_best = b;
            c_best = c;
            d_best = d;
            plane->set_equation(a_best, b_best, c_best, d_best);
            plane->set_best_error(residual);

            // [2] find inlier 3D points by calculating the distance between lms to the plane
            inliers_list.clear();
            for (unsigned int j = 0; j < lms.size(); j++)
            {
                if (lms[j]->isBad())
                {
                    continue;
                }

                // calculate point-plane distance
                double point2plane_error = plane->calculate_distance(Converter::toVector3d(lms[j]->GetWorldPos()));
                if (point2plane_error < _planar_distance_thresh)
                {
                    inliers_list.push_back(j);
                }
            }

            // [3] try to update best parameters which minimize the distance between all plane-linked (3D) points to the estimated plane
            if (inliers_list.size() >= POINTS_PER_RANSAC)
            {
                auto error = estimate_plane_SVD(lms, inliers_list, a, b, c, d);

                // early break if the error is smaller than before
                if (error < best_error)
                {
                    best_error = error;
                    a_best = a;
                    b_best = b;
                    c_best = c;
                    d_best = d;
                    plane->set_equation(a_best, b_best, c_best, d_best);
                    plane->set_best_error(best_error);

                    // find best inlier 3D points
                    best_inliers_list.clear();
                    for (unsigned int z = 0; z < inliers_list.size(); z++)
                    {
                        int index = inliers_list[z];
                        best_inliers_list.push_back(std::move(index));
                    }

                    best_found = true;
                }
            }
        } // end of ransac iteration

        if (!best_found || best_error > _final_error_thresh)
        {
            plane->set_need_refinement();
            return false;
        }

        // [4] assign the best lms to the plane
        std::vector<ORB_SLAM3::MapPoint *> plane_best_inlier_map_points;
        for (unsigned int i = 0; i < best_inliers_list.size(); i++)
        {
            auto lm = lms[best_inliers_list[i]];

            if (lm->isBad())
            {
                continue;
            }

            double dis = plane->calculate_distance(Converter::toVector3d(lm->GetWorldPos()));
            if (dis < _planar_distance_thresh)
            {
                plane_best_inlier_map_points.push_back(lm);
            }
        }

        if (plane_best_inlier_map_points.size() < POINTS_PER_RANSAC)
        {
            plane->set_invalid();
            return false;
        }

        plane->remove_landmarks_ownership();
        plane->set_landmarks(plane_best_inlier_map_points);
        plane->set_landmarks_ownership();

        return true;
    }
//面的奇异值分解
//该方法使用奇异值分解对给定的一组地图点进行平面拟合，估计平面的法向量和与原点的距离，并计算拟合的平均残差。
// 这些平面参数可以用于后续的平面优化和更新过程
    double Planar_Mapping_module::estimate_plane_SVD(std::vector<ORB_SLAM3::MapPoint *> const &all_plane_points, std::vector<int> indexes, double &a, double &b, double &c, double &d)
    {
        std::vector<ORB_SLAM3::MapPoint *> candidate_points;
        candidate_points.reserve(indexes.size());
        for (unsigned int i = 0; i < indexes.size(); i++)
        {
            candidate_points.push_back(all_plane_points.at(indexes[i]));
        }

        // build a linear system for plane
        Eigen::Matrix<double, 3, Eigen::Dynamic> X(3, candidate_points.size());

        for (unsigned int i = 0; i < candidate_points.size(); i++)
        {
            X.col(i) = Converter::toVector3d(candidate_points[i]->GetWorldPos());
        }

        // FW: why we need to centralize the coordinates?
        Vec3_t centroid = X.rowwise().mean();
        Eigen::Matrix<double, 3, Eigen::Dynamic> centered_X = X.colwise() - centroid;

        // Singular Value Decomposition
        Eigen::JacobiSVD<Eigen::Matrix<double, 3, Eigen::Dynamic>> svd(centered_X, Eigen::ComputeFullU);
        Eigen::Matrix3d const U = svd.matrixU();
        Eigen::Vector3d normal = U.col(2).normalized();
        d = -normal.dot(centroid);
        a = normal(0);
        b = normal(1);
        c = normal(2);

        // not really a residual because we average it according to the total n# of points
        // as having a bigger residual with more points is expected...
        Eigen::Matrix<double, Eigen::Dynamic, 1> const b_vector = Eigen::Matrix<double, Eigen::Dynamic, 1>::Ones(X.cols(), 1) * d;
        double const residual = ((X.transpose() * normal) + b_vector).norm() / candidate_points.size();

        return std::fabs(residual);
    }
//平面细化处理
    void Planar_Mapping_module::refinement()
    {
        std::lock_guard<std::mutex> lock(_mtxPlaneMutex);

        if (mpAtlas->get_num_landmark_planes() < 2)
        {
            return;
        }

        // merge if two plane have similar normal vector or very close to each other, etc.
        if (merge_planes())
        {
//            if (_setVerbose)
//            {
//              std::cout<<"面合并成功==="<<std::endl;
//            }
        }

        refine_planes(); // re-estimate plane parameters for the plane which need to be refined, e.g. after merge
        refine_points(); // TODO: this should be only down in local BA?
    }
//它用于合并地图中的平面。
    bool Planar_Mapping_module::merge_planes()
    {
        auto all_planes = mpAtlas->get_all_landmark_planes();

        if (all_planes.empty())
        {
            return false;
        }

        // at least two planes should exist
        if (all_planes.size() < 2)
        {
            return false;
        }

        // Two constants are used to decide whether or not two planes are equal
        // DOT_PRODUCT_THRESHOLD is used when comparing the angle between two planes
        // OFFSET_DELTA_THRESHOLD is used when comparing the offset between two planes
        OFFSET_DELTA_THRESHOLD = _offset_delta_factor * _planar_distance_thresh;

        std::vector<ORB_SLAM3::Plane *> planes_will_be_removed; // a container used to aggregate merged planes or outlier planes
        bool was_merged = false;
        for (unsigned int i = 0; i < all_planes.size(); i++)
        {
            // merge_parent: all_planes[i]
            if (!all_planes[i]->is_valid())
            {
                continue;
            }

            // merge_candidate: all_planes[j]
            for (unsigned int j = i + 1; j < all_planes.size(); j++)
            {
                if (!all_planes[j]->is_valid())
                {
                    j++;
                    if (j >= all_planes.size())
                    { // avoid out of range issue
                        break;
                    }
                }

                // get plane normal
                Vec3_t parent_normal = all_planes[i]->get_normal();
                Vec3_t candidate_normal = all_planes[j]->get_normal();

                // get plane normal's norm
                double parent_normalizer = 1.0 / all_planes[i]->get_normal_norm();
                double candidate_normalizer = 1.0 / all_planes[j]->get_normal_norm();

                // get plane distance to the origin
                double parent_offset = all_planes[i]->get_offset();
                double candidate_offset = all_planes[j]->get_offset();

                // calculate geometric thresholds
                double offset_delta = parent_offset * parent_normalizer - candidate_offset * candidate_normalizer;
                double normalized_dot_product = parent_normal.dot(candidate_normal) * (parent_normalizer * candidate_normalizer);

                if (std::fabs(offset_delta) > OFFSET_DELTA_THRESHOLD &&
                    std::fabs(normalized_dot_product) < DOT_PRODUCT_THRESHOLD)
                {
                    // if two planes are not close to each other, or vary a lot in terms of normal
                    continue;
                }
                else if (std::fabs(offset_delta) < OFFSET_DELTA_THRESHOLD &&
                         std::fabs(normalized_dot_product) > DOT_PRODUCT_THRESHOLD)
                {
                    // if two planes are close to each other, and normals are nearly parallel (dot product ~= 1)
                    // merge
                    if (all_planes[i]->get_num_landmarks() > all_planes[j]->get_num_landmarks())
                    {
                        all_planes[i]->merge(all_planes[j]);             // transfer the point' ownership to the merge_parent
                        planes_will_be_removed.push_back(all_planes[j]); // aggregate planes which were merged

                        // update information of merged plane
                        update_plane_via_RANSAC(all_planes[i]);
                        all_planes[i]->set_need_refinement();
                    }
                    else
                    {
                        all_planes[j]->merge(all_planes[i]);
                        planes_will_be_removed.push_back(all_planes[i]);

                        // update information of merged plane
                        update_plane_via_RANSAC(all_planes[j]);
                        all_planes[j]->set_need_refinement();
                    }

                    was_merged = true;
                }
            }
        }

        // remove merged planes (which labeled as outlier before) from map_database
        if (was_merged)
        {
            for (auto &pl : planes_will_be_removed)
            {
                mpAtlas->erase_landmark_plane(pl);
            }
        }

        return was_merged;
    }
//检查平面的有效性和细化标志来决定是否需要更新平面方程，以及是否删除地标点较少的小平面
    bool Planar_Mapping_module::refine_planes()
    {
        bool was_changed = false;
        std::vector<ORB_SLAM3::Plane *> planes_will_be_erased;

        for (auto &plane : mpAtlas->get_all_landmark_planes())
        {
            if (!plane->is_valid())
            {
                planes_will_be_erased.push_back(plane);
                continue;
            }

            if (plane->need_refinement())
            {
                // a merged plane will be labeled as need refinement
                // update the plane equation using RANSAC
                if (update_plane_via_RANSAC(plane))
                {
                    plane->set_refinement_is_done();
                    was_changed = true;
                }
            }
            else
            {
                // check if the plane is very small
                if (plane->get_num_landmarks() < 2 * POINTS_PER_RANSAC)
                {
                    planes_will_be_erased.push_back(plane);
                }
            }
        }

        // erase outlier plane from map_database
        if (!planes_will_be_erased.empty())
        {
            for (auto &pl : planes_will_be_erased)
            {
                pl->remove_landmarks_ownership();
                mpAtlas->erase_landmark_plane(pl);
            }
        }

        if (_setVerbose)
        {
            if (was_changed)
            {
//                spdlog::info("\t \t | plane refined!");
            }
        }

        return was_changed;
    }
//计算地标点到平面的距离，并根据距离变化来更新地标点的位置，使其尽可能接近平面。这有助于改善地图的几何结构和精度。
    bool Planar_Mapping_module::refine_points()
    {
        bool was_changed = false;

        for (auto &plane : mpAtlas->get_all_landmark_planes())
        {
            if (!plane->is_valid() ||
                plane->need_refinement() ||
                plane->get_num_landmarks() < POINTS_PER_RANSAC)
            {
                continue;
            }

            auto lms = plane->get_landmarks();
            for (auto &lm : lms)
            {
                if (lm->isBad() || !lm->get_Owning_Plane())
                {
                    continue;
                }

                Vec3_t pos_w = Converter::toVector3d(lm->GetWorldPos());
                double dist_old = plane->calculate_distance(pos_w);
                if (std::fabs(dist_old) > 0.0)
                {
                    // if a point is assumed on the plane, then it should move along the normal direction
                    // and minimize the distance between plane
                    // update 3D coordinates along the normal direction
                    Vec3_t n = plane->get_normal().normalized();
                    pos_w -= n * dist_old;
                    double dist_new = plane->calculate_distance(pos_w);
                    if (dist_new < _planar_distance_thresh &&
                        dist_new < dist_old)
                    {
                        lm->SetWorldPos(Converter::toCvMat(pos_w));//将eigien 3d>>cv mat
                        was_changed = true;
                    }
                }
            }
        }

        if (_setVerbose)
        {
//            if (was_changed)
//            {
//                std::cout<<"点细化成功==="<<std::endl;
//            }
        }

        return was_changed;
    }
    //imp 这个可能好一点,改动太多懒得改了
//该方法使用 Graph-cut RANSAC 算法对平面进行估计，并通过最小化与平面的距离来选择内点集合
// 。然后，使用内点集合重新估计平面参数，并更新平面的地标点列表
//data 相当于orb3的MapPoint
    bool Planar_Mapping_module::estimate_plane_sequential_Graph_cut_RANSAC(Plane *plane)
    {
        std::vector<ORB_SLAM3::MapPoint *> lms = plane->get_landmarks();



        if (lms.empty())
        {
            return false;
        }

        if (lms.size() < POINTS_PER_RANSAC)
        {

            return false;
        }

        // 3D points in cv::Mat
        cv::Mat points(0, 3, CV_64F), point(1, 3, CV_64F);
        for (unsigned int i = 0; i < lms.size(); i++)
        {
            auto lm = lms[i];
            if (lm->isBad())
            {
                continue;
            }

            auto pos_w = Converter::toVector3d(lm->GetWorldPos());
            point.at<double>(0) = pos_w(0);
            point.at<double>(1) = pos_w(1);
            point.at<double>(2) = pos_w(2);
            points.push_back(point);
        }

        // adaptive parameters
        if (adaptive_number_ != 0)
        {
            _inlier_outlier_threshold = _planar_distance_thresh;
            _sphere_radius = adaptive_number_ * _final_error_thresh; // used to construct neighborhood graph
        }

        // [1] Apply Graph-cut RANSAC
        gcransac::utils::Default3DPlaneEstimator estimator; // The estimator used for the pose fitting
        gcransac::Plane3D model;                            // The estimated model parameters

        // Initialize the neighborhood used in Graph-cut RANSAC
        // FW: TODO: the graph may also be constructed by 3D grid (GridNeighborhoodGraph<3>)?
        gcransac::neighborhood::FlannNeighborhoodGraph neighborhood(&points,         // The data points
                                                                    _sphere_radius); // The radius of the neighborhood sphere used for determining the neighborhood structure

        // Initialize the samplers
        // FW: TODO: should the sampler of NAPSAC or Progressive-NAPSAC used?
        gcransac::sampler::UniformSampler main_sampler(&points);               // The main sampler is used inside the local optimization
        gcransac::sampler::UniformSampler local_optimization_sampler(&points); // The local optimization sampler is used inside the local optimization

        // Checking if the samplers are initialized successfully.
        if (!main_sampler.isInitialized() ||
            !local_optimization_sampler.isInitialized())
        {
            fprintf(stderr, "One of the samplers is not initialized successfully.\n");
            return false;
        }

        // Initializing SPRT test
        gcransac::preemption::SPRTPreemptiveVerfication<gcransac::utils::Default3DPlaneEstimator> preemptive_verification(
                points,                          // The set of 3D points
                estimator,                       // The linear estimator object (2D line or 3D plane)
                _minimum_inlier_ratio_for_sprt); // The minimum acceptable inlier ratio. Models with fewer inliers will not be accepted.

        gcransac::GCRANSAC<gcransac::utils::Default3DPlaneEstimator,
        gcransac::neighborhood::FlannNeighborhoodGraph,
        gcransac::MSACScoringFunction<gcransac::utils::Default3DPlaneEstimator>,
        gcransac::preemption::SPRTPreemptiveVerfication<gcransac::utils::Default3DPlaneEstimator>>
                gcransac;
        gcransac.setFPS(_fps);
        gcransac.settings.threshold = _inlier_outlier_threshold;                // The inlier-outlier threshold
        gcransac.settings.spatial_coherence_weight = _spatial_coherence_weight; // The weight of the spatial coherence term
        gcransac.settings.confidence = _confidence;                             // The required confidence in the results
        gcransac.settings.max_iteration_number = 5000;                          // The maximum number of iterations
        gcransac.settings.min_iteration_number = 20;                            // The minimum number of iterations

        // Start GC-RANSAC
        gcransac.run(points,                      // The normalized points
                     estimator,                   // The estimator
                     &main_sampler,               // The sample used for selecting minimal samples in the main iteration
                     &local_optimization_sampler, // The sampler used for selecting a minimal sample when doing the local optimization
                     &neighborhood,               // The neighborhood-graph
                     model,                       // The obtained model parameters
                     preemptive_verification);

        // Get the statistics of the results
        const gcransac::utils::RANSACStatistics &statistics = gcransac.getRansacStatistics();


        std::cout<<"model.descriptor(0, 0)=="<<model.descriptor(0, 0);
        std::cout<<"model.descriptor(0, 1)=="<<model.descriptor(1, 0);
        std::cout<<"model.descriptor(0, 2)=="<<model.descriptor(2, 0);
        std::cout<<"model.descriptor(0, 3)=="<<model.descriptor(3, 0);

        plane->set_equation(model.descriptor(0, 0), model.descriptor(1, 0), model.descriptor(2, 0), model.descriptor(3, 0));

        // reject model if inlier ratio is lower than i.e. 70%
        const double inlier_ratio = double(statistics.inliers.size()) / double(lms.size());
        if (inlier_ratio < _inliers_ratio_thr && (statistics.inliers.size() < POINTS_PER_RANSAC))
        {
            return false;
        }

        // [2] find inlier 3D points by calculating the distance between lms to the plane
        double best_error = std::numeric_limits<double>::max();
        std::vector<int> inliers_list;
        std::vector<ORB_SLAM3::MapPoint *> plane_best_inlier_map_points;
        for (unsigned int i = 0; i < statistics.inliers.size(); i++)
        {
            inliers_list.push_back(statistics.inliers[i]); // do this just for convenience because our code was implemented with "int" instead of "unsigned int"
            auto lm = lms[statistics.inliers[i]];
            plane_best_inlier_map_points.push_back(lm);
        }

        // [3] try to update best parameters which minimize the distance between all plane-linked (3D) points to the estimated plane
        // estimate plane parameters again using all the inliers found before
        double a, b, c, d;
        auto error = estimate_plane_SVD(lms, inliers_list, a, b, c, d);
        if (error < best_error)
        {
            best_error = error;
            plane->set_equation(a, b, c, d);
            plane->set_best_error(best_error);
        }

        // [4] assign the best lms to the plane
        plane->remove_landmarks_ownership();
        plane->set_landmarks(plane_best_inlier_map_points);
        plane->set_landmarks_ownership();

        return true;
    }
//imp

    void Planar_Mapping_module::load_configuration()
    {


        _use_graph_cut = false;
        _setVerbose = true;

        _iterationsCount = 50;
        _inliers_ratio_thr =0.7;
        MIN_NUMBER_POINTS_BEFORE_RANSAC = 12;
        POINTS_PER_RANSAC = 12;

        _check_3x3_window = true;

        DOT_PRODUCT_THRESHOLD = 0.8;
        _offset_delta_factor = 6;

        PLANAR_DISTANCE_CORRECTION =  0.02;
        FINAL_ERROR_CORRECTION = 0.01;

        _confidence =  0.99;
        _fps = 30;
        _inlier_outlier_threshold =  0.02;
        _spatial_coherence_weight =  0.6;
        _sphere_radius = 0.02;
        _minimum_inlier_ratio_for_sprt =  0.95;
        adaptive_number_ = 2;
    }
} //