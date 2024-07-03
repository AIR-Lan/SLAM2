

#include "MapDrawer.h"
#include "MapPoint.h"
#include "MapLine.h"
#include "KeyFrame.h"
#include <pangolin/pangolin.h>
#include <mutex>
#include "ProbabilityMapping.h"
#include "Object.h"
#include "landmark_plane.h"

namespace ORB_SLAM3
{


MapDrawer::MapDrawer(Atlas* pAtlas, const string &strSettingPath):mpAtlas(pAtlas)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    bool is_correct = ParseViewerParamFile(fSettings);
    _mPlaneColors = {
            {0.0, 1.0, 0.0},
            {0.0, 0.0, 1.0},
            {1.0, 0.0, 0.0},
            {0.5, 0.1, 0.1},
            {0.1, 0.5, 0.1},
            {0.1, 0.1, 0.5},
            {0.1, 0.5, 0.5},
            {0.5, 0.5, 0.1},
            {0.5, 0.1, 0.5},
            {0.7, 0.3, 0.3},
            {0.3, 0.7, 0.3},
            {0.3, 0.3, 0.7},
            {0.3, 0.7, 0.7},
            {0.7, 0.7, 0.3},
            {0.7, 0.3, 0.7},
            {0.3, 0.3, 0.3},
            {0.2, 0.5, 0.7},
            {0.2, 0.7, 0.5},
            {0.7, 0.5, 0.2},
            {0.5, 0.7, 0.2},
            {0.7, 0.2, 0.5},
            {0.5, 0.2, 0.7},
            {0.8, 0.4, 0.4},
            {0.4, 0.8, 0.4},
            {0.4, 0.4, 0.8},
    };
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
}

bool MapDrawer::ParseViewerParamFile(cv::FileStorage &fSettings)
{
    bool b_miss_params = false;

    cv::FileNode node = fSettings["Viewer.KeyFrameSize"];
    if(!node.empty())
    {
        mKeyFrameSize = node.real();
    }
    else
    {
        std::cerr << "*Viewer.KeyFrameSize parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.KeyFrameLineWidth"];
    if(!node.empty())
    {
        mKeyFrameLineWidth = node.real();
    }
    else
    {
        std::cerr << "*Viewer.KeyFrameLineWidth parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.GraphLineWidth"];
    if(!node.empty())
    {
        mGraphLineWidth = node.real();
    }
    else
    {
        std::cerr << "*Viewer.GraphLineWidth parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.PointSize"];
    if(!node.empty())
    {
        mPointSize = node.real();
    }
    else
    {
        std::cerr << "*Viewer.PointSize parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.CameraSize"];
    if(!node.empty())
    {
        mCameraSize = node.real();
    }
    else
    {
        std::cerr << "*Viewer.CameraSize parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.CameraLineWidth"];
    if(!node.empty())
    {
        mCameraLineWidth = node.real();
    }
    else
    {
        std::cerr << "*Viewer.CameraLineWidth parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    return !b_miss_params;
}

void MapDrawer::DrawMapPoints()
{
    const vector<MapPoint*> &vpMPs = mpAtlas->GetAllMapPoints();
    const vector<MapPoint*> &vpRefMPs = mpAtlas->GetReferenceMapPoints();

    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())
        return;

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,0.0);

    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        cv::Mat pos = vpMPs[i]->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
    }
    glEnd();

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(1.0,0.0,0.0);

    for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        cv::Mat pos = (*sit)->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));

    }

    glEnd();
}

void MapDrawer::DrawMapLines()
{

    std::vector<ORB_SLAM3::Plane *> planes;
    MapDrawer::get_landmark_planes(planes);//获取当前atlas所有的平面
    if (planes.empty())
        return;
    std::vector<Vec3_t> plane_norm_all;  // 用来保存所有的 每个平面的法向量
    for (auto const &plane : planes)
    {
        if (!plane->is_valid()) {
            continue;
        }

        Vec3_t plane_norm = plane->get_normal().normalized();
        // 将plane_norm添加到plane_normals中
        plane_norm_all.push_back(plane_norm);
    }


    //mynote 画地图线
    const vector<MapLine*> &vpMLs = mpAtlas->GetAllMapLines();
    const vector<MapLine*> &vpRefMLs = mpAtlas->GetReferenceMapLines();

    set<MapLine*> spRefMLs(vpRefMLs.begin(), vpRefMLs.end());

    if(vpMLs.empty())
        return;

    float mLineSize = 4.0;

    glLineWidth(mLineSize);
    glColor3f(0.4, 0.35, 0.8);//所有地图线的颜色为黑色
    glBegin(GL_LINES);
    //对地图中所有的地图线进行遍历
    for(size_t i=0, iend=vpMLs.size(); i<iend;i++)
    {

        if(vpMLs[i]->isBad() || spRefMLs.count(vpMLs[i]))
            continue;
        Vector6d sep = vpMLs[i]->GetWorldPos();
        Vector3d sp_eigen = sep.head(3);//地图线起点
        Vector3d ep_eigen = sep.tail(3);//地图线终点

        // 计算线段的向量
        Vector3d lineVector = ep_eigen - sp_eigen;//地图线的方向向量
        //计算线段的长度
        double length = lineVector.norm();
        double thre_line = 5;
        // 计算单位向量，以便计算角度
        Vector3d line_unit_vector = lineVector.normalized();//地图线的单位方向向量
        //对所有的平面法向量进行遍历，计算当前这条线和所有法向量的角度值，
        bool draw_line = false;
        for(auto &plan_norm_single : plane_norm_all)
        {
            // 计算当前这条线这个向量与一个平面法线的夹角的cos值
            double cos_theta = line_unit_vector.dot(plan_norm_single);
            // 转换为角度
            double theta = acos(cos_theta) * 180.0 / M_PI;
            // 如果角度在[0, 10]或者[170, 180]范围内，我们保留这个线段
            if ((theta >= 0 && theta <= 10) || (theta >= 170 && theta <= 180))
            {
                draw_line =true;
            }
        }

        if (draw_line)
        {
            glVertex3f(static_cast<float>(sp_eigen(0)),static_cast<float>(sp_eigen(1)),static_cast<float>(sp_eigen(2)));
            glVertex3f(static_cast<float>(ep_eigen(0)),static_cast<float>(ep_eigen(1)),static_cast<float>(ep_eigen(2)));
        }

    }
    glEnd();
//*******************画参考地图线*************************
    glPointSize(4.0f);
    //参考地图线设为淡蓝色
    glColor3f(0.4, 0.35, 0.8);
    glBegin(GL_LINES);
    //对地图中参考地图线进行遍历
    for(set<MapLine*>::iterator sit=spRefMLs.begin(), send=spRefMLs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        Vector6d sep = (*sit)->GetWorldPos();
        Vector3d sp_eigen = sep.head(3);//地图线起点
        Vector3d ep_eigen = sep.tail(3);//地图线终点

        // 计算线段的向量
        Vector3d lineVector = ep_eigen - sp_eigen;//地图线的方向向量
        //计算线段的长度
        double length = lineVector.norm();
        double thre_line = 5;
        // 计算单位向量，以便计算角度
        Vector3d line_unit_vector = lineVector.normalized();//地图线的单位方向向量
        //对所有的平面法向量进行遍历，计算当前这条线和所有法向量的角度值，
        bool draw_line = false;
        for(auto &plan_norm_single : plane_norm_all)
        {
            // 计算当前这条线这个向量与一个平面法线的夹角的cos值
            double cos_theta = line_unit_vector.dot(plan_norm_single);
            // 转换为角度
            double theta = acos(cos_theta) * 180.0 / M_PI;
            // 如果角度在[0, 10]或者[170, 180]范围内，我们保留这个线段
            if ((theta >= 0 && theta <= 10) || (theta >= 170&& theta <= 180))
            {
                draw_line =true;
            }
        }


        // 如果角度在[0, 10]或者[170, 180]范围内，我们保留这个线段
        if (draw_line)
        {
            glVertex3f(static_cast<float>(sp_eigen(0)),static_cast<float>(sp_eigen(1)),static_cast<float>(sp_eigen(2)));
            glVertex3f(static_cast<float>(ep_eigen(0)),static_cast<float>(ep_eigen(1)),static_cast<float>(ep_eigen(2)));
        }
//        std::cout<<"线的四个顶点值=="<<static_cast<float>(sp_eigen(0)) << static_cast<float>(sp_eigen(1))<< static_cast<float>(sp_eigen(2))<<std::endl;
//        std::cout<<"线的四个顶点值=="<<static_cast<float>(ep_eigen(0)) << static_cast<float>(ep_eigen(1))<< static_cast<float>(ep_eigen(2))<<std::endl;
//            glVertex3f(static_cast<float>(sp_eigen(0)),static_cast<float>(sp_eigen(1)),static_cast<float>(sp_eigen(2)));
//            glVertex3f(static_cast<float>(ep_eigen(0)),static_cast<float>(ep_eigen(1)),static_cast<float>(ep_eigen(2)));

    }
    glEnd();
}

    //在地图上画面 myplan mynote 这段函数在 viewer
// FW: myplan
void MapDrawer::draw_landmarks_plane()
{
//    if (!*_menu_show_plane)
//    {
//        return;
//    }

    std::vector<ORB_SLAM3::Plane *> planes;
    MapDrawer::get_landmark_planes(planes);




    if (planes.empty())
        return;

    auto PlaneColors = MapDrawer::get_available_color();
    auto c = PlaneColors.begin();

    //mynote

    for (auto const &plane : planes)
    {
        if (!plane->is_valid())
        {
            continue;
        }
//        //mynote

        Vec3_t plane_norm = plane->get_normal();
        //将所有的平面法向量保存
        all_plane_normals.push_back(plane_norm);



        auto const map_pts = plane->get_landmarks();

        if (map_pts.empty())
        {
            continue;
        }

        if (c == PlaneColors.end())
        {
//             std::cout<<"Gotta get more colors!?"<<std::endl;
            c = PlaneColors.begin();
        }

        auto const &color = *(c++);

        // assign a color to the plane
        if (!plane->_has_color)
        {
            plane->_b = color._b;
            plane->_r = color._r;
            plane->_g = color._g;

            plane->_has_color = true;
        }

        // simply pick two map points to formulate the base vectors of a plane patch
        Eigen::Vector3d baseVector1{0, 0, 0};
        Eigen::Vector3d baseVector2{0, 0, 0};
        for (unsigned int i = 0; i < map_pts.size(); i++)
        {
            if (map_pts[i]->isBad() ||
                (plane->calculate_distance(Converter::toVector3d(map_pts[i]->GetWorldPos())) > plane->get_best_error()))
            {
                continue;
            }

            for (unsigned int j = i + 1; j < map_pts.size() - 1; j++)
            {
                if (map_pts[j]->isBad() ||
                    (plane->calculate_distance(Converter::toVector3d(map_pts[j]->GetWorldPos())) > plane->get_best_error()))
                {
                    continue;
                }

                cv::Mat baseVector_1 = (map_pts[i]->GetWorldPos() - map_pts[j]->GetWorldPos());
                double normValue = cv::norm(baseVector_1);
                cv::Mat normalizedVector = baseVector_1 / normValue;
                baseVector1 = Converter::toVector3d(normalizedVector);//经过标准化平面上两点的向量
                baseVector2 = baseVector1.cross(plane->get_normal().normalized());//在平面上经过标准化和base1和平面法线向量垂直的向量



//                std::cout<<"第一个向量顶点值=="<<static_cast<float>(baseVector1(0))<<"+" << static_cast<float>(baseVector1(1))<<"+"<< static_cast<float>(baseVector1(2))<<std::endl;
//                std::cout<<"第二个向量顶点值=="<<static_cast<float>(baseVector2(0))<<"+" << static_cast<float>(baseVector2(1))<<"+"<< static_cast<float>(baseVector2(2))<<std::endl;


                break;
            }

            break;
        }




        // mynote 画地图面 visualize with plane-patch
        for (auto const &point : map_pts)
        {
            glBegin(GL_QUADS);
            glColor4f(plane->_r, plane->_g, plane->_b, _transparency_alpha); // r,g,b, transparency
//            glColor4f(1.0, 0.0, 0.0, 0.7); // r,g,b, transparency
            if (!point->isBad() && point->get_Owning_Plane())
            {
                // draw the plane patch centered around the point
                auto const tr = Converter::toVector3d(point->GetWorldPos()) + _square_size * (baseVector1 + baseVector2);
                auto const br = Converter::toVector3d(point->GetWorldPos()) + _square_size * (baseVector1 - baseVector2);
                auto const bl = Converter::toVector3d(point->GetWorldPos()) - _square_size * (baseVector1 + baseVector2);
                auto const tl = Converter::toVector3d(point->GetWorldPos()) + _square_size * (baseVector2 - baseVector1);

                glVertex3f(tr(0), tr(1), tr(2));
                glVertex3f(br(0), br(1), br(2));
                glVertex3f(bl(0), bl(1), bl(2));
                glVertex3f(tl(0), tl(1), tl(2));

            }
            glEnd();


        }

        // only visualize the normal and base vectors of the dominate plane/stable plane,
        // e.g. new detected plane will not be visualized, this make the visualization more clean
//        仅可视化主平面/稳定平面的法线和基向量，例如新检测到的平面将不会被可视化，这使得可视化更加清晰
        if (_draw_plane_normal)
        {
            double center_x = 0.0;
            double center_y = 0.0;
            double center_z = 0.0;
            unsigned int num_points = 0;
            for (auto const &point : map_pts)
            {
                if (!point->isBad() && point->get_Owning_Plane())
                {
                    auto a = point->GetWorldPos();
                    auto b = point->GetWorldPos();
                    auto c = point->GetWorldPos();

                    center_x += Converter::toVector3d(a)(0);
                    center_y += Converter::toVector3d(b)(1);
                    center_z += Converter::toVector3d(c)(2);
                    num_points++;
                }
            }

            // update plane centroid information
            center_x = center_x / num_points;
            center_y = center_y / num_points;
            center_z = center_z / num_points;
            Eigen::Vector3d center_point(center_x, center_y, center_z);
            plane->setCentroid(center_point);

            if ((!plane->need_refinement()) && plane->get_num_landmarks() > 36)
            {
                glEnable(GL_LINE_SMOOTH);
                glLineWidth(30.0f);

                glBegin(GL_LINES);//线段的绘制
                // auto const p1 = center_point;
                auto const p2 = center_point + 4 * _square_size * plane->get_normal().normalized();
                auto const p3 = center_point + 4 * _square_size * baseVector1;
                auto const p4 = center_point + 4 * _square_size * baseVector2;

                // draw the plane normal (color is same as the corresponding plane)
                glColor3f(plane->_r, plane->_g, plane->_b);
                glVertex3f(static_cast<float>(center_point(0)),static_cast<float>(center_point(1)), static_cast<float>(center_point(2)));
                glVertex3f(static_cast<float>(p2(0)), static_cast<float>(p2(1)), static_cast<float>(p2(2)));

                // draw the plane base_vector_1 (yellow)
                glColor3f(1.0f, 1.0f, 0.0f);
                glVertex3f(static_cast<float>(center_point(0)), static_cast<float>(center_point(1)), static_cast<float>(center_point(2)));
                glVertex3f(static_cast<float>(p3(0)), static_cast<float>(p3(1)), static_cast<float>(p3(2)));

                // draw the plane base_vector_2 (green)
                glColor3f(0.0f, 1.0f, 0.0f);
                glVertex3f(static_cast<float>(center_point(0)), static_cast<float>(center_point(1)), static_cast<float>(center_point(2)));
                glVertex3f(static_cast<float>(p4(0)), static_cast<float>(p4(1)), static_cast<float>(p4(2)));
                glEnd();//结束定义线段的绘制
            }
        }
    }
}

//TODO 用atlas还是mpmap
//myplan 获取面
unsigned int MapDrawer::get_landmark_planes(std::vector<Plane *> &all_landmark_planes)
{
    all_landmark_planes = mpAtlas->get_all_landmark_planes();
    return mpAtlas->get_num_landmark_planes();
}

std::vector<MapDrawer::PlaneColor> MapDrawer::get_available_color()
{
    std::vector<MapDrawer::PlaneColor> used_planecolor;
    for (auto plane : mpAtlas->get_all_landmark_planes())
    {
        if (!plane->is_valid())
        {
            continue;
        }

        if (plane->_has_color)
        {
//                std::cout<<"面有颜色"<<std::endl;
            used_planecolor.push_back(MapDrawer::PlaneColor(plane->_r, plane->_g, plane->_b));
        }
    }

    std::vector<MapDrawer::PlaneColor> un_used_planecolor;
    for (auto color : _mPlaneColors)
    {
        bool found = false;
        for (auto used_color : used_planecolor)
        {
            if (color._r == used_color._r && color._g == used_color._g && color._b == used_color._b)
                found = true;
        }

        if (!found)
        {
            un_used_planecolor.push_back(MapDrawer::PlaneColor(color._r, color._g, color._b));
        }
    }

    return un_used_planecolor;
}//myplan end




    //myplan 半周密建图
void MapDrawer::DrawSemiDense(const double sigma)
{
    const vector<KeyFrame*> &vpKf = mpAtlas->GetAllKeyFrames();
    if(vpKf.empty())return;

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,1.0,0.0);

    int draw_cnt(0);
    for(size_t i = 0; i < vpKf.size();++i)
    {
        KeyFrame* kf = vpKf[i];
        kf->SetNotEraseDrawer();
        if( kf->isBad() || !kf->semidense_flag_ || !kf->interKF_depth_flag_) {
            kf->SetEraseDrawer();
            continue;
        }

        unique_lock<mutex> lock(kf->mMutexSemiDensePoints);

        draw_cnt ++;
        for(int y = 0; y< kf->im_.rows; y++)
            for(int x = 0; x< kf->im_.cols; x++)
            {
                if (kf->depth_sigma_.at<float>(y,x) > sigma) continue;

                if( kf->depth_map_checked_.at<float>(y,x) > 0.000001 )
                {
                    Eigen::Vector3f Pw  (kf->SemiDensePointSets_.at<float>(y,3*x),
                                         kf->SemiDensePointSets_.at<float>(y,3*x+1),
                                         kf->SemiDensePointSets_.at<float>(y,3*x+2));

                    float b = kf->rgb_.at<uchar>(y, 3*x) / 255.0;
                    float g = kf->rgb_.at<uchar>(y, 3*x+1) / 255.0;
                    float r = kf->rgb_.at<uchar>(y, 3*x+2) / 255.0;
                    glColor3f(r, g, b);

                    glVertex3f( Pw[0],Pw[1],Pw[2]);
                }
            }
        kf->SetEraseDrawer();
    }
    glEnd();
}

void MapDrawer::DrawModel()
{
    const vector<KeyFrame*> &vpKf = mpAtlas->GetAllKeyFrames();
    Model* pModel = mpAtlas->GetModel();
    if(vpKf.empty()) return;
    if(pModel == NULL) return;

    pModel->SetNotErase();

    // get the most recent reconstructed keyframe to texture
    KeyFrame* kfToTexture = NULL;
    KeyFrame* prevKf = NULL;
    for(size_t i = 0; i < vpKf.size();++i) {
        KeyFrame *kf = vpKf[i];
        kf->SetNotEraseDrawer();
        if (kf->isBad()) {
            kf->SetEraseDrawer();
            continue;
        }
        if (prevKf == NULL){
            kfToTexture = kf;
            prevKf = kf;
        } else if (kf->mnId > prevKf->mnId){
            kfToTexture = kf;
            prevKf->SetEraseDrawer();
            prevKf = kf;
        }
    }
    if (kfToTexture == NULL) return;


    static unsigned int frameTex = 0;
    if (!frameTex)
        glGenTextures(1, &frameTex);

    cv::Size imSize = kfToTexture->rgb_.size();

    glBindTexture(GL_TEXTURE_2D, frameTex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    // image are saved in RGB format, grayscale images are converted
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,
                 imSize.width, imSize.height, 0,
                 GL_BGR,
                 GL_UNSIGNED_BYTE,
                 kfToTexture->rgb_.data);


    glEnable(GL_TEXTURE_2D);

    glBegin(GL_TRIANGLES);
    glColor3f(1.0,1.0,1.0);

    for (list<dlovi::Matrix>::const_iterator it = pModel->GetTris().begin(); it != pModel->GetTris().end(); it++) {

        dlovi::Matrix point0 = pModel->GetPoints()[(*it)(0)];
        dlovi::Matrix point1 = pModel->GetPoints()[(*it)(1)];
        dlovi::Matrix point2 = pModel->GetPoints()[(*it)(2)];

        vector<float> uv0 = kfToTexture->GetTexCoordinate(point0(0),point0(1),point0(2));
        vector<float> uv1 = kfToTexture->GetTexCoordinate(point1(0),point1(1),point1(2));
        vector<float> uv2 = kfToTexture->GetTexCoordinate(point2(0),point2(1),point2(2));

        // if all vertices are projected in the image
        if (uv0.size() == 2 && uv1.size() == 2 && uv2.size() == 2) {

            glTexCoord2f(uv0[0], uv0[1]);
            glVertex3d(point0(0), point0(1), point0(2));

            glTexCoord2f(uv1[0], uv1[1]);
            glVertex3d(point1(0), point1(1), point1(2));

            glTexCoord2f(uv2[0], uv2[1]);
            glVertex3d(point2(0), point2(1), point2(2));

        }
    }

    glEnd();

    glDisable(GL_TEXTURE_2D);


    kfToTexture->SetEraseDrawer();

    pModel->SetErase();

}

void MapDrawer::DrawTriangles(pangolin::OpenGlMatrix &Twc)
    {
        Model* pModel = mpAtlas->GetModel();
        if(pModel == NULL) return;

        pModel->SetNotErase();


        glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

        GLfloat light_position[] = { 0.0, 0.0, 1.0, 0.0 };
        glLightfv(GL_LIGHT0, GL_POSITION, light_position);

        glPopMatrix();

        glEnable(GL_LIGHTING);
        glEnable(GL_LIGHT0);

        glShadeModel(GL_FLAT);

        GLfloat material_diffuse[] = {0.2, 0.5, 0.8, 1};
        glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, material_diffuse);

        glBegin(GL_TRIANGLES);
        glColor3f(1.0,1.0,1.0);

        for (list<dlovi::Matrix>::const_iterator it = pModel->GetTris().begin(); it != pModel->GetTris().end(); it++) {

            dlovi::Matrix point0 = pModel->GetPoints()[(*it)(0)];
            dlovi::Matrix point1 = pModel->GetPoints()[(*it)(1)];
            dlovi::Matrix point2 = pModel->GetPoints()[(*it)(2)];

            dlovi::Matrix edge10 = point1 - point0;
            dlovi::Matrix edge20 = point2 - point0;

            dlovi::Matrix normal = edge20.cross(edge10);
            normal = normal / normal.norm();

            glNormal3d(normal(0), normal(1), normal(2));

            glVertex3d(point0(0), point0(1), point0(2));
            glVertex3d(point1(0), point1(1), point1(2));
            glVertex3d(point2(0), point2(1), point2(2));

        }
        glEnd();

        glDisable(GL_LIGHTING);

        pModel->SetErase();

}

//不太懂 myplan
void MapDrawer::DrawFrame()
{
    const vector<KeyFrame*> &vpKf = mpAtlas->GetAllKeyFrames();
    if(vpKf.empty()) return;

    // get the most recent reconstructed keyframe to texture
    KeyFrame* kfToTexture = NULL;
    KeyFrame* prevKf = NULL;
    for(size_t i = 0; i < vpKf.size();++i) {
        KeyFrame *kf = vpKf[i];
        kf->SetNotEraseDrawer();
        if (kf->isBad() || !kf->semidense_flag_ || !kf->interKF_depth_flag_) {
            kf->SetEraseDrawer();
            continue;
        }
        if (prevKf == NULL){
            kfToTexture = kf;
            prevKf = kf;
        } else if (kf->mnId > prevKf->mnId){
            kfToTexture = kf;
            prevKf->SetEraseDrawer();
            prevKf = kf;
        }
    }
    if (kfToTexture == NULL) return;


    cv::Size imSize = kfToTexture->rgb_.size();

    pangolin::GlTexture imageTexture(imSize.width, imSize.height, GL_RGB, false, 0, GL_BGR,
                                     GL_UNSIGNED_BYTE);

    imageTexture.Upload(kfToTexture->rgb_.data, GL_BGR, GL_UNSIGNED_BYTE);

    imageTexture.RenderToViewportFlipY();


    kfToTexture->SetEraseDrawer();
}

// BRIEF [EAO-SLAM] draw objects.
//myplan 新增画立体物体函数
void MapDrawer::DrawObject(const bool bCubeObj, const bool QuadricObj,
                           const string &flag,
                           const bool bShowBottle,  const bool bShowChair, const bool bShowTvmonitors,
                           const bool bShowKeyboard,const bool bShowMouse, const bool bShowBook,   const bool bShowBear)
{
    const vector<Object_Map*> &vObjs = mpAtlas->GetObjects();

    vector<cv::Mat> object_cen;

    int id = -1;
    for(size_t i = 0; i < vObjs.size(); ++i)
    {
        Object_Map* Obj = vObjs[i];

        if((Obj->mObjectFrame.size() < 5) && (flag != "NA"))
            continue;

        if((Obj->mvpMapObjectMappoints.size() < 10) || (Obj->bBadErase == true))
        {
            continue;
        }

        id ++;

        // Display a certain category of object.
        // cup. mynote 39 瓶子 41 杯子
        if(!bShowBottle && ((Obj->mnClass == 39) || (Obj->mnClass == 41) || (Obj->mnClass == 44) || (Obj->mnClass == 76)))
        {
            continue;
        }
        // chair. mynote 56 椅子
        if(!bShowChair && ((Obj->mnClass == 56) || (Obj->mnClass == 69) || (Obj->mnClass == 72) || (Obj->mnClass == 75)))
        {
            continue;
        }
        // tvmonitor.
        if(!bShowTvmonitors && (Obj->mnClass == 62))
        {
            continue;
        }
        // keyboard.
        if(!bShowKeyboard && (Obj->mnClass == 66))
        {
            continue;
        }
        // mouse.
        if(!bShowMouse && ((Obj->mnClass == 64) || (Obj->mnClass == 65) || (Obj->mnClass == 67)))
        {
            continue;
        }
        // book.mynote 73 书本
        if(!bShowBook && (Obj->mnClass == 73))
        {
            continue;
        }
        // bear.
        if(!bShowBear && (Obj->mnClass == 77))
        {
            continue;
        }

        // color.
        if(i % 10 == 0)
            glColor3f(1.0,0.0,0.0);
        else if(i % 10 == 1)
            glColor3f(0.0,1.0,0.0);
        else if(i % 10 == 2)
            glColor3f(0.0,0.0,1.0);
        else if(i % 10 == 3)
            glColor3f(0.5,0.0,0.0);
        else if(i % 10 == 4)
            glColor3f(0.0,0.5,0.0);
        else if(i % 10 == 5)
            glColor3f(0.0,0.0,0.5);
        else if(i % 10 == 6)
            glColor3f(1.0,1.0,0);
        else if(i % 10 == 7)
            glColor3f(1.0,0,1.0);
        else if(i % 10 == 8)
            glColor3f(0.5,0.5,0.0);
        else if(i % 10 == 9)
            glColor3f(0.5,0,0.5);

        glLineWidth(mCameraLineWidth);

        // *************************************
        //    STEP 1. [EAO-SLAM] Draw cubes.   *
        // *************************************
        if(bCubeObj && ((Obj->mnClass == 73) || (Obj->mnClass == 64) || (Obj->mnClass == 65)
                        || (Obj->mnClass == 66) || (Obj->mnClass == 56) || (Obj->mnClass == 72)))
        {
            bool bObjAsOrigin = true;

            // object center.
            if(bObjAsOrigin)
            {
                cv::Mat Twobj_t = Converter::toCvMat(Obj->mCuboid3D.pose).t();
                cv::Mat Twobj_t_inv = Twobj_t.inv();

                std::cout << "Twobj == " << std::endl << " " << Obj->mCuboid3D.pose << std::endl << std::endl;

                glPushMatrix();
                glMultMatrixf(Twobj_t.ptr<GLfloat>(0));
            }

            // draw object center.
            glPointSize(4*mPointSize);
            glBegin(GL_POINTS);
            if(bObjAsOrigin)
                glVertex3f(0, 0, 0);
            else
                glVertex3f(Obj->mCenter3D.at<float>(0), Obj->mCenter3D.at<float>(1), Obj->mCenter3D.at<float>(2));
            glEnd();

            // ******************************************
            //                 7------6                 *
            //                /|     /|                 *
            //               / |    / |                 *
            //              4------5  |                 *
            //              |  3---|--2                 *
            //              | /    | /                  *
            //              0------1                    *
            // ******************************************

            glBegin(GL_LINES);
            if(bObjAsOrigin)
            {
                float lenth = Obj->mCuboid3D.lenth/2;
                float width = Obj->mCuboid3D.width/2;
                float height = Obj->mCuboid3D.height/2;

                if(Obj->mnClass == 0)
                {
                    glVertex3f(-lenth, -width, 0);      // 5
                    glVertex3f(lenth, -width, 0);       // 6

                    glVertex3f(lenth, -width, 0);       // 6
                    glVertex3f(lenth, width, 0);        // 7

                    glVertex3f(lenth, width, 0);        // 7
                    glVertex3f(-lenth, width, 0);       // 8

                    glVertex3f(-lenth, width, 0);       // 8
                    glVertex3f(-lenth, -width, 0);      // 5


                    glVertex3f(-lenth, -width, -height);    // 1
                    glVertex3f(-lenth, -width, 0);          // 5

                    glVertex3f(lenth, -width, -height);     // 2
                    glVertex3f(lenth, -width, 0);           // 6

                    glVertex3f(lenth, width, height);       // 9
                    glVertex3f(-lenth, width, height);      // 10

                    glVertex3f(lenth, width, -height);      // 3
                    glVertex3f(lenth, width, height);       // 9

                    glVertex3f(-lenth, width, -height);     // 4
                    glVertex3f(-lenth, width, height);      // 10
                }
                else
                {
                    // chair, fixed scale, for better visulazation.
                    if(Obj->mnClass == 56)
                    {
//                        lenth = 0.09;
//                        width = 0.08;
//                        height = 0.12;

//                         lenth = Obj->mCuboid3D.lenth/2;
//                         width = Obj->mCuboid3D.width/2;
//                         height = Obj->mCuboid3D.height/2;

                         lenth = 0.3;//这个是宽
                         width = 0.4;//这个是高
                         height = 0.3;//这个是长

//                        std::cout << "Length===: " << lenth << std::endl;
//                        std::cout << "Width==: " << width << std::endl;
//                        std::cout << "Height===: " << height << std::endl;
                    }

                    glVertex3f(-lenth, -width, -height);    // 1
                    glVertex3f(lenth, -width, -height);     // 2

                    glVertex3f(lenth, -width, -height);     // 2
                    glVertex3f(lenth, width, -height);      // 3

                    glVertex3f(lenth, width, -height);      // 3
                    glVertex3f(-lenth, width, -height);     // 4

                    glVertex3f(-lenth, width, -height);     // 4
                    glVertex3f(-lenth, -width, -height);    // 1

                    glVertex3f(-lenth, -width, height);     // 5
                    glVertex3f(lenth, -width, height);      // 6

                    glVertex3f(lenth, -width, height);      // 6
                    glVertex3f(lenth, width, height);       // 7

                    glVertex3f(lenth, width, height);       // 7
                    glVertex3f(-lenth, width, height);      // 8

                    glVertex3f(-lenth, width, height);      // 8
                    glVertex3f(-lenth, -width, height);     // 5

                    glVertex3f(-lenth, -width, -height);    // 1
                    glVertex3f(-lenth, -width, height);     // 5

                    glVertex3f(lenth, -width, -height);     // 2
                    glVertex3f(lenth, -width, height);      // 6

                    glVertex3f(lenth, width, -height);      // 3
                    glVertex3f(lenth, width, height);       // 7

                    glVertex3f(-lenth, width, -height);     // 4
                    glVertex3f(-lenth, width, height);      // 8
                }
                glEnd();
                glPopMatrix();
            }
        } // draw cubes END ----------------------------------------------------------------------------

        // ****************************************
        //    STEP 2. [EAO-SLAM] Draw quadrics.   *
        // ****************************************
        if(QuadricObj && !((Obj->mnClass == 73) || (Obj->mnClass == 64) || (Obj->mnClass == 65)
                           || (Obj->mnClass == 66) || (Obj->mnClass == 56) || (Obj->mnClass == 72)))
        {
            // half axial length.
            float lenth = Obj->mCuboid3D.lenth/2;
            float width = Obj->mCuboid3D.width/2;
            float height = Obj->mCuboid3D.height/2;

            // tvmonitor, fixed scale, for better visulazation.
            if(Obj->mnClass == 62)
            {
                lenth = 0.3;//长
                width = 0.2;//这个是高
                height = 0.03;//这个是宽

//                lenth = 0.13;
//                width = 0.035;
//                height = 0.08;
            }
            // if(Obj->mnClass == 75)
            // {
            //     lenth = 0.08;
            //     width = 0.08;
            //     height = 0.08;
            // }

            cv::Mat axe = cv::Mat::zeros(3,1,CV_32F);
            axe.at<float>(0) = lenth;
            axe.at<float>(1) = width;
            axe.at<float>(2) = height;

            // quadrcis pose.
            cv::Mat Twq = cv::Mat::zeros(4,4,CV_32F);
            Twq.at<float>(0, 0) = 1;
            Twq.at<float>(0, 1) = 0;
            Twq.at<float>(0, 2) = 0;
            //Twq.at<float>(0, 3) = Obj->mCenter3D.at<float>(0);
            Twq.at<float>(0, 3) = Obj->mCuboid3D.cuboidCenter[0];
            Twq.at<float>(1, 0) = 0;
            Twq.at<float>(1, 1) = 1;
            Twq.at<float>(1, 2) = 0;
            //Twq.at<float>(1, 3) = Obj->mCenter3D.at<float>(1);
            Twq.at<float>(1, 3) = Obj->mCuboid3D.cuboidCenter[1];
            Twq.at<float>(2, 0) = 0;
            Twq.at<float>(2, 1) = 0;
            Twq.at<float>(2, 2) = 1;
            //Twq.at<float>(2, 3) = Obj->mCenter3D.at<float>(2);
            Twq.at<float>(2, 3) = Obj->mCuboid3D.cuboidCenter[2];
            Twq.at<float>(3, 0) = 0;
            Twq.at<float>(3, 1) = 0;
            Twq.at<float>(3, 2) = 0;
            Twq.at<float>(3, 3) = 1;

            // create a quadric.
            GLUquadricObj *pObj = gluNewQuadric();
            cv::Mat Twq_t = Twq.t();

            // color
            cv::Scalar sc;
            sc = cv::Scalar(0, 255, 0);

            // add to display list
            glPushMatrix();
            glMultMatrixf(Twq_t.ptr<GLfloat >(0));
            glScalef(
                    (GLfloat)(axe.at<float>(0,0)),
                    (GLfloat)(axe.at<float>(0,1)),
                    (GLfloat)(axe.at<float>(0,2))
            );

            gluQuadricDrawStyle(pObj, GLU_LINE);
            gluQuadricNormals(pObj, GLU_NONE);
            glBegin(GL_COMPILE);
            gluSphere(pObj, 1., 15, 10);

            glEnd();
            glPopMatrix();
            // draw quadrics END ---------------------------------------------------------------------
        }
    }
} // draw objects END ----------------------------------------------------------------------------



void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph, const bool bDrawInertialGraph)
{
    const float &w = mKeyFrameSize;
    const float h = w*0.75;
    const float z = w*0.6;

    const vector<KeyFrame*> vpKFs = mpAtlas->GetAllKeyFrames();

    if(bDrawKF)
    {

        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFrame* pKF = vpKFs[i];
            cv::Mat Twc = pKF->GetPoseInverse().t();
            unsigned int index_color = pKF->mnOriginMapId;

            glPushMatrix();

            glMultMatrixf(Twc.ptr<GLfloat>(0));

            if(!pKF->GetParent()) // It is the first KF in the map
            {
                glLineWidth(mKeyFrameLineWidth*5);
                if(pKF->mbCreatedByObjs)
                    glColor3f(1.0f,0.0f,0.0f);
                else
                    glColor3f(0.0f,0.0f,1.0f);
                glBegin(GL_LINES);

                //cout << "Initial KF: " << mpAtlas->GetCurrentMap()->GetOriginKF()->mnId << endl;
                //cout << "Parent KF: " << vpKFs[i]->mnId << endl;
            }
            else
            {
                glLineWidth(mKeyFrameLineWidth);
                //myplan 新增函数
                if(pKF->mbCreatedByObjs)
                    glColor3f(1.0f,0.0f,0.0f);
                else
                    glColor3f(0.0f,0.0f,1.0f);
                //glColor3f(0.0f,0.0f,1.0f);
                //glColor3f(mfFrameColors[index_color][0],mfFrameColors[index_color][1],mfFrameColors[index_color][2]);
                glBegin(GL_LINES);
            }

            glVertex3f(0,0,0);
            glVertex3f(w,h,z);
            glVertex3f(0,0,0);
            glVertex3f(w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,h,z);

            glVertex3f(w,h,z);
            glVertex3f(w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(-w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(w,h,z);

            glVertex3f(-w,-h,z);
            glVertex3f(w,-h,z);
            glEnd();

            glPopMatrix();

            glEnd();
        }
    }

    if(bDrawGraph)
    {
        glLineWidth(mGraphLineWidth);
        glColor4f(0.0f,1.0f,0.0f,0.6f);
        glBegin(GL_LINES);

        // cout << "-----------------Draw graph-----------------" << endl;
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            // Covisibility Graph
            const vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
            cv::Mat Ow = vpKFs[i]->GetCameraCenter();
            if(!vCovKFs.empty())
            {
                for(vector<KeyFrame*>::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
                {
                    if((*vit)->mnId<vpKFs[i]->mnId)
                        continue;
                    cv::Mat Ow2 = (*vit)->GetCameraCenter();
                    glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                    glVertex3f(Ow2.at<float>(0),Ow2.at<float>(1),Ow2.at<float>(2));
                }
            }

            // Spanning tree
            KeyFrame* pParent = vpKFs[i]->GetParent();
            if(pParent)
            {
                cv::Mat Owp = pParent->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owp.at<float>(0),Owp.at<float>(1),Owp.at<float>(2));
            }

            // Loops
            set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
            for(set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
            {
                if((*sit)->mnId<vpKFs[i]->mnId)
                    continue;
                cv::Mat Owl = (*sit)->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owl.at<float>(0),Owl.at<float>(1),Owl.at<float>(2));
            }
        }

        glEnd();
    }

    if(bDrawInertialGraph && mpAtlas->isImuInitialized())
    {
        glLineWidth(mGraphLineWidth);
        glColor4f(1.0f,0.0f,0.0f,0.6f);
        glBegin(GL_LINES);

        //Draw inertial links
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFrame* pKFi = vpKFs[i];
            cv::Mat Ow = pKFi->GetCameraCenter();
            KeyFrame* pNext = pKFi->mNextKF;
            if(pNext)
            {
                cv::Mat Owp = pNext->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owp.at<float>(0),Owp.at<float>(1),Owp.at<float>(2));
            }
        }

        glEnd();
    }

    vector<Map*> vpMaps = mpAtlas->GetAllMaps();

    if(bDrawKF)
    {
        for(Map* pMap : vpMaps)
        {
            if(pMap == mpAtlas->GetCurrentMap())
                continue;

            vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();

            for(size_t i=0; i<vpKFs.size(); i++)
            {
                KeyFrame* pKF = vpKFs[i];
                cv::Mat Twc = pKF->GetPoseInverse().t();
                unsigned int index_color = pKF->mnOriginMapId;

                glPushMatrix();

                glMultMatrixf(Twc.ptr<GLfloat>(0));

                if(!vpKFs[i]->GetParent()) // It is the first KF in the map
                {
                    glLineWidth(mKeyFrameLineWidth*5);
                    glColor3f(1.0f,0.0f,0.0f);
                    glBegin(GL_LINES);
                }
                else
                {
                    glLineWidth(mKeyFrameLineWidth);
                    //glColor3f(0.0f,0.0f,1.0f);
                    glColor3f(mfFrameColors[index_color][0],mfFrameColors[index_color][1],mfFrameColors[index_color][2]);
                    glBegin(GL_LINES);
                }

                glVertex3f(0,0,0);
                glVertex3f(w,h,z);
                glVertex3f(0,0,0);
                glVertex3f(w,-h,z);
                glVertex3f(0,0,0);
                glVertex3f(-w,-h,z);
                glVertex3f(0,0,0);
                glVertex3f(-w,h,z);

                glVertex3f(w,h,z);
                glVertex3f(w,-h,z);

                glVertex3f(-w,h,z);
                glVertex3f(-w,-h,z);

                glVertex3f(-w,h,z);
                glVertex3f(w,h,z);

                glVertex3f(-w,-h,z);
                glVertex3f(w,-h,z);
                glEnd();

                glPopMatrix();
            }
        }
    }
}

void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{

    const float &w = mCameraSize;//IMP 修改这里缩小可以看到轨迹
    const float h = w*0.75;
    const float z = w*0.6;

    glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

    glLineWidth(mCameraLineWidth);
    glColor3f(0.0f,1.0f,0.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}


void MapDrawer::SetCurrentCameraPose(const cv::Mat &Tcw)
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.clone();
}

void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M, pangolin::OpenGlMatrix &MOw)
{
    if(!mCameraPose.empty())
    {
        cv::Mat Rwc(3,3,CV_32F);
        cv::Mat twc(3,1,CV_32F);
        {
            unique_lock<mutex> lock(mMutexCamera);
            Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
            twc = -Rwc*mCameraPose.rowRange(0,3).col(3);
        }

        M.m[0] = Rwc.at<float>(0,0);
        M.m[1] = Rwc.at<float>(1,0);
        M.m[2] = Rwc.at<float>(2,0);
        M.m[3]  = 0.0;

        M.m[4] = Rwc.at<float>(0,1);
        M.m[5] = Rwc.at<float>(1,1);
        M.m[6] = Rwc.at<float>(2,1);
        M.m[7]  = 0.0;

        M.m[8] = Rwc.at<float>(0,2);
        M.m[9] = Rwc.at<float>(1,2);
        M.m[10] = Rwc.at<float>(2,2);
        M.m[11]  = 0.0;

        M.m[12] = twc.at<float>(0);
        M.m[13] = twc.at<float>(1);
        M.m[14] = twc.at<float>(2);
        M.m[15]  = 1.0;

        MOw.SetIdentity();
        MOw.m[12] = twc.at<float>(0);
        MOw.m[13] = twc.at<float>(1);
        MOw.m[14] = twc.at<float>(2);
    }
    else
    {
        M.SetIdentity();
        MOw.SetIdentity();
    }
}

void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M, pangolin::OpenGlMatrix &MOw, pangolin::OpenGlMatrix &MTwwp)
{
    if(!mCameraPose.empty())
    {
        cv::Mat Rwc(3,3,CV_32F);
        cv::Mat twc(3,1,CV_32F);
        cv::Mat Rwwp(3,3,CV_32F);
        {
            unique_lock<mutex> lock(mMutexCamera);
            Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
            twc = -Rwc*mCameraPose.rowRange(0,3).col(3);
        }

        M.m[0] = Rwc.at<float>(0,0);
        M.m[1] = Rwc.at<float>(1,0);
        M.m[2] = Rwc.at<float>(2,0);
        M.m[3]  = 0.0;

        M.m[4] = Rwc.at<float>(0,1);
        M.m[5] = Rwc.at<float>(1,1);
        M.m[6] = Rwc.at<float>(2,1);
        M.m[7]  = 0.0;

        M.m[8] = Rwc.at<float>(0,2);
        M.m[9] = Rwc.at<float>(1,2);
        M.m[10] = Rwc.at<float>(2,2);
        M.m[11]  = 0.0;

        M.m[12] = twc.at<float>(0);
        M.m[13] = twc.at<float>(1);
        M.m[14] = twc.at<float>(2);
        M.m[15]  = 1.0;

        MOw.SetIdentity();
        MOw.m[12] = twc.at<float>(0);
        MOw.m[13] = twc.at<float>(1);
        MOw.m[14] = twc.at<float>(2);

        MTwwp.SetIdentity();
        MTwwp.m[0] = Rwwp.at<float>(0,0);
        MTwwp.m[1] = Rwwp.at<float>(1,0);
        MTwwp.m[2] = Rwwp.at<float>(2,0);

        MTwwp.m[4] = Rwwp.at<float>(0,1);
        MTwwp.m[5] = Rwwp.at<float>(1,1);
        MTwwp.m[6] = Rwwp.at<float>(2,1);

        MTwwp.m[8] = Rwwp.at<float>(0,2);
        MTwwp.m[9] = Rwwp.at<float>(1,2);
        MTwwp.m[10] = Rwwp.at<float>(2,2);

        MTwwp.m[12] = twc.at<float>(0);
        MTwwp.m[13] = twc.at<float>(1);
        MTwwp.m[14] = twc.at<float>(2);
    }
    else
    {
        M.SetIdentity();
        MOw.SetIdentity();
        MTwwp.SetIdentity();
    }

}

} //namespace ORB_SLAM
