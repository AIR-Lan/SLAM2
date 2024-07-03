

#pragma once

//STL
#include <utility>
#include <vector>

//OpenCV
#include <opencv2/core.hpp>

#include "gridStructure.h"
#include "Frame.h"
#include "MapPoint.h"
#include "MapLine.h"

namespace ORB_SLAM3 {

class Frame;
class MapPoint;
class MapLine;

typedef std::pair<int, int> point_2d;
typedef std::pair<point_2d, point_2d> line_2d;

inline double dot(const std::pair<double, double> &a, const std::pair<double, double> &b) {
    return (a.first*b.first + a.second*b.second);
}

inline void normalize(std::pair<double, double> &v) {
    double magnitude = std::sqrt(dot(v, v));

    v.first /= magnitude;
    v.second /= magnitude;
}

class LineMatcher
{
public:
    int static matchNNR(const cv::Mat &desc1, const cv::Mat &desc2, float nnr, std::vector<int> &matches_12);

    int static match(const std::vector<MapLine*> &mvpLocalMapLines, Frame &CurrentFrame, float nnr, std::vector<int> &matches_12);

    int static match(const cv::Mat &desc1, const cv::Mat &desc2, float nnr, std::vector<int> &matches_12);

    int static distance(const cv::Mat &a, const cv::Mat &b);

    int static matchGrid(const std::vector<line_2d> &lines1, const cv::Mat &desc1, const GridStructure &grid, const cv::Mat &desc2, const std::vector<std::pair<double, double>> &directions2, const GridWindow &w, std::vector<int> &matches_12);

    int static SearchByProjection(Frame &CurrentFrame, Frame &LastFrame, const GridStructure &grid, const float &th, const float &angth);
};

} // namesapce ORB_SLAM3
