# pragma once
#ifndef YOLO_DETECT_H
#define YOLO_DETECT_H
#include <memory>
#include <torch/script.h>
#include <torch/torch.h>
#include <c10/cuda/CUDAStream.h>
#include <ATen/cuda/CUDAEvent.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/dnn/dnn.hpp>
#include "utils.h"
#include <opencv2/opencv.hpp>
#include <torch/script.h>
#include <algorithm>
#include <iostream>
#include <utility>
#include <time.h>
#include <Eigen/Core>  // Eigen核心部分
#include <Eigen/Geometry> // 提供了各种旋转和平移的表示
#define RESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */
#define BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */
#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#define BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
#define BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
#define BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */
#define BOLDWHITE   "\033[1m\033[37m"      /* Bold White */
using namespace std;

class YOLOV5
{
public:
/***
* @brief constructor
* @param model_path - path of the TorchScript weight file
* @param device_type - inference with CPU/GPU
*/
    YOLOV5();
    ~YOLOV5();
    void GetImage(cv::Mat& RGB);
    bool Detect();
    vector<cv::Rect2i> mvPersonArea = {};
    cv::Mat convertTo3Channels(const cv::Mat& binImg);
public:
    cv::Mat mRGB;
    torch::jit::script::Module mModule;
    std::vector<std::string> mClassnames;
    vector<string> mvDynamicNames;    //定义的动态类别
    //定义的静态类别
    vector<string> mvStaicNames;
    ///动态区域
    vector<cv::Rect2i> mvDynamicArea;
    /// view 用
    //map<string, vector<cv::Rect2i>> mmDetectMap;
    vector<Detection> mmDetectMap;
    ///检测不同图像大小
    int image_width = 640;
    int image_height = 480;

    map<string, map<float, vector<cv::Point2f>>> mmboxinfo;
    torch::jit::script::Module module_;
    torch::Device device_;
    bool half_;
    /***
 * @brief inference module
 * @param img - input image
 * @param conf_threshold - confidence threshold
 * @param iou_threshold - IoU threshold for nms
 * @return detection result - bounding box, score, class index
 */
    std::vector<std::vector<Detection>>
    Run(const cv::Mat& img, float conf_threshold, float iou_threshold);

private:
    /***
     * @brief Padded resize
     * @param src - input image
     * @param dst - output image
     * @param out_size - desired output size
     * @return padding information - pad width, pad height and zoom scale
     */
    std::vector<float> LetterboxImage(const cv::Mat& src, cv::Mat& dst, const cv::Size& out_size = cv::Size(640, 640));

    /***
     * @brief Performs Non-Maximum Suppression (NMS) on inference results
     * @note For 640x640 image, 640 / 32(max stride) = 20, sum up boxes from each yolo layer with stride (8, 16, 32) and
     *       3 scales at each layer, we can get total number of boxes - (20x20 + 40x40 + 80x80) x 3 = 25200
     * @param detections - inference results from the network, example [1, 25200, 85], 85 = 4(xywh) + 1(obj conf) + 80(class score)
     * @param conf_thres - object confidence(objectness) threshold
     * @param iou_thres - IoU threshold for NMS algorithm
     * @return detections with shape: nx7 (batch_index, x1, y1, x2, y2, score, classification)
     */
    std::vector<std::vector<Detection>> PostProcessing(const torch::Tensor& detections,
                                                       float pad_w, float pad_h, float scale, const cv::Size& img_shape,
                                                       float conf_thres = 0.4, float iou_thres = 0.6);

    /***
     * @brief Rescale coordinates to original input image
     * @param data - detection result after inference and nms
     * @param pad_w - width padding
     * @param pad_h - height padding
     * @param scale - zoom scale
     * @param img_shape - original input image shape
     */
    void ScaleCoordinates(std::vector<Detection>& data, float pad_w, float pad_h,
                          float scale, const cv::Size& img_shape);

    /***
     * @brief box (center x, center y, width, height) to (x1, y1, x2, y2)
     * @param x - input box with xywh format
     * @return box with xyxy format
     */
    torch::Tensor xywh2xyxy(const torch::Tensor& x);

    /***
     * @brief Convert data from Tensors to vectors
     */
    void Tensor2Detection(const at::TensorAccessor<float, 2>& offset_boxes,
                          const at::TensorAccessor<float, 2>& det,
                          std::vector<cv::Rect>& offset_box_vec,
                          std::vector<float>& score_vec);
    Eigen::Vector3i color_box(int& class_id);

};


#endif //YOLO_DETECT_H