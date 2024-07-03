#include <YOLOV5.h>
#include <fstream>

using namespace cv;

YOLOV5::YOLOV5():device_(torch::kCUDA)//构造函数的声明和成员变量初始化根据里面参数选择设备
{
    try {
        //加载预训练的YOLOv5模型
        //module_ = torch::jit::load("/home/xin/turtlebot3_ws/src/ORB_SLAM3_PRO/weights/best_home.torchscript_gpu.pt");
        module_ = torch::jit::load("/home/lzh/VINS/SPL_SLAM/weights/yolov5s_gpu.pt");
    }
    catch (const c10::Error& e) {
        //如果加载模型出错，会打印错误信息
        std::cerr << "Error loading the model!\n";
        std::exit(EXIT_FAILURE);
    }
    //设备类型确定是否使用半精度浮点数进行计算，在GPU下使用半精度
    half_ = (device_ != torch::kCPU);
    module_.to(device_);//将模型移动到指定的设备上，即将模型加载到CUDA设备或CPU上

    if (half_) {
        module_.to(torch::kHalf);//使用半精度浮点数进行计算，则将模型转换为半精度格式
    }
    module_.eval();//模型设置为评估模式，表示不进行训练，只进行推理

    std::ifstream f("/home/lzh/VINS/SPL_SLAM/weights/coco.names");//打开包含类别标签的文件
    std::string name = "";
    while (std::getline(f, name))
    {
        //逐行读取文件中的类别标签，并将它们存储在名为mClassnames的容器中
        mClassnames.push_back(name);
    }

    mvDynamicNames = {"person", "bicycle", "motorbike", "bus", "car","sports ball","frisbee"};
    mvStaicNames = {"aeroplane","traffic light","fire hydrant","stop sign","parking meter","bench","bird","cat","dog","horse","sheep","cow","elephant","bear","zebra","giraffe","backpack","umbrella","handbag","tie","suitcase","skis","snowboard","kite","baseball bat","baseball glove","skateboard","surfboard","tennis racket","bottle","wine glass","cup","fork","knife","spoon","bowl","banana","apple","sandwich","orange","broccoli","carrot","hot dog","pizza","donut","cake","chair","sofa","pottedplant","bed","diningtable","toilet","tvmonitor","laptop","mouse","remote","keyboard","cell phone","microwave","oven","toaster","sink","refrigerator","book","clock","vase","scissors","teddy bear","hair drier","toothbrush"};

}


YOLOV5::~YOLOV5()
{

}

bool YOLOV5::Detect()
{
    cv::Mat img;
    if(mRGB.empty())
    {
        std::cout << "Read RGB failed!" << std::endl;
        return false;
    }
    // Preparing input tensor
    //将 mRGB 图像调整为指定的尺寸 640 和 480，并将结果存储在 img 中。
    cv::resize(mRGB, img, cv::Size(image_width, image_height));
    int N = img.channels();
    if(N==1)
    {
        //如果是单通道的图像转为3通道的
        img = convertTo3Channels(img);
    }
    //auto根据变量的初始化值自动推导出变量的类型
    //调用YOLOV5类中的Run函数接受图像和两个阈值
    auto result = YOLOV5::Run(img, 0.4, 0.5);//传递图像、置信度阈值和非极大值抑制阈值，返回目标检测结果
    if (result.size() > 0)
    {

        //如果检测结果不为空，则遍历检测结果
        for(auto res = result.begin(); res != result.end(); res++)
        {
            //将检测结果保存在成员变量mmDetectMap
            mmDetectMap = *res;
            //遍历当前检测结果中的每个检测目标
            for(auto det = res->begin(); det != res->end(); det ++)
            {
                //从当前检测结果中获取目标的边界框信息，并将其赋值给变量DetectArea
                cv::Rect2i DetectArea = det->bbox;
                //从当前检测结果中获取目标的分数，并将其赋值给变量 score，类型为 float
                float score = det->score;
                //从当前检测结果中获取目标的类别ID
                int classID = det->class_idx;
                /// 判断80多个类别中等于自己定义的动态区间内的类别
                if (count(mvDynamicNames.begin(), mvDynamicNames.end(), mClassnames[classID]))
                {
                    //目标在动态类中将当前目标的边界框信息赋值给变量DynamicArea
                    cv::Rect2i DynamicArea = det->bbox;
                    //将满足动态区间条件的所有的动态目标的边界框信息添加到mvDynamicArea容器中
                    mvDynamicArea.push_back(DynamicArea);
                }
                // 将目标信息写入txt文件




                //myplan 将检测到的目标信息保存在txt文件中
//                cout <<GREEN<< "#--------------------------------------------#" << endl;
//                cout <<YELLOW<< "# 成功检测到目标 ：        "<< mClassnames[classID]<< endl;
//                cout <<GREEN<< "#--------------------------------------------#" << endl;
//                cout  << endl;

            }

        }

        if (mvDynamicArea.size() == 0)//动态目标容器内没东西
        {
            //无效边界框
            cv::Rect2i tDynamicArea(1, 1, 1, 1);
            mvDynamicArea.push_back(tDynamicArea);
        }
    }

    return true;
}
//Tracking::GrabImage中将图像传递到yolo检测函数
void YOLOV5::GetImage(cv::Mat &RGB)
{
    mRGB = RGB;
}


std::vector<std::vector<Detection>>
//进程主函数。用于目标检测模型并进行后处理
YOLOV5::Run(const cv::Mat& img, float conf_threshold, float iou_threshold)
{
    torch::NoGradGuard no_grad;//禁用梯度计算。因为目标检测过程中不需要进行梯度更新
    // keep the original image for visualization purpose
    cv::Mat img_input = img.clone();//对输入的图像img进行克隆，以便进行可视化处理。
    //调用LetterboxImage函数将输入图像调整为指定尺寸(640x640)，返回填充信息
    std::vector<float> pad_info = LetterboxImage(img_input, img_input, cv::Size(640, 640));
    const float pad_w = pad_info[0];//获取填充后的图像宽度。
    const float pad_h = pad_info[1];//获取填充后的图像高度。
    const float scale = pad_info[2];//获取图像的缩放比例。
    //将图像从BGR颜色空间转换为RGB颜色空间
    cv::cvtColor(img_input, img_input, cv::COLOR_BGR2RGB);  // BGR -> RGB
    //图像数据转换为CV_32FC3格式，并进行归一化
    img_input.convertTo(img_input, CV_32FC3, 1.0f / 255.0f);  // normalization 1/255
    //将图像数据转换为torch::Tensor对象，并将其发送到指定的设备上。
    auto tensor_img = torch::from_blob(img_input.data, {1, img_input.rows, img_input.cols, img_input.channels()}).to(device_);
    //对图像张量进行维度变换，将其从BHWC（批次、通道、高度、宽度）转换为BCHW（批次、通道、高度、宽度）。
    tensor_img = tensor_img.permute({0, 3, 1, 2}).contiguous();  // BHWC -> BCHW (Batch, Channel, Height, Width)

    if (half_)
    {
        //如果使用半精度浮点数计算，则将图像张量转换为半精度数据类型。
        tensor_img = tensor_img.to(torch::kHalf);
    }
    //定义 inputs  用于存储输入模型的参数
    std::vector<torch::jit::IValue> inputs;
    //将图像张量添加到inputs中
    inputs.emplace_back(tensor_img);
    // inference
    //将输入数据传递给模型的前向传播函数，得到输出
    torch::jit::IValue output = module_.forward(inputs);

    ///end = std::chrono::high_resolution_clock::now();
    ///duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    // It should be known that it takes longer time at first time
    ///std::cout << "inference takes : " << duration.count() << " ms" << std::endl;

    /*** Post-process ***/

    ///start = std::chrono::high_resolution_clock::now();
    //从模型输出中提取检测结果，得到一个包含检测框信息的张量
    auto detections = output.toTuple()->elements()[0].toTensor();

    // result: n * 7
    // batch index(0), top-left x/y (1,2), bottom-right x/y (3,4), score(5), class id(6)
   //调用PostProcessing函数对检测结果进行后处理返回目标检测结果
    auto result = PostProcessing(detections, pad_w, pad_h, scale, img.size(), conf_threshold, iou_threshold);
    ///end = std::chrono::high_resolution_clock::now();
    ///duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    // It should be known that it takes longer time at first time
    ///std::cout << "post-process takes : " << duration.count() << " ms" << std::endl;
    return result;
}

//将输入图像调整为指定尺寸并进行填充
std::vector<float> YOLOV5::LetterboxImage(const cv::Mat& src, cv::Mat& dst, const cv::Size& out_size) {
    auto in_h = static_cast<float>(src.rows);//获取输入图像的高度
    auto in_w = static_cast<float>(src.cols);//获取输入图像的宽度
    float out_h = out_size.height;//获取目标尺寸的高度
    float out_w = out_size.width;//获取目标尺寸的宽度

    float scale = std::min(out_w / in_w, out_h / in_h);//算缩放比例，选择宽度和高度的最小比例作为缩放因子

    int mid_h = static_cast<int>(in_h * scale);//根据缩放比例计算中间调整后图像的高度
    int mid_w = static_cast<int>(in_w * scale);//根据缩放比例计算中间调整后图像的宽度。
    //将输入图像按照中间调整后的尺寸进行调整
    cv::resize(src, dst, cv::Size(mid_w, mid_h));
    //计算填充后图像的顶部填充大小。
    int top = (static_cast<int>(out_h) - mid_h) / 2;
    //计算填充后图像的底部填充大小。
    int down = (static_cast<int>(out_h)- mid_h + 1) / 2;
    //计算填充后图像的左侧填充大小。
    int left = (static_cast<int>(out_w)- mid_w) / 2;
    //计算填充后图像的右侧填充大小。
    int right = (static_cast<int>(out_w)- mid_w + 1) / 2;
    //对调整后的图像进行边界填充，使用常数值(114, 114, 114)填充边界
    cv::copyMakeBorder(dst, dst, top, down, left, right, cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
    //创建一个包含填充信息的std::vector，包括左侧填充大小、顶部填充大小和缩放比例。
    std::vector<float> pad_info{static_cast<float>(left), static_cast<float>(top), scale};
    return pad_info;
}

//对模型输出的检测结果进行后处理
std::vector<std::vector<Detection>> YOLOV5::PostProcessing(const torch::Tensor& detections,
                                                           float pad_w, float pad_h, float scale, const cv::Size& img_shape,
                                                           float conf_thres, float iou_thres) {
    constexpr int item_attr_size = 5;//定义每个检测结果属性的大小包括边界框坐标和置信度
    int batch_size = detections.size(0);//获取批次中的图像数量。
    // number of classes, e.g. 80 for coco dataset
    //计算类别的数量，例如COCO数据集有80个类别。
    auto num_classes = detections.size(2) - item_attr_size;

    // get candidates which object confidence > threshold
    //根据置信度阈值，创建一个掩码，表示满足置信度要求的检测结果
    auto conf_mask = detections.select(2, 4).ge(conf_thres).unsqueeze(2);
    //创建一个存储输出检测结果的二维向量
    std::vector<std::vector<Detection>> output;
    //为输出向量预留空间
    output.reserve(batch_size);

    // iterating all images in the batch
    //遍历每个batch中所有每个图像的检测结果
    for (int batch_i = 0; batch_i < batch_size; batch_i++) {
        // apply constrains to get filtered detections for current image
        //：根据掩码过滤出满足置信度要求的检测结果，并将其视图调整为二维张量
        auto det = torch::masked_select(detections[batch_i], conf_mask[batch_i]).view({-1, num_classes + item_attr_size});

        // if none detections remain then skip and start to process next image
        if (0 == det.size(0)) {
            //如果没有检测结果满足置信度要求，则跳过当前图像的处理。
            continue;
        }

        // compute overall score = obj_conf * cls_conf, similar to x[:, 5:] *= x[:, 4:5]
        //计算检测结果的综合得分，即对象置信度乘以类别置信度
        det.slice(1, item_attr_size, item_attr_size + num_classes) *= det.select(1, 4).unsqueeze(1);

        // box (center x, center y, width, height) to (x1, y1, x2, y2)
        //将边界框的(x, y, w, h)格式转换为(x1, y1, x2, y2)格式。
        torch::Tensor box = xywh2xyxy(det.slice(1, 0, 4));

        // [best class only] get the max classes score at each result (e.g. elements 5-84)
        //计算检测结果中类别得分的最大值及其索引。
        std::tuple<torch::Tensor, torch::Tensor> max_classes = torch::max(det.slice(1, item_attr_size, item_attr_size + num_classes), 1);

        // class score
        //获取最大类别得分
        auto max_conf_score = std::get<0>(max_classes);
        // index
        //获取最大类别得分的索引。
        auto max_conf_index = std::get<1>(max_classes);
        //转换最大类别得分的数据类型。
        max_conf_score = max_conf_score.to(torch::kFloat).unsqueeze(1);
        //转换最大类别得分的索引的数据类型。
        max_conf_index = max_conf_index.to(torch::kFloat).unsqueeze(1);

        // shape: n * 6, top-left x/y (0,1), bottom-right x/y (2,3), score(4), class index(5)
        //将边界框坐标、最大类别得分和最大类别索引按列拼接成新的张量det
        det = torch::cat({box.slice(1, 0, 4), max_conf_score, max_conf_index}, 1);

        // for batched NMS
        constexpr int max_wh = 4096;
        //计算缩放后的边界框坐标
        auto c = det.slice(1, item_attr_size, item_attr_size + 1) * max_wh;
        //：将缩放后的边界框坐标与偏移量相加
        auto offset_box = det.slice(1, 0, 4) + c;
        //创建一个cv::Rect类型的向量，用于存储偏移后的边界框
        std::vector<cv::Rect> offset_box_vec;
        //创建一个float类型的向量，用于存储检测结果的得分。
        std::vector<float> score_vec;

        // copy data back to cpu
        //将偏移后的边界框数据拷贝到CPU内存。
        auto offset_boxes_cpu = offset_box.cpu();
        //：将检测结果数据拷贝到CPU内存
        auto det_cpu = det.cpu();
        //使用访问器accessor访问检测结果数据
        const auto& det_cpu_array = det_cpu.accessor<float, 2>();

        // use accessor to access tensor elements efficiently
        //将偏移后的边界框数据和检测结果数据转换为cv::Rect和float类型的向量
        Tensor2Detection(offset_boxes_cpu.accessor<float,2>(), det_cpu_array, offset_box_vec, score_vec);

        // run NMS
        //创建一个整数向量，用于存储非最大抑制后的检测结果索引
        std::vector<int> nms_indices;
        //应用非最大抑制算法，过滤掉重叠度高的检测结果。
        cv::dnn::NMSBoxes(offset_box_vec, score_vec, conf_thres, iou_thres, nms_indices);

        /// vector<Detection> det_vec
        //创建一个Detection类型的向量，用于存储最终的检测结果
        std::vector<Detection> det_vec;
        //遍历非最大抑制后的检测结果索引。
        for (int index : nms_indices)
        {
            Detection t;
            //获取当前索引对应的检测结果数据。
            const auto& b = det_cpu_array[index];
            //根据检测结果数据设置目标的边界框
            t.bbox =
                    cv::Rect(cv::Point(b[Det::tl_x], b[Det::tl_y]),
                             cv::Point(b[Det::br_x], b[Det::br_y]));
            //获取当前检测结果的得分
            t.score = det_cpu_array[index][Det::score];
            //获取当前检测结果的类别索引。
            t.class_idx = det_cpu_array[index][Det::class_idx];
            //根据类别索引获取目标的颜色(R通道)。
            t.color_r =  color_box(t.class_idx).z();
            t.color_g =  color_box(t.class_idx).y();
            t.color_b =  color_box(t.class_idx).x();
            //将当前目标的信息添加到最终的检测结果向量。
            det_vec.emplace_back(t);
        }
        //根据缩放和填充信息，对检测结果的边界框坐标进行还原。
        ScaleCoordinates(det_vec, pad_w, pad_h, scale, img_shape);

        // save final detection for the current image
        //将当前图像的检测结果添加到输出向量中
        output.emplace_back(det_vec);
    } // end of batch iterating

    return output;
}

//对检测结果边界框坐标进行还原和裁切功能
void YOLOV5::ScaleCoordinates(std::vector<Detection>& data,float pad_w, float pad_h,
                              float scale, const cv::Size& img_shape)
{
    //定义了一个lambda函数clip，用于将给定的值限制在指定的范围内。
    auto clip = [](float n, float lower, float upper)
    {
        return std::max(lower, std::min(n, upper));
    };
    //创建一个新的Detection类型的向量，用于存储处理后的边界框信息
    std::vector<Detection> detections;
    //遍历输入的检测结果向量
    for (auto & i : data) {
        //根据填充和缩放信息，将边界框的左上角x坐标还原为原始图像坐标
        float x1 = (i.bbox.tl().x - pad_w)/scale;  // x padding
        //根据填充和缩放信息，将边界框的左上角y坐标还原为原始图像坐标。
        float y1 = (i.bbox.tl().y - pad_h)/scale;  // y padding
        //根据填充和缩放信息，将边界框的右下角x坐标还原为原始图像坐标。
        float x2 = (i.bbox.br().x - pad_w)/scale;  // x padding
        //根据填充和缩放信息，将边界框的右下角y坐标还原为原始图像坐标。
        float y2 = (i.bbox.br().y - pad_h)/scale;  // y padding

        //裁剪边界框的左上角x坐标，确保其在图像范围内
        //该函数的目的是修改 data 中的边界框，使其坐标反映出填充和缩放的效果
        x1 = clip(x1, 0, img_shape.width);
        y1 = clip(y1, 0, img_shape.height);
        x2 = clip(x2, 0, img_shape.width);
        y2 = clip(y2, 0, img_shape.height);

        //根据裁剪后的坐标信息，更新边界框的位置
        i.bbox = cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2));
    }
}

//将边界框中心点高度和宽度转换为左上右下的坐标点
//输出边界框四个点的坐标
torch::Tensor YOLOV5::xywh2xyxy(const torch::Tensor& x) {
    auto y = torch::zeros_like(x);
    // convert bounding box format from (center x, center y, width, height) to (x1, y1, x2, y2)
    //计算左上角的 x 坐标，即中心点 x 坐标减去宽度的一半。
    y.select(1, Det::tl_x) = x.select(1, 0) - x.select(1, 2).div(2);
    y.select(1, Det::tl_y) = x.select(1, 1) - x.select(1, 3).div(2);
    y.select(1, Det::br_x) = x.select(1, 0) + x.select(1, 2).div(2);
    y.select(1, Det::br_y) = x.select(1, 1) + x.select(1, 3).div(2);
    return y;
}

//将张量中的数据转换为检测结果的矩形框和分数
//offset_boxes：一个 at::TensorAccessor<float, 2> 类型的二维张量，表示偏移后的边界框的坐标。
//det：一个 at::TensorAccessor<float, 2> 类型的二维张量，表示检测结果的相关信息。
//offset_box_vec：一个 std::vector<cv::Rect> 类型的向量，用于存储转换后的矩形框。
//score_vec：一个 std::vector<float> 类型的向量，用于存储转换后的分数。
//函数通过遍历 offset_boxes 张量的每一行，在每次迭代中将矩形框和分数添加到相应的向量中。具体操作如下：
void YOLOV5::Tensor2Detection(const at::TensorAccessor<float, 2>& offset_boxes,
                              const at::TensorAccessor<float, 2>& det,
                              std::vector<cv::Rect>& offset_box_vec,
                              std::vector<float>& score_vec) {

    for (int i = 0; i < offset_boxes.size(0) ; i++) {
        offset_box_vec.emplace_back(
                //将左上角点和右下角点传递给 cv::Rect 构造函数，创建一个矩形框，并将其添加到 offset_box_vec 向量中。
                cv::Rect(cv::Point(offset_boxes[i][Det::tl_x], offset_boxes[i][Det::tl_y]),
                         cv::Point(offset_boxes[i][Det::br_x], offset_boxes[i][Det::br_y]))
        );
        score_vec.emplace_back(det[i][Det::score]);
    }
}

//将单通道图像转为3通道
Mat YOLOV5::convertTo3Channels(const Mat& binImg)
{
    Mat three_channel = Mat::zeros(binImg.rows,binImg.cols,CV_8UC3);
    vector<Mat> channels;
    for (int i=0;i<3;i++)
    {
        channels.push_back(binImg);
    }
    merge(channels,three_channel);
    return three_channel;
}

Eigen::Vector3i YOLOV5::color_box(int& class_id)
{
    int x = 0; int y = 0; int z = 0;
    //https://blog.csdn.net/qq_51985653/article/details/113392665?  R G B
    if (mClassnames[class_id] == "person"){x = 241; y =193; z = 217;}  //紫色
    if (mClassnames[class_id] == "chair"){x =115; y =74; z = 18;}     /// 棕土棕色
    if (mClassnames[class_id] == "book"){x = 218; y =112; z = 214;}   /// 淡紫色
    if (mClassnames[class_id] == "car"){x = 1; y =255; z = 255;}      //青色
    if (mClassnames[class_id] == "keyboard"){x = 135; y =38; z = 87;}  /// 草莓色
    if (mClassnames[class_id] == "cup"){x = 255; y =97; z = 0;}    ///橙色
    if (mClassnames[class_id] == "laptop"){x = 163; y =148; z = 128;} //金属色
    if (mClassnames[class_id] == "tvmonitor"){x = 163; y =148; z = 128;}  /// 金属色
    if (mClassnames[class_id] == "knife"){x = 51; y =161; z = 201;}
    if (mClassnames[class_id] == "sofa"){x = 227; y =168; z = 105;}
    if (mClassnames[class_id] == "bed"){x = 188; y =143; z = 143;}
    if (mClassnames[class_id] == "bicycle"){x = 25; y =25; z = 112;}
    if (mClassnames[class_id] == "bear"){x = 139; y =69; z = 19;} /// 马赫棕色
    if (mClassnames[class_id] == "motorbike"){x = 218; y =112; z = 214;}  // 淡紫色
    if (mClassnames[class_id] == "bus"){x = 176; y =224; z = 230;}  //浅灰蓝色
    if (mClassnames[class_id] == "mouse"){x = 255; y = 215; z = 1;}  /// 金黄色
    if (mClassnames[class_id] == "clock"){x = 255; y =127; z = 80;}
    if (mClassnames[class_id] == "refrigerator"){x = 189; y =252; z = 201;}
    if (mClassnames[class_id] == "teddy bear"){x = 139; y =69; z = 19;} /// 紫色
    if (mClassnames[class_id] == "handbag"){x = 128; y =128; z = 105;}
    if (mClassnames[class_id] == "backpack"){x = 255; y =192; z = 203;} //很正的粉色
    if (mClassnames[class_id] == "bottle"){x = 0; y =199; z = 140;}  //土耳其玉色
    if (mClassnames[class_id] == "wine glass"){x = 176; y =23; z = 31;}
    if (mClassnames[class_id] == "truck"){x = 163; y =148; z = 128;}
    if (mClassnames[class_id] == "train"){x = 189; y =252; z = 201;}
    if (mClassnames[class_id] == "vase"){x = 127; y =255; z = 212;}
    if (mClassnames[class_id] == "cell phone"){x = 255; y =250; z = 250;}  ///雪白色
    if (mClassnames[class_id] == "pottedplant"){x = 61; y =145; z = 64;} ///盆栽植物

    // 未给特殊颜色的
    if (mClassnames[class_id] == "aeroplane"){x = 160; y =32; z = 240;}  //紫色
    if (mClassnames[class_id] == "boat"){x = 94; y =38; z = 18;}     // 乌贼墨棕色
    if (mClassnames[class_id] == "traffic light"){x = 218; y =112; z = 214;}   //淡紫色
    if (mClassnames[class_id] == "fire hydrant"){x = 0; y =255; z = 255;}      //青色
    if (mClassnames[class_id] == "stop sign"){x = 128; y =42; z = 42;}  // 棕色
    if (mClassnames[class_id] == "parking meter"){x = 3; y =168; z = 158;}
    if (mClassnames[class_id] == "bench"){x = 220; y =220; z = 220;}  //金属色
    if (mClassnames[class_id] == "bird"){x = 128; y =138; z = 125;}  //冷灰
    if (mClassnames[class_id] == "cat"){x = 139; y =69; z = 19;}
    if (mClassnames[class_id] == "dog"){x = 139; y =69; z = 19;}
    if (mClassnames[class_id] == "horse"){x = 188; y =143; z = 143;}
    if (mClassnames[class_id] == "sheep"){x = 25; y =25; z = 112;}
    if (mClassnames[class_id] == "cow"){x = 115; y =74; z = 18;}
    if (mClassnames[class_id] == "elephant"){x = 218; y =112; z = 214;}  // 淡紫色
    if (mClassnames[class_id] == "zebra"){x = 176; y =224; z = 230;}  //浅灰蓝色
    if (mClassnames[class_id] == "mouse"){x = 0; y =255; z = 127;}
    if (mClassnames[class_id] == "giraffe"){x = 255; y =127; z = 80;}
    if (mClassnames[class_id] == "backpack"){x = 189; y =252; z = 201;}
    if (mClassnames[class_id] == "umbrella"){x = 210; y =7180; z = 240;}
    if (mClassnames[class_id] == "tie"){x = 128; y =128; z = 105;}
    if (mClassnames[class_id] == "suitcase"){x = 255; y =192; z = 203;} //很正的粉色
    if (mClassnames[class_id] == "frisbee"){x = 135; y =38; z = 37;}  //草莓色
    if (mClassnames[class_id] == "skis"){x = 176; y =23; z = 31;}
    if (mClassnames[class_id] == "snowboard"){x = 163; y =148; z = 128;}
    if (mClassnames[class_id] == "sports ball"){x = 189; y =252; z = 201;}
    if (mClassnames[class_id] == "kite"){x = 127; y =255; z = 212;}
    if (mClassnames[class_id] == "baseball bat"){x = 160; y =32; z = 240;}  //紫色
    if (mClassnames[class_id] == "baseball glove"){x = 94; y =38; z = 18;}     // 乌贼墨棕色
    if (mClassnames[class_id] == "skateboard"){x = 218; y =112; z = 214;}   //淡紫色
    if (mClassnames[class_id] == "surfboard"){x = 0; y =255; z = 255;}      //青色
    if (mClassnames[class_id] == "tennis racket"){x = 128; y =42; z = 42;}  // 棕色
    if (mClassnames[class_id] == "fork"){x = 3; y =168; z = 158;}
    if (mClassnames[class_id] == "spoon"){x = 220; y =220; z = 220;}  //金属色
    if (mClassnames[class_id] == "bowl"){x = 128; y =138; z = 125;}  //冷灰
    if (mClassnames[class_id] == "banana"){x = 51; y =161; z = 201;}
    if (mClassnames[class_id] == "apple"){x = 227; y =168; z = 105;}
    if (mClassnames[class_id] == "sandwich"){x = 188; y =143; z = 143;}
    if (mClassnames[class_id] == "orange"){x = 25; y =25; z = 112;}
    if (mClassnames[class_id] == "broccoli"){x = 115; y =74; z = 18;}
    if (mClassnames[class_id] == "carrot"){x = 218; y =112; z = 214;}  // 淡紫色
    if (mClassnames[class_id] == "hot dog"){x = 176; y =224; z = 230;}  //浅灰蓝色
    if (mClassnames[class_id] == "pizza"){x = 0; y =255; z = 127;}
    if (mClassnames[class_id] == "donut"){x = 255; y =127; z = 80;}
    if (mClassnames[class_id] == "cake"){x = 189; y =252; z = 201;}
    if (mClassnames[class_id] == "diningtable"){x = 128; y =128; z = 105;}
    if (mClassnames[class_id] == "toilet"){x = 255; y =192; z = 203;} //很正的粉色
    if (mClassnames[class_id] == "remote"){x = 135; y =38; z = 37;}  //草莓色
    if (mClassnames[class_id] == "microwave"){x = 163; y =148; z = 128;}
    if (mClassnames[class_id] == "oven"){x = 189; y =252; z = 201;}
    if (mClassnames[class_id] == "toaster"){x = 127; y =255; z = 212;}
    if (mClassnames[class_id] == "sink"){x = 25; y =25; z = 112;}
    if (mClassnames[class_id] == "scissors"){x = 115; y =74; z = 18;}
    if (mClassnames[class_id] == "hair drier"){x = 218; y =112; z = 214;}  // 淡紫色
    if (mClassnames[class_id] == "toothbrush"){x = 176; y =224; z = 230;}  //浅灰蓝

    Eigen::Vector3i color_rgb(x,y,z);
    return color_rgb;
}