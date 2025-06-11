#include <iostream>
#include <fstream>

#include <NvOnnxParser.h>

#include "infer.h"
#include "preprocess.h"
#include "postprocess.h"
#include "calibrator.h"
#include "utils.h"

using namespace nvinfer1;
YoloDetector::YoloDetector(ros::NodeHandle &nh) : nh_(nh)
{
    read_config_file(nh_);

    gLogger = Logger(ILogger::Severity::kVERBOSE);
    cudaSetDevice(gpuId_);            // 绑定指定GPU设备，确保后续CUDA操作在该设备执行
    CHECK(cudaStreamCreate(&stream)); // 创建CUDA流，用于异步并行执行核函数（如推理与后处理流水线化）
    // load engine
    get_engine();

    context = engine->createExecutionContext();
    context->setBindingDimensions(0, Dims32{4, {1, 3, detect_image_H_, detect_image_W_}}); // 管理推理过程，设置输入维度为[1, 3, kInputH, kInputW]（批次1，3通道，固定输入尺寸如640x640）

    // get engine output info
    Dims32 outDims = context->getBindingDimensions(1); // [1, 84, 8400] 84 = 4（坐标） + 1（置信度） + 80（COCO类别概率）
    OUTPUT_CANDIDATES = outDims.d[2];                  // 8400 8400 不同尺度的锚框总数（如3个检测头，每头网格数分别为80x80、40x40、20x20，总计8400）
    int outputSize = 1;                                // 84 * 8400 用于分配设备内存
    for (int i = 0; i < outDims.nbDims; i++)
    {
        outputSize *= outDims.d[i];
    }
    ROS_INFO("OUTPUT_CANDIDATES: %d", OUTPUT_CANDIDATES);
    ROS_INFO("outputSize: %d", outputSize);
    // prepare output data space on host
    outputData = new float[1 + detect_max_output_bbox_ * detect_num_box_element_];
    // prepare input and output space on device
    vBufferD.resize(2, nullptr);
    // 在gpu上分配内存
    CHECK(cudaMalloc(&vBufferD[0], 3 * detect_image_H_ * detect_image_W_ * sizeof(float))); // 存储预处理后的图像数据（CHW格式，尺寸3*kInputH*kInputW）
    CHECK(cudaMalloc(&vBufferD[1], outputSize * sizeof(float)));                            // 存储模型原始输出（outputSize长度）

    CHECK(cudaMalloc(&transposeDevice, outputSize * sizeof(float)));                                           // 用于转置输出张量（常见于将通道维度后移）
    CHECK(cudaMalloc(&decodeDevice, (1 + detect_max_output_bbox_ * detect_num_box_element_) * sizeof(float))); // 存储解码后的检测框信息（格式可能为[batch_id, x1, y1, x2, y2, class_id, score]）
    ros::Duration(3).sleep();                                                                                  // 阻塞3秒
    if (use_ros_)
    {
        this->registerPub();
        this->registerCallback();
        thread_process_image = std::thread([&]
                                           {
            printf("[thread_process_image] Start thread_procees_image.\n");
            process_Image_Thread();
            printf("[thread_process_image] processVIOKFThread exit.\n"); });
    }
    else
    {
        inference_image(image_path_);
    }
}

void YoloDetector::read_config_file(ros::NodeHandle &nh)
{
    nh.param<std::string>("config_file", config_file_, "");
    if (config_file_.empty())
    {
        ROS_ERROR("please set right config_file path!");
    }
    printf("config_file: %s\n", config_file_.c_str());

    cv::FileStorage fsSettings;
    fsSettings.open(config_file_.c_str(), cv::FileStorage::READ);
    if (!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
        return;
    }

    cv::FileNode fs_yolov11_tensorrt_config = fsSettings["yolov11_tensorrt_config"];
    trtfile_path_ = static_cast<std::string>(fs_yolov11_tensorrt_config["trtFile"]);
    onnxfile_path_ = static_cast<std::string>(fs_yolov11_tensorrt_config["onnxFile"]);
    gpuId_ = static_cast<int>(fs_yolov11_tensorrt_config["kGpuId"]);
    nmsThresh_ = static_cast<float>(fs_yolov11_tensorrt_config["kNmsThresh"]);
    confThresh_ = static_cast<float>(fs_yolov11_tensorrt_config["kConfThresh"]);
    numClass_ = static_cast<int>(fs_yolov11_tensorrt_config["kNumClass"]);
    detect_image_H_ = static_cast<int>(fs_yolov11_tensorrt_config["kInputH"]);
    detect_image_W_ = static_cast<int>(fs_yolov11_tensorrt_config["kInputW"]);
    detect_max_output_bbox_ = static_cast<int>(fs_yolov11_tensorrt_config["kMaxNumOutputBbox"]);
    detect_num_box_element_ = static_cast<int>(fs_yolov11_tensorrt_config["kNumBoxElement"]);
    use_fp16_ = static_cast<int>(fs_yolov11_tensorrt_config["use_FP16_Mode"]);
    use_int8_ = static_cast<int>(fs_yolov11_tensorrt_config["use_INT8_Mode"]);
    if (use_int8_)
    {
        cacheFile_ = static_cast<std::string>(fs_yolov11_tensorrt_config["cacheFile"]);
        calibrationDataPath_ = static_cast<std::string>(fs_yolov11_tensorrt_config["calibrationDataPath"]);
    }
    cv::FileNode vClassNames_names = fs_yolov11_tensorrt_config["vClassNames"];
    for (const auto &name : vClassNames_names)
    {
        vClassNames_.push_back(static_cast<std::string>(name));
        printf("vClassNames: %s\n", static_cast<std::string>(name).c_str());
    }

    output_path_ = (std::string)fsSettings["output_path"];
    image_topic_ = (std::string)fsSettings["image_topic"];
    image_path_ = (std::string)fsSettings["image_path"];
    use_ros_ = (int)fsSettings["use_ros"];
}

void YoloDetector::inference_image(std::string &image_path)
{
    std::vector<std::string> file_names;
    if (read_files_in_dir(image_path, file_names) < 0)
    {
        std::cout << "read_files_in_dir failed." << std::endl;
        ROS_ERROR("please set right config_file path!");
        return;
    }
    // inference
    for (long unsigned int i = 0; i < file_names.size(); i++)
    {
        std::string imagePath = std::string(image_path) + "/" + file_names[i];
        cv::Mat img = cv::imread(imagePath, cv::IMREAD_COLOR);
        if (img.empty())
            continue;

        auto start = std::chrono::system_clock::now();

        std::vector<Detection> res = inference(img);

        auto end = std::chrono::system_clock::now();
        int cost = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        std::cout << "Image: " << file_names[i] << " cost: " << cost << " ms." << std::endl;

        // draw result on image
        draw_image(img, res);
        cv::imwrite(image_path + "/output/" + file_names[i], img);
        std::cout << "Image: " << file_names[i] << " done." << std::endl;
    }
    ROS_ERROR("Successfully detected, please ctrl + c to stop and see in %s!", (image_path + "/output").c_str());
    ros::shutdown();
    return;
}
void YoloDetector::get_engine()
{
    if (access(trtfile_path_.c_str(), F_OK) == 0)
    {
        std::ifstream engineFile(trtfile_path_, std::ios::binary);
        long int fsize = 0;

        engineFile.seekg(0, engineFile.end);
        fsize = engineFile.tellg();
        engineFile.seekg(0, engineFile.beg);
        std::vector<char> engineString(fsize);
        engineFile.read(engineString.data(), fsize);
        if (engineString.size() == 0)
        {
            std::cout << "Failed getting serialized engine!" << std::endl;
            return;
        }
        std::cout << "Succeeded getting serialized engine!" << std::endl;

        runtime = createInferRuntime(gLogger);
        engine = runtime->deserializeCudaEngine(engineString.data(), fsize);
        if (engine == nullptr)
        {
            std::cout << "Failed loading engine!" << std::endl;
            return;
        }
        std::cout << "Succeeded loading engine!" << std::endl;
    }
    else
    {
        IBuilder *builder = createInferBuilder(gLogger);
        INetworkDefinition *network = builder->createNetworkV2(1U << int(NetworkDefinitionCreationFlag::kEXPLICIT_BATCH));
        IOptimizationProfile *profile = builder->createOptimizationProfile();
        IBuilderConfig *config = builder->createBuilderConfig();
        config->setMaxWorkspaceSize(1 << 30);
        IInt8Calibrator *pCalibrator = nullptr;
        if (use_fp16_)
        {
            config->setFlag(BuilderFlag::kFP16);
        }
        if (use_int8_)
        {
            config->setFlag(BuilderFlag::kINT8);
            int batchSize = 8;
            pCalibrator = new Int8EntropyCalibrator2(batchSize, detect_image_W_, detect_image_H_, calibrationDataPath_.c_str(), cacheFile_.c_str());
            config->setInt8Calibrator(pCalibrator);
        }

        nvonnxparser::IParser *parser = nvonnxparser::createParser(*network, gLogger);
        if (!parser->parseFromFile(onnxfile_path_.c_str(), int(gLogger.reportableSeverity)))
        {
            std::cout << std::string("Failed parsing .onnx file!") << std::endl;
            for (int i = 0; i < parser->getNbErrors(); ++i)
            {
                auto *error = parser->getError(i);
                std::cout << std::to_string(int(error->code())) << std::string(":") << std::string(error->desc()) << std::endl;
            }
            return;
        }
        std::cout << std::string("Succeeded parsing .onnx file!") << std::endl;

        ITensor *inputTensor = network->getInput(0);
        profile->setDimensions(inputTensor->getName(), OptProfileSelector::kMIN, Dims32{4, {1, 3, detect_image_H_, detect_image_W_}});
        profile->setDimensions(inputTensor->getName(), OptProfileSelector::kOPT, Dims32{4, {1, 3, detect_image_H_, detect_image_W_}});
        profile->setDimensions(inputTensor->getName(), OptProfileSelector::kMAX, Dims32{4, {1, 3, detect_image_H_, detect_image_W_}});
        config->addOptimizationProfile(profile);

        IHostMemory *engineString = builder->buildSerializedNetwork(*network, *config);
        std::cout << "Succeeded building serialized engine!" << std::endl;

        runtime = createInferRuntime(gLogger);
        engine = runtime->deserializeCudaEngine(engineString->data(), engineString->size());
        if (engine == nullptr)
        {
            std::cout << "Failed building engine!" << std::endl;
            return;
        }
        std::cout << "Succeeded building engine!" << std::endl;

        if (use_int8_ && pCalibrator != nullptr)
        {
            delete pCalibrator;
        }
        std::string trtfile_output_path = output_path_ + "/onnx_model";
        std::ofstream engineFile(trtfile_path_, std::ios::binary);
        engineFile.write(static_cast<char *>(engineString->data()), engineString->size());
        std::cout << "Succeeded saving .plan file!" << std::endl;

        delete engineString;
        delete parser;
        delete config;
        delete network;
        delete builder;
    }
}

YoloDetector::~YoloDetector()
{
    cudaStreamDestroy(stream);

    for (int i = 0; i < 2; ++i)
    {
        CHECK(cudaFree(vBufferD[i]));
    }

    CHECK(cudaFree(transposeDevice));
    CHECK(cudaFree(decodeDevice));

    delete[] outputData;

    delete context;
    delete engine;
    delete runtime;
}

std::vector<Detection> YoloDetector::inference(cv::Mat &img)
{
    if (img.empty()) {
        ROS_ERROR("Input image is empty");
        return {};
    }

    // ROS_INFO("Processing image: %dx%d", img.cols, img.rows);

    // put input on device, then letterbox、bgr to rgb、hwc to chw、normalize.
    preprocess(img, (float *)vBufferD[0], detect_image_H_, detect_image_W_, stream);

    // tensorrt inference
    bool status = context->enqueueV2(vBufferD.data(), stream, nullptr);
    if (!status) {
        ROS_ERROR("TensorRT inference failed");
        return {};
    }

    // transpose [1 84 8400] convert to [1 8400 84]
    transpose((float *)vBufferD[1], transposeDevice, OUTPUT_CANDIDATES, numClass_ + 4, stream);
    // convert [1 8400 84] to [1 7001]
    decode(transposeDevice, decodeDevice, OUTPUT_CANDIDATES, numClass_, confThresh_, detect_max_output_bbox_, detect_num_box_element_, stream);
    // cuda nms
    nms(decodeDevice, nmsThresh_, detect_max_output_bbox_, detect_num_box_element_, stream);

    CHECK(cudaMemcpyAsync(outputData, decodeDevice, (1 + detect_max_output_bbox_ * detect_num_box_element_) * sizeof(float), cudaMemcpyDeviceToHost, stream));
    cudaStreamSynchronize(stream);

    std::vector<Detection> vDetections;
    int count = std::min((int)outputData[0], detect_max_output_bbox_);
    // ROS_INFO("Raw detection count before NMS: %d", count);
    
    for (int i = 0; i < count; i++)
    {
        int pos = 1 + i * detect_num_box_element_;
        int keepFlag = (int)outputData[pos + 6];
        if (keepFlag == 1)
        {
            Detection det;
            memcpy(det.bbox, &outputData[pos], 4 * sizeof(float));
            det.conf = outputData[pos + 4];
            det.classId = (int)outputData[pos + 5];
            // ROS_INFO("Detection %d: class=%d, conf=%.3f, bbox=[%.1f, %.1f, %.1f, %.1f]", 
            //         i, det.classId, det.conf, det.bbox[0], det.bbox[1], det.bbox[2], det.bbox[3]);
            vDetections.push_back(det);
        }
    }

    // ROS_INFO("Final detection count after NMS: %lu", vDetections.size());

    for (size_t j = 0; j < vDetections.size(); j++)
    {
        scale_bbox(img, vDetections[j].bbox, detect_image_W_, detect_image_H_);
    }

    return vDetections;
}

vision_msgs::Detection2DArray YoloDetector::convertDetectionsToROS(
    const std::vector<Detection> &detections,
    const std::string &frame_id)
{
    vision_msgs::Detection2DArray detection_array;

    // 设置消息头
    detection_array.header.stamp = ros::Time::now();
    detection_array.header.frame_id = frame_id;

    // 遍历检测结果
    for (const auto &det : detections)
    {
        vision_msgs::Detection2D detection_msg;

        // 边界框转换
        vision_msgs::BoundingBox2D bbox;
        bbox.center.x = (det.bbox[0] + det.bbox[2]) / 2.0f; // 中心点x
        bbox.center.y = (det.bbox[1] + det.bbox[3]) / 2.0f; // 中心点y
        bbox.size_x = det.bbox[2] - det.bbox[0];            // 宽度
        bbox.size_y = det.bbox[3] - det.bbox[1];            // 高度

        detection_msg.bbox = bbox;

        // 添加到检测数组
        detection_array.detections.push_back(detection_msg);
    }

    return detection_array;
}

void YoloDetector::draw_image(cv::Mat &img, std::vector<Detection> &inferResult)
{
    // draw inference result on image
    for (size_t i = 0; i < inferResult.size(); i++)
    {
        cv::Scalar bboxColor(0, 255, 0);
        cv::Rect r(
            round(inferResult[i].bbox[0]),
            round(inferResult[i].bbox[1]),
            round(inferResult[i].bbox[2] - inferResult[i].bbox[0]),
            round(inferResult[i].bbox[3] - inferResult[i].bbox[1]));
        cv::rectangle(img, r, bboxColor, 2);

        std::string className = vClassNames_[(int)inferResult[i].classId];
        std::string labelStr = className + " " + std::to_string(inferResult[i].conf).substr(0, 4);

        cv::Size textSize = cv::getTextSize(labelStr, 0, 1, 1, NULL);
        cv::Point topLeft(r.x, r.y - textSize.height - 3);
        cv::Point bottomRight(r.x + textSize.width, r.y);
        cv::rectangle(img, topLeft, bottomRight, bboxColor, -1);
        cv::putText(img, labelStr, cv::Point(r.x, r.y - 2), 0, 1, cv::Scalar(0, 0, 255), 2);
        std::string freq_string = "Freq: " + std::to_string(1000 / detect_time_).substr(0, 4) + "Hz";
        cv::putText(img, freq_string, cv::Point(10, 30), 0, 1, cv::Scalar(0, 0, 255), 2);
    }
}

void YoloDetector::registerPub()
{
    this->detected_image_pub = this->nh_.advertise<sensor_msgs::Image>("detected_image", 10);
    this->detected_boxes_pub = this->nh_.advertise<vision_msgs::Detection2DArray>("detected_bounding_boxes", 10);
    this->detected_time_pub = this->nh_.advertise<std_msgs::Float32>("yolo_time", 1);
}

void YoloDetector::registerCallback()
{
    this->image_sub = this->nh_.subscribe(this->image_topic_, 100, &YoloDetector::image_callback, this);
}

void YoloDetector::image_callback(const sensor_msgs::ImageConstPtr img_msg)
{
    // ROS_INFO("Received image: %dx%d, encoding: %s", 
    //          img_msg->width, img_msg->height, img_msg->encoding.c_str());
    auto _orig_image = getImageFromMsg(img_msg);
    if (_orig_image->image.empty()) {
        ROS_ERROR("Failed to convert image message to OpenCV format");
        return;
    }
    VisualImageDesc visual_imagedesc(img_msg->header.stamp, _orig_image->image);
    std::lock_guard<std::mutex> lock(image_mutex);
    image_buf.emplace(visual_imagedesc);
}

void YoloDetector::process_Image_Thread()
{
    while (ros::ok())
    {
        if (!image_buf.empty())
        {
            std::lock_guard<std::mutex> lock(image_mutex);
            if (image_buf.size() > 10)
            {
                ROS_WARN("[process_Image_Thread] Low efficient on sync_process pending frames: %d", image_buf.size());
            }
            VisualImageDesc _current_frame = image_buf.front();
            image_buf.pop();
            this->detection_inference(_current_frame);
        }
    }
}

void YoloDetector::detection_inference(VisualImageDesc &_current_frame)
{
    auto start = std::chrono::steady_clock::now();
    std::vector<Detection> result = inference(_current_frame.raw_image);
    img_detected = true;
    auto end = std::chrono::steady_clock::now();
    auto cost = std::chrono::duration<float, std::milli>(end - start).count();
    detect_time_ = cost;
    ROS_DEBUG("Detection time: %.2f ms", cost);
    
    if (result.empty()) {
        ROS_WARN("No detections found in frame");
    } else {
        ROS_DEBUG("Published %lu detections", result.size());
    }
    
    std_msgs::Float32 cost_msg;
    cost_msg.data = detect_time_;
    detected_time_pub.publish(cost_msg);
    auto ros_detections = this->convertDetectionsToROS(result, "camera_link");
    detected_boxes_pub.publish(ros_detections);
    cv::Mat tmp_image;
    tmp_image = _current_frame.raw_image.clone();
    
    this->draw_image(tmp_image, result);
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", tmp_image).toImageMsg();
    detected_image_pub.publish(msg);
}

cv_bridge::CvImagePtr YoloDetector::getImageFromMsg(const sensor_msgs::Image &img_msg)
{
    cv_bridge::CvImagePtr ptr;
    // std::cout << img_msg->encoding << std::endl;
    if (img_msg.encoding == "8UC1" || img_msg.encoding == "mono8")
    {
        ptr = cv_bridge::toCvCopy(img_msg, "8UC1");
    }
    else if (img_msg.encoding == "16UC1" || img_msg.encoding == "mono16")
    {
        ptr = cv_bridge::toCvCopy(img_msg, "16UC1");
        ptr->image.convertTo(ptr->image, CV_8UC1, 1.0 / 256.0);
    }
    else
    {
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
    }
    return ptr;
}

cv_bridge::CvImagePtr YoloDetector::getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImagePtr ptr;
    // std::cout << img_msg->encoding << std::endl;
    if (img_msg->encoding == "8UC1" || img_msg->encoding == "mono8")
    {
        ptr = cv_bridge::toCvCopy(img_msg, "8UC1");
    }
    else if (img_msg->encoding == "16UC1" || img_msg->encoding == "mono16")
    {
        ptr = cv_bridge::toCvCopy(img_msg, "16UC1");
        ptr->image.convertTo(ptr->image, CV_8UC1, 1.0 / 256.0);
    }
    else
    {
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
    }
    return ptr;
}
