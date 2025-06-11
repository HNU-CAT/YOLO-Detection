#ifndef INFER_H
#define INFER_H

#include <opencv2/opencv.hpp>
#include "public.h"
#include "types.h"
#include "utils.h"

#include <vision_msgs/Detection2DArray.h>
#include <vision_msgs/BoundingBox2D.h>
#include <sensor_msgs/Image.h>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float32.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <mutex>
#include <queue>
#include <thread>
using namespace nvinfer1;

struct VisualImageDesc
{
    ros::Time stamp;
    cv::Mat raw_image;
    VisualImageDesc(
        ros::Time _stamp,
        cv::Mat _raw_image) : stamp(_stamp), raw_image(_raw_image)
    {}
};
class YoloDetector
{
public:
    YoloDetector(ros::NodeHandle &nh);
    ~YoloDetector();
    void read_config_file(ros::NodeHandle &nh);
    std::vector<Detection> inference(cv::Mat &img);
    void draw_image(cv::Mat &img, std::vector<Detection> &inferResult);
    void inference_image(std::string &image_path);

    std::mutex image_mutex;
    std::queue<VisualImageDesc> image_buf;
    std::thread thread_process_image;
    std::string image_topic_;
    std::string image_path_;
    int use_ros_;

    void registerPub();
    void registerCallback();
    void image_callback(const sensor_msgs::ImageConstPtr img_msg);
    void process_Image_Thread();
    void detection_inference(VisualImageDesc &_current_frame);
    vision_msgs::Detection2DArray convertDetectionsToROS(
        const std::vector<Detection> &detections,
        const std::string &frame_id = "camera");
    cv_bridge::CvImagePtr getImageFromMsg(const sensor_msgs::Image &img_msg);
    cv_bridge::CvImagePtr getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg);

private:
    void get_engine();

    ros::NodeHandle nh_;

    ros::Publisher detected_image_pub;
    ros::Publisher detected_boxes_pub;
    ros::Publisher detected_time_pub;

    ros::Subscriber image_sub;
    cv::Mat received_image;
    float detect_time_;
    std::vector<Detection> res;
    bool img_received = false;
    bool img_detected = false;

    std::string config_file_;

private:
    Logger gLogger;
    std::string trtfile_path_;
    std::string onnxfile_path_;
    int gpuId_;
    int numClass_;
    float nmsThresh_;
    float confThresh_;
    int detect_image_H_, detect_image_W_;
    int detect_max_output_bbox_;
    int detect_num_box_element_;
    int use_fp16_ = 0;
    int use_int8_ = 0;
    std::string cacheFile_;
    std::string calibrationDataPath_;
    std::string output_path_;
    std::vector<std::string> vClassNames_;

    ICudaEngine *engine;
    IRuntime *runtime;
    IExecutionContext *context;

    cudaStream_t stream;

    float *outputData;
    std::vector<void *> vBufferD;
    float *transposeDevice;
    float *decodeDevice;

    int OUTPUT_CANDIDATES; // 8400: 80 * 80 + 40 * 40 + 20 * 20
};

#endif // INFER_H
