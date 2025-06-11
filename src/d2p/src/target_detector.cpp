#include "d2p/target_detector.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace d2p {

TargetDetector::TargetDetector(ros::NodeHandle& nh) 
    : nh_(nh),
      depth_scale_(1000.0),
      depth_filter_margin_(5),
      img_rows_(480),
      img_cols_(640),
      depth_min_value_(0.2),
      depth_max_value_(10.0),
      odom_received_(false),
      next_target_id_(0),
      target_size_x_(0.2),
      target_size_y_(0.2),
      target_size_z_(0.1),
      min_target_size_(0.1),
      max_target_size_(0.5),
      max_tracking_distance_(4.0),
      max_history_size_(50),
      min_tracking_confidence_(0.6),
      max_tracking_velocity_(5.0),
      max_tracking_acceleration_(2.0) {
    
    try {
        ROS_INFO("Starting TargetDetector initialization...");
        
        // 创建私有节点句柄，使用正确的命名空间
        ros::NodeHandle private_nh("~");
        ROS_INFO("Node namespace: %s", private_nh.getNamespace().c_str());
        
        // 初始化参数
        ROS_INFO("Initializing parameters...");
        initializeParameters(private_nh);
        
        // 加载变换矩阵
        ROS_INFO("Loading transforms...");
        loadTransforms(private_nh);
        
        // 设置订阅者
        ROS_INFO("Setting up subscribers...");
        setupSubscribers();
        
        // 设置发布者
        ROS_INFO("Setting up publishers...");
        setupPublishers();
        
        ROS_INFO("TargetDetector initialization completed successfully");
    } catch (const std::exception& e) {
        ROS_ERROR("Exception during TargetDetector initialization: %s", e.what());
        throw;  // 重新抛出异常，让上层处理
    } catch (...) {
        ROS_ERROR("Unknown exception during TargetDetector initialization");
        throw;
    }
}

void TargetDetector::initializeParameters(const ros::NodeHandle& nh) {
    try {
        ROS_INFO("Loading topic names...");
        // Load topic names
        nh.param("topics/depth_image", depth_image_topic_, std::string("/camera/depth/image_rect_raw"));
        nh.param("topics/detections", detections_topic_, std::string("/yolo_detector/detections"));
        nh.param("topics/odometry", odometry_topic_, std::string("/fake_odom"));
        nh.param("topics/target_position", target_position_topic_, std::string("/target_3d_position"));
        nh.param("topics/bbox_visualization", bbox_visualization_topic_, std::string("/yolo_3d_bboxes"));
        nh.param("topics/depth_visualization", depth_visualization_topic_, std::string("/filtered_depth_image"));
        nh.param("topics/trajectory_visualization", trajectory_visualization_topic_, std::string("/target_trajectories"));

        // 打印加载的话题名称用于调试
        ROS_INFO("Loaded topic names:");
        ROS_INFO("  Depth image: %s", depth_image_topic_.c_str());
        ROS_INFO("  Detections: %s", detections_topic_.c_str());
        ROS_INFO("  Odometry: %s", odometry_topic_.c_str());
        ROS_INFO("  Target position: %s", target_position_topic_.c_str());
        ROS_INFO("  Bbox visualization: %s", bbox_visualization_topic_.c_str());
        ROS_INFO("  Depth visualization: %s", depth_visualization_topic_.c_str());
        ROS_INFO("  Trajectory visualization: %s", trajectory_visualization_topic_.c_str());

        // Load queue sizes
        ROS_INFO("Loading queue sizes...");
        nh.param("queue_sizes/depth_image", depth_image_queue_size_, 10);
        nh.param("queue_sizes/detections", detections_queue_size_, 100);
        nh.param("queue_sizes/odometry", odometry_queue_size_, 30);
        nh.param("queue_sizes/visualization", visualization_queue_size_, 10);

        // Load target parameters
        ROS_INFO("Loading target parameters...");
        nh.param("target_size_x", target_size_x_, 0.2);
        nh.param("target_size_y", target_size_y_, 0.2);
        nh.param("target_size_z", target_size_z_, 0.1);
        nh.param("min_target_size", min_target_size_, 0.1);
        nh.param("max_target_size", max_target_size_, 0.5);

        // Load tracking parameters
        ROS_INFO("Loading tracking parameters...");
        nh.param("max_tracking_distance", max_tracking_distance_, 4.0);
        nh.param("max_history_size", max_history_size_, 50);
        nh.param("min_tracking_confidence", min_tracking_confidence_, 0.6);
        nh.param("max_tracking_velocity", max_tracking_velocity_, 5.0);
        nh.param("max_tracking_acceleration", max_tracking_acceleration_, 2.0);

        // Load other parameters
        ROS_INFO("Loading other parameters...");
        double depth_scale;
        if (!nh.param("depth_scale", depth_scale, 1000.0)) {
            ROS_WARN("Using default depth_scale: 1000.0");
            depth_scale_ = 1000.0;
        } else {
            depth_scale_ = depth_scale;
        }

        int depth_filter_margin;
        if (!nh.param("depth_filter_margin", depth_filter_margin, 5)) {
            ROS_WARN("Using default depth_filter_margin: 5");
            depth_filter_margin_ = 5;
        } else {
            depth_filter_margin_ = depth_filter_margin;
        }

        double depth_min_value;
        if (!nh.param("depth_min_value", depth_min_value, 0.2)) {
            ROS_WARN("Using default depth_min_value: 0.2");
            depth_min_value_ = 0.2;
        } else {
            depth_min_value_ = depth_min_value;
        }

        double depth_max_value;
        if (!nh.param("depth_max_value", depth_max_value, 10.0)) {
            ROS_WARN("Using default depth_max_value: 10.0");
            depth_max_value_ = 10.0;
        } else {
            depth_max_value_ = depth_max_value;
        }

        // Camera intrinsics
        ROS_INFO("Loading camera intrinsics...");
        std::vector<double> default_intrinsics = {604.404296875, 604.404296875, 325.03704833984375, 245.77059936523438};
        if (!nh.param("color_intrinsics", color_intrinsics_, default_intrinsics)) {
            ROS_WARN("Using default color intrinsics");
            color_intrinsics_ = default_intrinsics;
        }
        
        if (color_intrinsics_.size() != 4) {
            ROS_ERROR("Invalid color intrinsics size: %zu, expected 4", color_intrinsics_.size());
            throw std::runtime_error("Invalid color intrinsics size");
        }
        
        fx_ = color_intrinsics_[0];
        fy_ = color_intrinsics_[1];
        cx_ = color_intrinsics_[2];
        cy_ = color_intrinsics_[3];
        
        ROS_INFO("Camera intrinsics: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f", fx_, fy_, cx_, cy_);
        
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in initializeParameters: %s", e.what());
        throw;
    } catch (...) {
        ROS_ERROR("Unknown exception in initializeParameters");
        throw;
    }
}

void TargetDetector::loadTransforms(const ros::NodeHandle& nh) {
    std::vector<double> body_to_cam_vec, body_to_cam_color_vec;
    
    if (nh.getParam("transforms/body_to_cam", body_to_cam_vec) && 
        body_to_cam_vec.size() == 16) {
        body_to_cam_ = Eigen::Map<Eigen::Matrix4d>(body_to_cam_vec.data());
        ROS_INFO("Successfully loaded body_to_cam transform");
    } else {
        ROS_ERROR("Failed to load body_to_cam transform from namespace %s", nh.getNamespace().c_str());
    }
    
    if (nh.getParam("transforms/body_to_cam_color", body_to_cam_color_vec) && 
        body_to_cam_color_vec.size() == 16) {
        body_to_cam_color_ = Eigen::Map<Eigen::Matrix4d>(body_to_cam_color_vec.data());
        ROS_INFO("Successfully loaded body_to_cam_color transform");
    } else {
        ROS_ERROR("Failed to load body_to_cam_color transform from namespace %s", nh.getNamespace().c_str());
    }
}

void TargetDetector::setupSubscribers() {
    try {
        ROS_INFO("Setting up depth image subscriber...");
        depth_sub_.subscribe(nh_, depth_image_topic_, depth_image_queue_size_);
        
        ROS_INFO("Setting up detection subscriber...");
        detection_sub_.subscribe(nh_, detections_topic_, detections_queue_size_);
        
        ROS_INFO("Setting up synchronizer...");
        sync_.reset(new Sync(SyncPolicy(detections_queue_size_), depth_sub_, detection_sub_));
        sync_->registerCallback(boost::bind(&TargetDetector::syncCallback, this, _1, _2));
        
        ROS_INFO("Setting up odometry subscriber...");
        odom_sub_ = nh_.subscribe(odometry_topic_, odometry_queue_size_, &TargetDetector::odomCallback, this);
        
        ROS_INFO("All subscribers setup completed");
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in setupSubscribers: %s", e.what());
        throw;
    } catch (...) {
        ROS_ERROR("Unknown exception in setupSubscribers");
        throw;
    }
}

void TargetDetector::setupPublishers() {
    try {
        ROS_INFO("Setting up publishers...");
        yolo_bboxes_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(bbox_visualization_topic_, visualization_queue_size_);
        depth_viz_pub_ = nh_.advertise<sensor_msgs::Image>(depth_visualization_topic_, visualization_queue_size_);
        trajectory_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(trajectory_visualization_topic_, visualization_queue_size_);
        ROS_INFO("All publishers setup completed");
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in setupPublishers: %s", e.what());
        throw;
    } catch (...) {
        ROS_ERROR("Unknown exception in setupPublishers");
        throw;
    }
}

void TargetDetector::processDepthImage(const cv::Mat& input, cv::Mat& output) {
    if(input.empty()) {
        ROS_WARN("Empty depth image received!");
        return;
    }
    
    // 使用静态变量存储上一次的时间戳，用于控制处理频率
    static ros::Time last_process_time = ros::Time::now();
    ros::Time current_time = ros::Time::now();
    
    // 控制处理频率，最多20Hz
    if ((current_time - last_process_time).toSec() < 0.05) {  // 50ms = 20Hz
        output = aligned_depth_image_;  // 直接使用上一帧的结果
        return;
    }
    last_process_time = current_time;
    
    // 转换为float类型进行处理
    cv::Mat float_depth;
    input.convertTo(float_depth, CV_32F, 1.0/depth_scale_);
    
    // 使用更快的滤波方法
    cv::Mat filtered;
    // 使用高斯滤波替代双边滤波，提高速度
    cv::GaussianBlur(float_depth, filtered, cv::Size(5, 5), 1.5);
    
    // 使用更快的空洞填充方法
    cv::Mat filled = filtered.clone();
    cv::Mat mask = (filled == 0);
    if (cv::countNonZero(mask) > 0) {
        cv::Mat dilated;
        cv::dilate(filled, dilated, cv::Mat(), cv::Point(-1,-1), 2);
        dilated.copyTo(filled, mask);
    }
    
    // 转换回uint16
    filled.convertTo(output, CV_16U, depth_scale_);
    
    // 发布可视化（使用节流控制）
    static ros::Time last_viz_time = ros::Time::now();
    if ((current_time - last_viz_time).toSec() >= 0.1) {  // 10Hz可视化
        publishDepthVisualization(filled);
        last_viz_time = current_time;
    }
}

void TargetDetector::publishDepthVisualization(const cv::Mat& depth_image) {
    // 创建彩色深度图
    cv::Mat depth_colored;
    cv::Mat depth_normalized;
    
    // 获取深度范围（使用更高效的方法）
    double min_depth = std::numeric_limits<double>::max();
    double max_depth = std::numeric_limits<double>::min();
    
    // 使用OpenMP加速深度范围计算
    #pragma omp parallel
    {
        double local_min = std::numeric_limits<double>::max();
        double local_max = std::numeric_limits<double>::min();
        
        #pragma omp for nowait
        for(int i = 0; i < depth_image.rows; i++) {
            for(int j = 0; j < depth_image.cols; j++) {
                float val = depth_image.at<float>(i,j);
                if(val > 0) {
                    local_min = std::min(local_min, static_cast<double>(val));
                    local_max = std::max(local_max, static_cast<double>(val));
                }
            }
        }
        
        #pragma omp critical
        {
            min_depth = std::min(min_depth, local_min);
            max_depth = std::max(max_depth, local_max);
        }
    }
    
    // 如果深度范围太小，使用默认范围
    if (max_depth - min_depth < 0.1) {
        min_depth = depth_min_value_;
        max_depth = depth_max_value_;
    }
    
    // 归一化到0-255
    depth_image.convertTo(depth_normalized, CV_8UC1, 255.0/(max_depth - min_depth), -min_depth * 255.0/(max_depth - min_depth));
    
    // 应用颜色映射
    cv::applyColorMap(depth_normalized, depth_colored, cv::COLORMAP_JET);
    
    // 简化可视化，移除刻度条以提高性能
    // 转换为ROS消息并发布
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", depth_colored).toImageMsg();
    depth_viz_pub_.publish(msg);
    
    // 使用ROS_DEBUG_THROTTLE减少日志输出频率
    // ROS_DEBUG_THROTTLE(1.0, "Depth range: %.2f - %.2f meters", min_depth, max_depth);
}

void TargetDetector::syncCallback(const sensor_msgs::ImageConstPtr& depth_msg,
                                const vision_msgs::Detection2DArrayConstPtr& detections) {
    if(!odom_received_) {
        ROS_WARN_THROTTLE(1.0, "Waiting for odometry data...");
        return;
    }

    // 检查深度图像消息
    if (!depth_msg) {
        ROS_ERROR_THROTTLE(1.0,"Received null depth image message!");
        return;
    }

    // 检查检测消息
    if (!detections) {
        ROS_ERROR_THROTTLE(1.0,"Received null detections message!");
        return;
    }

    try {
        // 处理深度图像
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(depth_msg, depth_msg->encoding);
            if (!cv_ptr || cv_ptr->image.empty()) {
                ROS_ERROR("Failed to convert depth image or empty image");
                return;
            }
            // ROS_DEBUG("Successfully converted depth image, size: %dx%d", 
            //          cv_ptr->image.cols, cv_ptr->image.rows);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        if(depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
            cv_ptr->image.convertTo(cv_ptr->image, CV_16UC1, depth_scale_);
        }

        std::lock_guard<std::mutex> depth_lock(depth_mutex_);
        processDepthImage(cv_ptr->image, aligned_depth_image_);

        // 处理检测结果
        if(detections->detections.empty()) {
            ROS_INFO_THROTTLE(1.0, "No detections received");
            return;
        }

        // 统计有效检测结果
        size_t valid_detections = 0;
        for(const auto& detection : detections->detections) {
            // 检查bbox是否有效
            if(detection.bbox.size_x > 0 && detection.bbox.size_y > 0) {
                // 检查坐标是否在图像范围内
                if(detection.bbox.center.x >= 0 && detection.bbox.center.x < img_cols_ &&
                   detection.bbox.center.y >= 0 && detection.bbox.center.y < img_rows_) {
                    valid_detections++;
                } else {
                    ROS_WARN("Detection bbox center (%.2f, %.2f) out of image bounds (%dx%d)",
                            detection.bbox.center.x, detection.bbox.center.y, img_cols_, img_rows_);
                }
            }
        }

        // ROS_INFO("Received %zu detections, %zu with valid bboxes", 
        //          detections->detections.size(), valid_detections);

        if(valid_detections == 0) {
            ROS_WARN_THROTTLE(1.0, "All detections have invalid bboxes!");
            return;
        }

        std::vector<Target3D> current_targets;
        for(const auto& detection : detections->detections) {
            // 检查bbox是否有效
            if(detection.bbox.size_x <= 0 || detection.bbox.size_y <= 0) {
                ROS_DEBUG_THROTTLE(1.0, "Skipping detection with invalid bbox");
                continue;
            }

            // 检查坐标是否在图像范围内
            if(detection.bbox.center.x < 0 || detection.bbox.center.x >= img_cols_ ||
               detection.bbox.center.y < 0 || detection.bbox.center.y >= img_rows_) {
                ROS_WARN("Skipping detection with bbox center (%.2f, %.2f) out of image bounds (%dx%d)",
                        detection.bbox.center.x, detection.bbox.center.y, img_cols_, img_rows_);
                continue;
            }

            try {
                Target3D target;
                estimateTarget3D(detection, aligned_depth_image_, target);
                if(target.is_tracked) {
                    current_targets.push_back(target);
                    tracked_targets_[target.id] = target;
                }
            } catch (const std::exception& e) {
                ROS_ERROR("Exception in estimateTarget3D: %s", e.what());
                continue;
            } catch (...) {
                ROS_ERROR("Unknown exception in estimateTarget3D");
                continue;
            }
        }

        // 更新跟踪
        // try {
        //     updateTargetTracking();
        // } catch (const std::exception& e) {
        //     ROS_ERROR("Exception in updateTargetTracking: %s", e.what());
        // } catch (...) {
        //     ROS_ERROR("Unknown exception in updateTargetTracking");
        // }

        // 发布可视化
        if (!current_targets.empty()) {
            try {
                publishVisualization();
                // ROS_INFO("Published visualization for %zu targets", current_targets.size());
            } catch (const std::exception& e) {
                ROS_ERROR("Exception in publishVisualization: %s", e.what());
            } catch (...) {
                ROS_ERROR("Unknown exception in publishVisualization");
            }
        }

    } catch (const std::exception& e) {
        ROS_ERROR("Exception in syncCallback: %s", e.what());
    } catch (...) {
        ROS_ERROR("Unknown exception in syncCallback");
    }
}

void TargetDetector::estimateTarget3D(const vision_msgs::Detection2D& detection,
                                    const cv::Mat& depth_image,
                                    Target3D& target) {
    // 首先检查深度图像是否有效
    if (depth_image.empty()) {
        ROS_ERROR("Empty depth image!");
        target.is_tracked = false;
        return;
    }

    // 检查深度图像尺寸
    if (depth_image.rows != img_rows_ || depth_image.cols != img_cols_) {
        ROS_ERROR("Depth image size mismatch! Expected %dx%d, got %dx%d", 
                 img_rows_, img_cols_, depth_image.rows, depth_image.cols);
        target.is_tracked = false;
        return;
    }

    // 检查检测框是否有效
    if (detection.bbox.size_x <= 0 || detection.bbox.size_y <= 0) {
        ROS_DEBUG_THROTTLE(1.0, "Detection has invalid bbox, skipping 3D estimation");
        target.is_tracked = false;
        return;
    }

    // 获取检测框，添加安全检查
    double center_x = detection.bbox.center.x;
    double center_y = detection.bbox.center.y;
    double size_x = detection.bbox.size_x;
    double size_y = detection.bbox.size_y;

    // 确保坐标在有效范围内
    if (center_x < 0 || center_x >= img_cols_ || center_y < 0 || center_y >= img_rows_) {
        ROS_ERROR("Detection center (%.2f, %.2f) out of image bounds (%dx%d)",
                 center_x, center_y, img_cols_, img_rows_);
        target.is_tracked = false;
        return;
    }

    // 打印检测框信息
    // ROS_DEBUG("Processing detection box: center=(%.2f, %.2f), size=(%.2f, %.2f)", 
    //           center_x, center_y, size_x, size_y);

    // 计算边界框的整数坐标，确保在图像范围内
    int top_x = static_cast<int>(std::max(0.0, std::round(center_x - size_x/2.0)));
    int top_y = static_cast<int>(std::max(0.0, std::round(center_y - size_y/2.0)));
    int width = static_cast<int>(std::min(static_cast<double>(img_cols_ - top_x), std::round(size_x)));
    int height = static_cast<int>(std::min(static_cast<double>(img_rows_ - top_y), std::round(size_y)));

    // 验证计算后的边界框
    if (width <= 0 || height <= 0 || 
        top_x + width > img_cols_ || top_y + height > img_rows_) {
        ROS_ERROR("Invalid calculated bbox: (%d, %d, %d, %d) for image size %dx%d",
                 top_x, top_y, width, height, img_cols_, img_rows_);
        target.is_tracked = false;
        return;
    }

    // ROS_DEBUG("Calculated bbox: (%d, %d, %d, %d)", top_x, top_y, width, height);

    // 收集深度值，添加边界检查
    std::vector<double> depth_values;
    std::vector<double> weights;
    const double inv_factor = 1.0 / depth_scale_;
    
    try {
        // 添加深度值统计
        double min_depth = std::numeric_limits<double>::max();
        double max_depth = std::numeric_limits<double>::min();
        double sum_depth = 0.0;
        int valid_count = 0;
        
        for(int v = top_y; v < top_y + height; ++v) {
            for(int u = top_x; u < top_x + width; ++u) {
                if (v >= 0 && v < depth_image.rows && u >= 0 && u < depth_image.cols) {
                    uint16_t depth_raw = depth_image.at<uint16_t>(v, u);
                    double depth = depth_raw * inv_factor;
                    
                    if(depth >= depth_min_value_ && depth <= depth_max_value_) {
                        // 更新统计信息
                        min_depth = std::min(min_depth, depth);
                        max_depth = std::max(max_depth, depth);
                        sum_depth += depth;
                        valid_count++;
                        
                        // 计算高斯权重
                        double dx = (u - (top_x + width/2.0)) / (width/2.0);
                        double dy = (v - (top_y + height/2.0)) / (height/2.0);
                        double weight = std::exp(-(dx*dx + dy*dy));
                        
                        depth_values.push_back(depth);
                        weights.push_back(weight);
                    }
                }
            }
        }


        if(depth_values.empty()) {
            ROS_WARN("No valid depth values in detection box!");
            target.is_tracked = false;
            return;
        }

        // 计算加权中值深度
        double depth_median, mad;
        calculateMAD(depth_values, weights, depth_median, mad);
        

        // ----------------中心点估计-----------------
        // 投影到3D空间（相机坐标系）
        Eigen::Vector3d center_cam;
        center_cam(0) = (center_x - cx_) * depth_median / fx_;  // 相机系：x向右
        center_cam(1) = (center_y - cy_) * depth_median / fy_;  // 相机系：y向下
        center_cam(2) = depth_median;                           // 相机系：z向前

    
        // 转换到机体系（调整坐标轴对应关系）
        // 相机系 -> 机体系：x(右) -> -y(左), y(下) -> -z(上), z(前) -> x(前)
        Eigen::Vector3d center_body;
        center_body(0) = center_cam(2);   // 相机z轴 -> 机体x轴（前）
        center_body(1) = -center_cam(0);  // 相机x轴 -> 机体y轴（左）
        center_body(2) = -center_cam(1);  // 相机y轴 -> 机体z轴（上）

        ROS_INFO("center_body: (%.2f, %.2f, %.2f)", center_body(0), center_body(1), center_body(2));
        //----------------尺寸估计-----------------
        // 估计目标尺寸 - 使用检测框的像素尺寸动态计算
        double confidence = 0.8;  // 使用默认置信度
        // 从像素尺寸和深度反投影，计算x,y方向上的尺寸（米）
        double x_width = (size_x / fx_) * center_cam(2) * confidence;
        double y_width = (size_y / fy_) * center_cam(2) * confidence;
        // z方向的尺寸使用ROI内深度的极差来估算
        double z_width = (max_depth - min_depth) * confidence;

        // 对尺寸进行约束，不超过参数中设置的最大值
        x_width = std::min(x_width, target_size_x_);
        z_width = std::min(z_width, target_size_z_);
        y_width = std::min(y_width, target_size_y_);

        // 转换尺寸到机体系（调整坐标轴对应关系）
        Eigen::Vector3d size_body;
        size_body(0) = z_width;   // 相机z轴 -> 机体x轴（前）
        size_body(1) = x_width;   // 相机x轴 -> 机体y轴（左）
        size_body(2) = y_width;   // 相机y轴 -> 机体z轴（上）

        // 转换到世界坐标系
        Eigen::Vector3d world_center, world_size;
        transformBBox(center_body, size_body,
                     position_, orientation_,  // 使用机体的位置和方向
                     world_center, world_size);

        ROS_INFO("world_center: (%.2f, %.2f, %.2f)", world_center(0), world_center(1), world_center(2));
        // 初始化目标
        target.id = next_target_id_++;
        target.x = world_center(0);
        target.y = world_center(1);
        target.z = world_center(2);
        target.x_width = x_width;
        target.y_width = y_width;
        target.z_width = z_width;
        target.is_human = false;
        target.is_dynamic = true;
        target.is_tracked = true;
        target.timestamp = ros::Time::now();
        target.last_seen = ros::Time::now();
        target.position_history.push_back(world_center);
        target.time_history.push_back(target.timestamp);
        if(target.position_history.size() > max_history_size_) {
            target.position_history.pop_front();
            target.time_history.pop_front();
        }

    } catch (const std::exception& e) {
        ROS_ERROR("Exception in estimateTarget3D: %s", e.what());
    } catch (...) {
        ROS_ERROR("Unknown exception in estimateTarget3D");
    }
}

void TargetDetector::updateTargetTracking() {
    // Predict positions for all tracked targets
    predictTargetPositions();
    
    // Update tracking information
    for(auto& pair : tracked_targets_) {
        Target3D& target = pair.second;
        
        // Remove old targets
        if((ros::Time::now() - target.timestamp).toSec() > 2.0) {
            target.is_tracked = false;
        }
        
        // Publish trajectory
        if(target.is_tracked) {
            publishTrajectory(target);
        }
    }
}

void TargetDetector::predictTargetPositions() {
    ros::Time current_time = ros::Time::now();
    
    for(auto& pair : tracked_targets_) {
        Target3D& target = pair.second;
        if(!target.is_tracked) continue;
        
        double dt = (current_time - target.timestamp).toSec();
        if(dt > 0) {
            predictKalmanFilter(target, dt);
        }
    }
}

void TargetDetector::initializeKalmanFilter(Target3D& target) {
    // Initialize state vector [x, y, z, vx, vy, vz, ax, ay, az]
    target.state.setZero();
    target.state(0) = target.x;
    target.state(1) = target.y;
    target.state(2) = target.z;
    
    // Initialize covariance matrix
    target.covariance.setIdentity();
    target.covariance *= 0.1;  // Initial uncertainty
}

void TargetDetector::predictKalmanFilter(Target3D& target, double dt) {
    // State transition matrix
    Eigen::Matrix<double, 9, 9> F;
    F.setIdentity();
    F.block<3,3>(0,3) = Eigen::Matrix3d::Identity() * dt;
    F.block<3,3>(0,6) = Eigen::Matrix3d::Identity() * 0.5 * dt * dt;
    F.block<3,3>(3,6) = Eigen::Matrix3d::Identity() * dt;
    
    // Process noise
    Eigen::Matrix<double, 9, 9> Q;
    Q.setIdentity();
    Q.block<3,3>(0,0) *= 0.1;  // Position noise
    Q.block<3,3>(3,3) *= 0.1;  // Velocity noise
    Q.block<3,3>(6,6) *= 0.1;  // Acceleration noise
    Q *= dt;
    
    // Predict
    target.state = F * target.state;
    target.covariance = F * target.covariance * F.transpose() + Q;
    
    // Update target state
    target.x = target.state(0);
    target.y = target.state(1);
    target.z = target.state(2);
    target.vx = target.state(3);
    target.vy = target.state(4);
    target.vz = target.state(5);
    target.ax = target.state(6);
    target.ay = target.state(7);
    target.az = target.state(8);
}

void TargetDetector::updateKalmanFilter(Target3D& target, const Eigen::Vector3d& measurement) {
    // Measurement matrix
    Eigen::Matrix<double, 3, 9> H;
    H.setZero();
    H.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
    
    // Measurement noise
    Eigen::Matrix3d R;
    R.setIdentity();
    R *= 0.1;  // Measurement noise
    
    // Kalman gain
    Eigen::Matrix<double, 9, 3> K = target.covariance * H.transpose() * 
                                  (H * target.covariance * H.transpose() + R).inverse();
    
    // Update
    Eigen::Vector3d innovation = measurement - H * target.state;
    target.state = target.state + K * innovation;
    target.covariance = (Eigen::Matrix<double, 9, 9>::Identity() - K * H) * target.covariance;
}

void TargetDetector::publishVisualization() {
    // 发布3D框
    std::vector<Target3D> targets;
    for(const auto& pair : tracked_targets_) {
        if(pair.second.is_tracked) {
            targets.push_back(pair.second);
            // 同时发布轨迹
            publishTrajectory(pair.second);
        }
    }
    
    if (!targets.empty()) {
        publish3DBox(targets, yolo_bboxes_pub_, 1.0, 0.0, 0.0);  // 使用红色显示3D框
        // ROS_INFO("Published %zu 3D boxes", targets.size());
    }
}

void TargetDetector::publish3DBox(const std::vector<Target3D>& targets,
                                const ros::Publisher& publisher,
                                double r, double g, double b) {
    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker marker;
    
    marker.header.frame_id = "map";  // 确保使用正确的坐标系
    marker.header.stamp = ros::Time::now();
    marker.ns = "target_boxes";
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.05;  // 线宽
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 0.5;  // 半透明
    marker.lifetime = ros::Duration(0.1);  // 设置较短的生命周期，确保实时更新
    
    for(size_t i = 0; i < targets.size(); ++i) {
        const auto& target = targets[i];
        marker.id = i;
        
        // 设置位置和方向
        marker.pose.position.x = target.x;
        marker.pose.position.y = target.y;
        marker.pose.position.z = target.z;
        marker.pose.orientation.w = 1.0;
        
        // 设置尺寸
        marker.scale.x = target.x_width;
        marker.scale.y = target.y_width;
        marker.scale.z = target.z_width;

    }
    
    if (!markers.markers.empty()) {
        publisher.publish(markers);
        // ROS_DEBUG("Published %zu markers", markers.markers.size());
    } else {
        // ROS_WARN("No markers to publish!");
    }
}

void TargetDetector::publishTrajectory(const Target3D& target) {
    if(target.position_history.size() < 2) return;
    
    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker marker;
    
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "target_trajectories";
    marker.id = target.id;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.05;  // 线宽
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(1.0);  // 增加轨迹显示时间
    
    // 添加轨迹点
    for(size_t i = 0; i < target.position_history.size(); ++i) {
        geometry_msgs::Point p;
        p.x = target.position_history[i](0);
        p.y = target.position_history[i](1);
        p.z = target.position_history[i](2);
        marker.points.push_back(p);
    }
    
    markers.markers.push_back(marker);
    trajectory_pub_.publish(markers);
}

void TargetDetector::calculateMAD(const std::vector<double>& values,
                                const std::vector<double>& weights,
                                double& median,
                                double& mad) {
    if(values.empty()) {
        median = 0.0;
        mad = 0.0;
        return;
    }
    
    // Calculate weighted median
    std::vector<std::pair<double, double>> value_weight_pairs;
    for(size_t i = 0; i < values.size(); ++i) {
        value_weight_pairs.push_back({values[i], weights[i]});
    }
    std::sort(value_weight_pairs.begin(), value_weight_pairs.end());
    
    double total_weight = 0.0;
    for(const auto& pair : value_weight_pairs) {
        total_weight += pair.second;
    }
    
    double half_weight = total_weight / 2.0;
    double current_weight = 0.0;
    for(const auto& pair : value_weight_pairs) {
        current_weight += pair.second;
        if(current_weight >= half_weight) {
            median = pair.first;
            break;
        }
    }
    
    // Calculate MAD
    std::vector<double> deviations;
    for(size_t i = 0; i < values.size(); ++i) {
        deviations.push_back(std::abs(values[i] - median) * weights[i]);
    }
    std::sort(deviations.begin(), deviations.end());
    mad = deviations[deviations.size() / 2];
}

void TargetDetector::transformBBox(const Eigen::Vector3d& center,
                                 const Eigen::Vector3d& size,
                                 const Eigen::Vector3d& position,
                                 const Eigen::Quaterniond& orientation,
                                 Eigen::Vector3d& new_center,
                                 Eigen::Vector3d& new_size) {
    // Transform center
    new_center = orientation * center + position;
    
    // Transform size (approximate)
    Eigen::Matrix3d R = orientation.toRotationMatrix();
    new_size = R * size;
    new_size = new_size.cwiseAbs();
}

void TargetDetector::odomCallback(const nav_msgs::OdometryConstPtr& odom) {
    std::lock_guard<std::mutex> odom_lock(odom_mutex_);
    
    // 从里程计直接获取位置和姿态（四元数）
    position_ = Eigen::Vector3d(odom->pose.pose.position.x,
                                odom->pose.pose.position.y,
                                odom->pose.pose.position.z);
    orientation_ = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                      odom->pose.pose.orientation.x,
                                      odom->pose.pose.orientation.y,
                                      odom->pose.pose.orientation.z);
    
    odom_received_ = true;
}

void TargetDetector::getCameraPose(const nav_msgs::OdometryConstPtr& odom,
                                 Eigen::Matrix4d& cam_pose,
                                 Eigen::Matrix4d& cam_pose_color) {
    // Convert quaternion to rotation matrix
    Eigen::Quaterniond quat(odom->pose.pose.orientation.w,
                           odom->pose.pose.orientation.x,
                           odom->pose.pose.orientation.y,
                           odom->pose.pose.orientation.z);
    Eigen::Matrix3d rot = quat.toRotationMatrix();
    
    // Create body to map transform
    Eigen::Matrix4d map_to_body;
    map_to_body.setIdentity();
    map_to_body.block<3,3>(0,0) = rot;
    map_to_body.block<3,1>(0,3) = Eigen::Vector3d(odom->pose.pose.position.x,
                                                 odom->pose.pose.position.y,
                                                 odom->pose.pose.position.z);
    
    // Calculate camera poses
    cam_pose = map_to_body * body_to_cam_;
    cam_pose_color = map_to_body * body_to_cam_color_;
}

} // namespace d2p 