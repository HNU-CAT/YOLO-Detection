#include "d2p/target_detector.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <limits>
#include "d2p/Tracker.h"

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

    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
        
        std::lock_guard<std::mutex> depth_lock(depth_mutex_);
        processDepthImage(cv_ptr->image, aligned_depth_image_);

        // 1. Predict step for all existing trackers
        updateTargetTracking();

        // 2. Extract measurements from detections
        std::vector<Eigen::Vector3d> current_measurements;
        std::vector<Eigen::Vector3d> measurement_sizes;
        for(const auto& detection : detections->detections) {
            Eigen::Vector3d world_center;
            Eigen::Vector3d size_body;
            if(estimateTarget3D(detection, aligned_depth_image_, world_center, size_body)) {
                current_measurements.push_back(world_center);
                measurement_sizes.push_back(size_body);
            }
        }
        
        // 3. Data association (simple nearest neighbor)
        std::vector<bool> matched_measurements(current_measurements.size(), false);
        std::vector<int> track_to_meas_map(tracked_targets_.size(), -1);
        
        int track_idx = 0;
        for(auto& pair : tracked_targets_) {
            Target3D& target = pair.second;
            if (!target.is_tracked) {
                track_idx++;
                continue;
            }

            double min_dist = max_tracking_distance_;
            int best_match_idx = -1;

            for(int i = 0; i < current_measurements.size(); ++i) {
                if(matched_measurements[i]) continue;
                
                double dist = (Eigen::Vector3d(target.x, target.y, target.z) - current_measurements[i]).norm();
                if(dist < min_dist) {
                    min_dist = dist;
                    best_match_idx = i;
                }
            }
            
            if(best_match_idx != -1) {
                track_to_meas_map[track_idx] = best_match_idx;
                matched_measurements[best_match_idx] = true;
            }
            track_idx++;
        }

        // 4. Update step
        track_idx = 0;
        for(auto& pair : tracked_targets_) {
            int meas_idx = track_to_meas_map[track_idx];
            if(meas_idx != -1) {
                Target3D& target = pair.second;
                TrackerState measurement_state;
                measurement_state.position = current_measurements[meas_idx].cast<float>();
                target.kf_tracker->update(measurement_state);
                
                TrackerState corrected_state = target.kf_tracker->getState();
                target.x = corrected_state.position(0);
                target.y = corrected_state.position(1);
                target.z = corrected_state.position(2);
                target.vx = corrected_state.velocity(0);
                target.vy = corrected_state.velocity(1);
                target.vz = corrected_state.velocity(2);
                
                target.x_width = measurement_sizes[meas_idx](0);
                target.y_width = measurement_sizes[meas_idx](1);
                target.z_width = measurement_sizes[meas_idx](2);
                target.last_seen = ros::Time::now();

                target.position_history.push_back(Eigen::Vector3d(target.x, target.y, target.z));
                if(target.position_history.size() > max_history_size_) {
                    target.position_history.pop_front();
                }
            }
            track_idx++;
        }

        // 5. Create new trackers for unmatched detections
        for(int i = 0; i < current_measurements.size(); ++i) {
            if(!matched_measurements[i]) {
                Target3D new_target;
                new_target.id = next_target_id_++;
                
                TrackerState initial_state;
                initial_state.position = current_measurements[i].cast<float>();
                new_target.kf_tracker = std::make_shared<Tracker>(initial_state);

                TrackerState state = new_target.kf_tracker->getState();
                new_target.x = state.position(0);
                new_target.y = state.position(1);
                new_target.z = state.position(2);
                new_target.vx = state.velocity(0);
                new_target.vy = state.velocity(1);
                new_target.vz = state.velocity(2);
                
                new_target.x_width = measurement_sizes[i](0);
                new_target.y_width = measurement_sizes[i](1);
                new_target.z_width = measurement_sizes[i](2);
                
                new_target.is_tracked = true;
                new_target.timestamp = ros::Time::now();
                new_target.last_seen = ros::Time::now();
                new_target.position_history.push_back(Eigen::Vector3d(new_target.x, new_target.y, new_target.z));

                tracked_targets_[new_target.id] = new_target;
            }
        }
        
        // 6. Publish visualization
        publishVisualization();

    } catch (const std::exception& e) {
        ROS_ERROR("Exception in syncCallback: %s", e.what());
    }
}

bool TargetDetector::estimateTarget3D(const vision_msgs::Detection2D& detection,
                                    const cv::Mat& depth_image,
                                    Eigen::Vector3d& world_center,
                                    Eigen::Vector3d& size_body) {
    if (depth_image.empty() || detection.bbox.size_x <= 0 || detection.bbox.size_y <= 0) {
        return false;
    }

    double center_x = detection.bbox.center.x;
    double center_y = detection.bbox.center.y;
    double size_x = detection.bbox.size_x;
    double size_y = detection.bbox.size_y;

    if (center_x < 0 || center_x >= img_cols_ || center_y < 0 || center_y >= img_rows_) {
        return false;
    }

    int top_x = static_cast<int>(std::max(0.0, std::round(center_x - size_x/2.0)));
    int top_y = static_cast<int>(std::max(0.0, std::round(center_y - size_y/2.0)));
    int width = static_cast<int>(std::min(static_cast<double>(img_cols_ - top_x), std::round(size_x)));
    int height = static_cast<int>(std::min(static_cast<double>(img_rows_ - top_y), std::round(size_y)));

    if (width <= 0 || height <= 0) {
        return false;
    }
    
    std::vector<double> depth_values;
    const double inv_factor = 1.0 / depth_scale_;
    double min_depth = std::numeric_limits<double>::max();
    double max_depth = std::numeric_limits<double>::min();

    for(int v = top_y; v < top_y + height; ++v) {
        for(int u = top_x; u < top_x + width; ++u) {
            uint16_t depth_raw = depth_image.at<uint16_t>(v, u);
            double depth = depth_raw * inv_factor;
            if(depth >= depth_min_value_ && depth <= depth_max_value_) {
                depth_values.push_back(depth);
                min_depth = std::min(min_depth, depth);
                max_depth = std::max(max_depth, depth);
            }
        }
    }

    if(depth_values.empty()) {
        return false;
    }

    std::sort(depth_values.begin(), depth_values.end());
    double depth_median = depth_values[depth_values.size()/2];

    Eigen::Vector3d center_cam;
    center_cam(0) = (center_x - cx_) * depth_median / fx_;
    center_cam(1) = (center_y - cy_) * depth_median / fy_;
    center_cam(2) = depth_median;

    Eigen::Vector3d center_body;
    center_body(0) = center_cam(2);
    center_body(1) = -center_cam(0);
    center_body(2) = -center_cam(1);

    double x_width = (size_x / fx_) * center_cam(2);
    double y_width = (size_y / fy_) * center_cam(2);
    double z_width = (max_depth - min_depth);
    
    size_body(0) = z_width;
    size_body(1) = x_width;
    size_body(2) = y_width;

    Eigen::Vector3d dummy_size;
    transformBBox(center_body, size_body, position_, orientation_, world_center, dummy_size);
    
    return true;
}

void TargetDetector::updateTargetTracking() {
    ros::Time current_time = ros::Time::now();
    for(auto it = tracked_targets_.begin(); it != tracked_targets_.end(); ) {
        Target3D& target = it->second;
        
        if((current_time - target.last_seen).toSec() > 2.0) {
            target.is_tracked = false;
            it = tracked_targets_.erase(it);
        } else {
            if (target.kf_tracker) {
                TrackerState predicted_state = target.kf_tracker->predict();
                target.x = predicted_state.position(0);
                target.y = predicted_state.position(1);
                target.z = predicted_state.position(2);
                target.vx = predicted_state.velocity(0);
                target.vy = predicted_state.velocity(1);
                target.vz = predicted_state.velocity(2);
            }
            ++it;
        }
    }
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