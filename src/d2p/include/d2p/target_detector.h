#pragma once

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <vision_msgs/Detection2DArray.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/opencv.hpp>
#include <mutex>
#include <deque>
#include <unordered_map>
#include "d2p/Tracker.h"

namespace d2p {

struct Target3D {
    int id;
    double x, y, z;
    double x_width, y_width, z_width;
    double vx, vy, vz;
    double ax, ay, az;
    bool is_human;
    bool is_dynamic;
    bool is_tracked;
    ros::Time timestamp;
    ros::Time last_seen;
    std::deque<Eigen::Vector3d> position_history;
    std::deque<ros::Time> time_history;
    
    // Kalman filter tracker
    std::shared_ptr<Tracker> kf_tracker;
};

class TargetDetector {
public:
    explicit TargetDetector(ros::NodeHandle& nh);
    ~TargetDetector() = default;

private:
    // ROS related
    ros::NodeHandle& nh_;
    ros::Publisher bbox_pub_;
    ros::Publisher yolo_bboxes_pub_;
    ros::Publisher depth_viz_pub_;
    ros::Publisher trajectory_pub_;
    
    message_filters::Subscriber<sensor_msgs::Image> depth_sub_;
    message_filters::Subscriber<vision_msgs::Detection2DArray> detection_sub_;
    ros::Subscriber odom_sub_;
    
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, vision_msgs::Detection2DArray> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Sync;
    boost::shared_ptr<Sync> sync_;

    // Topic names
    std::string depth_image_topic_;
    std::string detections_topic_;
    std::string odometry_topic_;
    std::string target_position_topic_;
    std::string bbox_visualization_topic_;
    std::string depth_visualization_topic_;
    std::string trajectory_visualization_topic_;

    // Queue sizes
    int depth_image_queue_size_;
    int detections_queue_size_;
    int odometry_queue_size_;
    int visualization_queue_size_;
    
    // Camera parameters
    double depth_scale_;
    int depth_filter_margin_;
    int img_rows_;
    int img_cols_;
    double depth_min_value_;
    double depth_max_value_;
    std::vector<double> color_intrinsics_;
    double fx_, fy_, cx_, cy_;
    
    // Target parameters
    double target_size_x_;
    double target_size_y_;
    double target_size_z_;
    double min_target_size_;
    double max_target_size_;
    
    // Tracking parameters
    double max_tracking_distance_;
    int max_history_size_;
    double min_tracking_confidence_;
    double max_tracking_velocity_;
    double max_tracking_acceleration_;
    
    // Transform matrices
    Eigen::Matrix4d body_to_cam_;
    Eigen::Matrix4d body_to_cam_color_;
    Eigen::Vector3d position_;
    Eigen::Quaterniond orientation_;
    Eigen::Vector3d position_color_;
    Eigen::Quaterniond orientation_color_;
    
    // Data storage
    cv::Mat aligned_depth_image_;
    cv::Mat filtered_depth_image_;
    std::mutex depth_mutex_;
    std::mutex odom_mutex_;
    bool odom_received_;
    
    // Target tracking
    std::unordered_map<int, Target3D> tracked_targets_;
    int next_target_id_;

    // Methods
    void initializeParameters(const ros::NodeHandle& nh);
    void setupSubscribers();
    void setupPublishers();
    void loadTransforms(const ros::NodeHandle& nh);
    
    void depthCallback(const sensor_msgs::ImageConstPtr& depth_msg);
    void detectionCallback(const vision_msgs::Detection2DArrayConstPtr& detections);
    void odomCallback(const nav_msgs::OdometryConstPtr& odom);
    void syncCallback(const sensor_msgs::ImageConstPtr& depth_msg,
                     const vision_msgs::Detection2DArrayConstPtr& detections);
    
    void processDepthImage(const cv::Mat& input, cv::Mat& output);
    void publishDepthVisualization(const cv::Mat& depth_image);
    bool estimateTarget3D(const vision_msgs::Detection2D& detection,
                         const cv::Mat& depth_image,
                         Eigen::Vector3d& world_center,
                         Eigen::Vector3d& size_body);
    void updateTargetTracking();
    void publishVisualization();
    
    // Helper functions
    void transformBBox(const Eigen::Vector3d& center,
                      const Eigen::Vector3d& size,
                      const Eigen::Vector3d& position,
                      const Eigen::Quaterniond& orientation,
                      Eigen::Vector3d& new_center,
                      Eigen::Vector3d& new_size);
    void getCameraPose(const nav_msgs::OdometryConstPtr& odom,
                      Eigen::Matrix4d& cam_pose,
                      Eigen::Matrix4d& cam_pose_color);
    void publish3DBox(const std::vector<Target3D>& targets,
                     const ros::Publisher& publisher,
                     double r, double g, double b);
    void publishTrajectory(const Target3D& target);
};

} // namespace d2p 