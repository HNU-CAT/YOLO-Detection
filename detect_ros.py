import rospy
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np
from geometry_msgs.msg import PoseStamped

# 初始化 YOLO 模型
model = YOLO('runs/detect/my_data4/weights/best.pt')  # 替换为你的模型路径
real_position = [0.0, 0.0, 0.0]
# 初始化 CvBridge
bridge = CvBridge()

camera_position = None  # [x, y, z]
camera_orientation = None  # [qx, qy, qz, qw]
# 相机到机体的旋转矩阵 cam2body_
cam2body_ = np.array([
    [0.0, 0.0, 1.0, 0.0],
    [-1.0, 0.0, 0.0, 0.0],
    [0.0, -1.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 1.0]
])

def quaternion_to_rotation_matrix(qx, qy, qz, qw):
    """将四元数转换为旋转矩阵"""
    rotation_matrix = np.array([
        [1 - 2 * (qy**2 + qz**2), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx**2 + qz**2), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx**2 + qy**2)]
    ])
    return rotation_matrix

def transform_to_global(X, Y, Z):
    """将相机坐标系中的点转换到全局坐标系"""
    global camera_position, camera_orientation, cam2body_

    if camera_position is None or camera_orientation is None:
        rospy.logwarn("相机位姿尚未接收，无法转换坐标")
        return None

    qx, qy, qz, qw = camera_orientation
    rotation_matrix = quaternion_to_rotation_matrix(qx, qy, qz, qw)
    point_camera = np.array([X, Y, Z, 1])  # 添加齐次坐标

    point_body = np.dot(cam2body_, point_camera)
    point_global = np.dot(rotation_matrix, point_body[:3]) + camera_position  # 丢弃齐次坐标，点到全局坐标系

    return point_global


def depth_image_callback(msg):
    """处理深度图像消息"""
    global depth_image
    try:
        depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    except Exception as e:
        rospy.logerr(f"深度图像处理错误: {e}")
        depth_image = None
    

def odom_callback(msg):
    """处理相机位姿信息"""
    global camera_position, camera_orientation
    camera_position = np.array([msg.pose.pose.position.x,
                                 msg.pose.pose.position.y,
                                 msg.pose.pose.position.z])
    camera_orientation = np.array([msg.pose.pose.orientation.x,
                                    msg.pose.pose.orientation.y,
                                    msg.pose.pose.orientation.z,
                                    msg.pose.pose.orientation.w])

def image_callback(msg):
    """处理RGB图像并进行目标检测和坐标转换"""
    global depth_image

    if depth_image is None:
        rospy.logwarn("深度图像未接收，跳过处理")
        return

    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        # 使用 YOLO 进行目标检测
        results = model.predict(source=cv_image, conf=0.5, save=False, show=False, verbose=False)
        detections = results[0].boxes.data.cpu().numpy()  # 获取检测结果 [x1, y1, x2, y2, confidence, class]

        # 创建深度图像副本
        depth_image_copy = np.copy(depth_image)
        # 用于区分相同类别目标的计数器
        target_id = 0

        for detection in detections:
            x1, y1, x2, y2, conf, cls = detection
            cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)  # 计算目标中心点
            depth = depth_image[cy, cx]   # 深度图单位为毫米，转换为米

            # 根据相机参数计算目标相机坐标系位置
            fx, fy = 554.254691191187, 554.254691191187  # 替换为相机的内参焦距
            cx_offset, cy_offset = 320.5, 240.5  # 替换为相机的主点坐标
            X = (cx - cx_offset) * depth / fx
            Y = (cy - cy_offset) * depth / fy
            Z = depth
    
            # 转换到全局坐标系
            global_coords = transform_to_global(X, Y, Z)
            if global_coords is not None:
                # rospy.loginfo(f"目标 {int(cls)} 在全局坐标系中的位置: X={global_coords[0]:.2f}, Y={global_coords[1]:.2f}, Z={global_coords[2]:.2f}")
                
                # 为每个目标生成唯一的ID
                target_id += 1
                detected_position = PoseStamped()  # 创建消息
                detected_position.pose.position.x = global_coords[0]  # 设置 X 坐标
                detected_position.pose.position.y = global_coords[1]  # 设置 Y 坐标
                detected_position.pose.position.z = global_coords[2]  # 设置 Z 坐标

                # 给消息添加唯一的 ID 标识，或者用目标的类别作为标识
                detected_position.header.stamp = rospy.Time.now()
                detected_position.header.frame_id = f"target_{target_id}"  # 使用target_id作为标识
                position_pub.publish(detected_position)  # 发布目标位置
            x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])
            depth_image_copy[y1:y2, x1:x2] = 0
        depth_image_msg = bridge.cv2_to_imgmsg(depth_image_copy, encoding="passthrough")
        depth_image_msg.header.stamp = rospy.Time.now()
        depth_image_msg.header.frame_id = "camera_frame"  # 根据实际情况设置坐标系
        depth_image_pub.publish(depth_image_msg)    
    except Exception as e:
        rospy.logerr(f"处理RGB图像时出错: {e}")
def main():
    rospy.init_node('yolo_ros_detector', anonymous=True)
    # 订阅深度图像
    rospy.Subscriber('/iris_0/realsense/depth_camera/depth/image_raw', Image, depth_image_callback)
    # 订阅RGB图像
    rospy.Subscriber('/iris_0/realsense/depth_camera/color/image_raw', Image, image_callback)
    # 订阅相机位姿信息
    rospy.Subscriber('/iris_0/mavros/vision_odom/odom', Odometry, odom_callback)
    global position_pub  # 声明使用全局变量
    # 发布转换后的无人机位置信息
    position_pub = rospy.Publisher('/drone/position', PoseStamped, queue_size=10)  # 创建发布者
    
    global depth_image_pub
    depth_image_pub = rospy.Publisher('/drone/modified_depth_image', Image, queue_size=10)
    rospy.spin()

if __name__ == "__main__":
    main()
