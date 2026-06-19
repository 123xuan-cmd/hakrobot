#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from vi_msgs.msg import ObjectInfo

# 目标类别
TARGET_CLASS = "bottle"

# 固定平面深度（相机坐标系下，单位：米）
# 这个值必须根据你现场桌面与相机的距离修改
Z_PLANE = 1.15

# 海康相机内参（已更新）
FX = 4006.35522
FY = 4001.64525
CX = 1624.43955
CY = 944.95876

# 模型权重路径
MODEL_PATH = "yolov8n.pt"

# 图像话题名称（确认你的海康相机发布的话题）
IMAGE_TOPIC = "/hikrobot_camera/rgb"  # ✅ 已修复：匹配你的真实相机话题

# =========================
# 全局变量
# =========================
bridge = CvBridge()
model = None
object_pub = None
latest_image = None

def pixel_to_camera(u, v, z):
    """使用固定深度 z，把像素坐标反投影到相机坐标系"""
    x = (u - CX) * z / FX
    y = (v - CY) * z / FY
    return x, y, z

def image_callback(msg):
    """接收图像话题"""
    global latest_image
    try:
        latest_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except Exception as e:
        rospy.logerr("Failed to convert image: %s", e)

def process_image():
    """处理图像并发布检测结果"""
    global latest_image, model, object_pub

    if latest_image is None:
        return

    color_image = latest_image.copy()

    # YOLO 推理
    results = model.predict(color_image, conf=0.5, verbose=False)
    result = results[0]
    canvas = result.plot()

    boxes = result.boxes
    if boxes is None or len(boxes) == 0:
        cv2.imshow("detection", canvas)
        cv2.waitKey(1)
        return

    for box in boxes:
        cls_id = int(box.cls[0].item())
        name = result.names[cls_id]

        # 只处理 bottle
        if name != TARGET_CLASS:
            continue

        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
        u = int((x1 + x2) / 2)
        v = int((y1 + y2) / 2)

        # 固定高度反投影
        x_cam, y_cam, z_cam = pixel_to_camera(u, v, Z_PLANE)

        # 可视化
        cv2.circle(canvas, (u, v), 5, (0, 255, 0), -1)
        text = f"{name}: ({x_cam:.3f}, {y_cam:.3f}, {z_cam:.3f})"
        cv2.putText(canvas, text, (u + 10, v - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        # 发布消息
        msg = ObjectInfo()
        msg.object_class = name
        msg.x = float(x_cam)
        msg.y = float(y_cam)
        msg.z = float(z_cam)
        object_pub.publish(msg)

        rospy.loginfo("Published %s pose: (%.3f, %.3f, %.3f)", name, x_cam, y_cam, z_cam)

    cv2.imshow("detection", canvas)
    cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node("object_detect", anonymous=True)

    # 加载模型
    model = YOLO(MODEL_PATH)
    rospy.loginfo("YOLOv8 model loaded: %s", MODEL_PATH)

    # 发布器
    object_pub = rospy.Publisher("object_pose", ObjectInfo, queue_size=10)

    # 订阅图像话题
    rospy.Subscriber(IMAGE_TOPIC, Image, image_callback, queue_size=1)

    # 创建显示窗口
    cv2.namedWindow("detection", cv2.WINDOW_NORMAL)

    # 定时处理图像
    rate = rospy.Rate(10)  # 10Hz
    try:
        while not rospy.is_shutdown():
            process_image()
            rate.sleep()
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
