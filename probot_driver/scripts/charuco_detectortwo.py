#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf
import tf2_ros
import geometry_msgs.msg
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np

class CharucoDetector:
    def __init__(self):
        rospy.init_node('charuco_detector_node')
        self.bridge = CvBridge()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)
        self.board = aruco.CharucoBoard((9, 5), 0.015, 0.011, self.dict)

        self.cam_matrix = None
        self.dist_coeffs = None
        self.frame_count = 0

        self.img_sub = rospy.Subscriber("image_in", Image, self.image_callback)
        self.info_sub = rospy.Subscriber("camera_info_in", CameraInfo, self.info_callback)
        rospy.loginfo("ChArUco Detector 启动，等待图像...")

    def info_callback(self, msg):
        if self.cam_matrix is None:
            self.cam_matrix = np.array(msg.K).reshape((3, 3))
            self.dist_coeffs = np.array(msg.D)
            rospy.loginfo("相机内参已接收: fx=%.1f fy=%.1f",
                          self.cam_matrix[0,0], self.cam_matrix[1,1])

    def image_callback(self, msg):
        self.frame_count += 1

        if self.frame_count % 30 == 0:
            rospy.loginfo("已收到 %d 帧图像, 内参状态: %s",
                          self.frame_count, "OK" if self.cam_matrix is not None else "未收到")

        if self.cam_matrix is None:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("图像转换失败: %s", e)
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = aruco.detectMarkers(gray, self.dict)

        if ids is None or len(ids) == 0:
            if self.frame_count % 30 == 0:
                rospy.logwarn("未检测到任何 ArUco 码，请检查标定板是否在视野内")
            cv2.imshow("ChArUco Detection", frame)
            cv2.waitKey(1)
            return

        rospy.loginfo_throttle(2, "检测到 %d 个 ArUco 码: %s", len(ids), ids.flatten().tolist())

        ret, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
            corners, ids, gray, self.board)

        if charuco_ids is None or len(charuco_ids) < 4:
            rospy.logwarn_throttle(2, "ChArUco 角点不足（当前=%s），需要至少4个",
                                   len(charuco_ids) if charuco_ids is not None else 0)
            aruco.drawDetectedMarkers(frame, corners, ids)
            cv2.imshow("ChArUco Detection", frame)
            cv2.waitKey(1)
            return

        retval, rvec, tvec = aruco.estimatePoseCharucoBoard(
            charuco_corners, charuco_ids, self.board,
            self.cam_matrix, self.dist_coeffs, None, None)

        if not retval:
            rospy.logwarn_throttle(2, "位姿估计失败")
            cv2.imshow("ChArUco Detection", frame)
            cv2.waitKey(1)
            return

        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "camera_color_optical_frame"
        t.child_frame_id = "camera_marker"

        t.transform.translation.x = tvec[0][0]
        t.transform.translation.y = tvec[1][0]
        t.transform.translation.z = tvec[2][0]

        rot_matrix, _ = cv2.Rodrigues(rvec)
        mat = np.eye(4)
        mat[:3, :3] = rot_matrix
        q = tf.transformations.quaternion_from_matrix(mat)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)
        rospy.loginfo_throttle(1, "TF 已发布: camera_marker, z=%.3fm", tvec[2][0])

        aruco.drawDetectedMarkers(frame, corners, ids)
        aruco.drawDetectedCornersCharuco(frame, charuco_corners, charuco_ids, (0, 255, 0))
        cv2.drawFrameAxes(frame, self.cam_matrix, self.dist_coeffs, rvec, tvec, 0.05)

        cv2.imshow("ChArUco Detection", frame)
        cv2.waitKey(1)

if __name__ == '__main__':
    try:
        node = CharucoDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    cv2.destroyAllWindows()
