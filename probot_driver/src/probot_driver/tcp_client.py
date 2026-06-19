#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
海康机械臂TCP客户端
负责与机械臂的TCP通讯
"""

import socket
import threading
import time
import math
import rospy

class HKTcpClient:
    """海康机械臂TCP客户端"""

    def __init__(self, host='192.168.2.64', port=8080):
        self.host         = host
        self.port         = port
        self.socket       = None
        self.connected    = False
        self.lock         = threading.Lock()
        self.connect_wait = 2.0

        rospy.loginfo(f"TCP Client initialized: {host}:{port}")

    def connect(self, timeout=5.0):
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.socket.settimeout(timeout)
            self.socket.connect((self.host, self.port))
            self.connected = True
            rospy.loginfo(f"✓ Connected to Hikrobot at {self.host}:{self.port}")

            rospy.loginfo(f"  等待海康端 SocketAccept 就绪 ({self.connect_wait}s)...")
            time.sleep(self.connect_wait)
            rospy.loginfo("  海康端就绪，可以发送数据")
            return True
        except socket.error as e:
            rospy.logerr(f"✗ Failed to connect: {e}")
            self.connected = False
            return False

    def disconnect(self):
        with self.lock:
            if self.socket:
                try:
                    self.socket.shutdown(socket.SHUT_WR)
                    time.sleep(0.2)
                    self.socket.close()
                except Exception:
                    pass
                self.socket = None
            self.connected = False
            rospy.loginfo("Disconnected from Hikrobot")

    def send_joint_positions(self, joint_positions):
        """
        发送关节位置（6个关节），不等待确认，直接返回

        格式：'-17.4268,-0.0286,0.0000,-17.3982,0.0000,0.0000\n'
        - 弧度 → 角度（math.degrees）
        - 保留4位小数
        - 逗号分隔，\n 结尾，ASCII 编码

        Args:
            joint_positions (list): 6个关节角度（弧度，来自ROS）

        Returns:
            bool: 发送是否成功
        """
        if not self.connected:
            rospy.logwarn("Not connected to robot")
            return False

        if len(joint_positions) != 6:
            rospy.logerr(f"Expected 6 joint positions, got {len(joint_positions)}")
            return False

        deg_positions = [math.degrees(float(pos)) for pos in joint_positions]
        data_str      = ",".join([f"{d:.4f}" for d in deg_positions]) + "\n"
        raw           = data_str.encode('ascii')

        rospy.logdebug(f"  发送: {repr(data_str)}")

        with self.lock:
            try:
                self.socket.sendall(raw)
                return True
            except socket.error as e:
                rospy.logerr(f"Failed to send data: {e}")
                self.connected = False
                return False

    def send_command(self, command):
        if not self.connected:
            rospy.logwarn("Not connected to robot")
            return False

        with self.lock:
            try:
                if not command.endswith('\n'):
                    command += '\n'
                self.socket.sendall(command.encode('ascii'))
                rospy.logdebug(f"Sent command: {command.strip()}")
                return True
            except socket.error as e:
                rospy.logerr(f"Failed to send command: {e}")
                self.connected = False
                return False

    def receive_response(self, buffer_size=1024, timeout=1.0):
        if not self.connected:
            rospy.logwarn("Not connected to robot")
            return None

        with self.lock:
            try:
                self.socket.settimeout(timeout)
                data = self.socket.recv(buffer_size)
                return data.decode('ascii', errors='ignore')
            except socket.timeout:
                rospy.logdebug("Receive timeout")
                return None
            except socket.error as e:
                rospy.logerr(f"Failed to receive data: {e}")
                self.connected = False
                return None

    def is_connected(self):
        return self.connected

    def reconnect(self, max_attempts=3):
        self.disconnect()
        for attempt in range(max_attempts):
            rospy.loginfo(f"Reconnecting... Attempt {attempt + 1}/{max_attempts}")
            if self.connect():
                return True
            time.sleep(2.0)
        return False
