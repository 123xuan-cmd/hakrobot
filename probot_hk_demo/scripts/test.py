#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Joint1 测试 ROS 节点
rosrun probot_driver test_joint1_node.py
"""

import rospy
import socket

# ─────────────────────────────────────────
#  配置区
# ─────────────────────────────────────────
ROBOT_IP           = '192.168.2.64'
ROBOT_PORT         = 8080
WAIT_AFTER_CONNECT = 2.0
STEP_INTERVAL      = 3.0
# ─────────────────────────────────────────

class JointTestNode:

    def __init__(self):
        rospy.init_node('joint_test_node', anonymous=True)

        self.ip   = rospy.get_param('~robot_ip',   ROBOT_IP)
        self.port = rospy.get_param('~robot_port', ROBOT_PORT)

        self.sock = None

    # ──────────────────────────────────────
    #  TCP
    # ──────────────────────────────────────
    def connect(self):
        rospy.loginfo(f"连接 {self.ip}:{self.port} ...")
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.sock.settimeout(5.0)
        self.sock.connect((self.ip, self.port))
        rospy.loginfo(f"✓ 连接成功，等待海康端就绪 {WAIT_AFTER_CONNECT}s ...")
        rospy.sleep(WAIT_AFTER_CONNECT)
        rospy.loginfo("✓ 就绪，开始测试")

    def disconnect(self):
        if self.sock:
            self.sock.close()
            self.sock = None
            rospy.loginfo("TCP 连接已关闭")

    def send(self, positions_deg):
        """发送6个关节角度（单位：度）"""
        data_str = ",".join([f"{d:.4f}" for d in positions_deg]) + "\n"
        self.sock.sendall(data_str.encode('ascii'))
        rospy.loginfo(f"→ 发送: {data_str.strip()}")

    def step(self, positions_deg, desc):
        """发送一步并等待"""
        rospy.loginfo("=" * 50)
        rospy.loginfo(desc)
        rospy.loginfo("=" * 50)
        self.send(positions_deg)
        rospy.sleep(STEP_INTERVAL)

    # ──────────────────────────────────────
    #  测试流程
    # ──────────────────────────────────────
    def run_tests(self):

        # 步骤1：j1 转动 +20°，其余关节保持 0
        self.step(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "步骤1：j1 → +20°，其余关节不动"
        )

        # 步骤2：所有关节归零（返回原点）
        self.step(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "步骤2：所有关节归零，返回原点"
        )

        rospy.loginfo("✅ 测试完成")

    # ──────────────────────────────────────
    #  入口
    # ──────────────────────────────────────
    def start(self):
        try:
            self.connect()
            self.run_tests()
        except rospy.ROSInterruptException:
            rospy.logwarn("ROS 中断，测试停止")
        except Exception as e:
            rospy.logerr(f"❌ 错误: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.disconnect()

# ─────────────────────────────────────────
if __name__ == '__main__':
    node = JointTestNode()
    node.start()
