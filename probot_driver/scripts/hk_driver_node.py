#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
海康机械臂驱动节点
"""

import rospy
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from probot_driver.tcp_client import HKTcpClient
from probot_driver.trajectory_handler import (
    TrajectoryActionServer,
    TrajectoryHandler,
)
from trajectory_msgs.msg import JointTrajectory

class HKDriverNode:
    """海康机械臂驱动节点"""

    def __init__(self):
        rospy.init_node('hk_driver_node', anonymous=False)

        self.robot_ip          = rospy.get_param('~robot_ip',          '192.168.2.64')
        self.robot_port        = rospy.get_param('~robot_port',         8080)
        self.use_action_server = rospy.get_param('~use_action_server',  True)
        self.send_interval     = rospy.get_param('~send_interval',      0.05)

        self.joint_names = [
            'joint1', 'joint2', 'joint3',
            'joint4', 'joint5', 'j_hkrobot'
        ]

        # 创建TCP客户端并连接
        self.tcp_client = HKTcpClient(self.robot_ip, self.robot_port)

        if not self.tcp_client.connect():
            rospy.logerr("首次连接失败，重试中...")
            if not self.tcp_client.reconnect(max_attempts=5):
                rospy.logfatal("无法连接到机械臂，退出")
                sys.exit(1)

        # 轨迹处理器（话题回调用）
        self.trajectory_handler = TrajectoryHandler(
            self.tcp_client,
            send_interval=self.send_interval
        )

        # 订阅轨迹话题
        self.trajectory_sub = rospy.Subscriber(
            'joint_path_command',
            JointTrajectory,
            self.trajectory_callback,
            queue_size=1
        )

        # 动作服务器（MoveIt接口）
        if self.use_action_server:
            self.action_server = TrajectoryActionServer(
                self.tcp_client,
                'joint_trajectory_action',
                send_interval=self.send_interval
            )

        rospy.loginfo("=" * 60)
        rospy.loginfo("HK Driver Node 启动成功")
        rospy.loginfo(f"机械臂地址   : {self.robot_ip}:{self.robot_port}")
        rospy.loginfo(f"发送间隔     : {self.send_interval * 1000:.0f} ms/点")
        rospy.loginfo(f"动作服务器   : {'启用' if self.use_action_server else '禁用'}")
        rospy.loginfo("=" * 60)

    def trajectory_callback(self, msg):
        """轨迹话题回调"""
        total = len(msg.points)
        rospy.loginfo(f"收到轨迹：{total} 个点，开始发送...")
        success = self.trajectory_handler.execute_trajectory(msg)
        if not success:
            rospy.logerr("轨迹执行失败")

    def run(self):
        rospy.spin()

    def shutdown(self):
        rospy.loginfo("节点关闭，断开连接")
        self.tcp_client.disconnect()

def main():
    try:
        node = HKDriverNode()
        rospy.on_shutdown(node.shutdown)
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main()
