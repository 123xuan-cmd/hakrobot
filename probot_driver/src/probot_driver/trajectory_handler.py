#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
轨迹处理器
负责处理MoveIt规划的轨迹并发送给机械臂

策略：
  直接发送 MoveIt 规划的原始点，不做任何插值
"""

import rospy
from trajectory_msgs.msg import JointTrajectory
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryResult,
)
import actionlib

class TrajectoryHandler:
    """轨迹处理器"""

    def __init__(self, tcp_client, send_interval=0.05):
        """
        初始化轨迹处理器

        Args:
            tcp_client (HKTcpClient): TCP客户端实例
            send_interval (float):    每个点发送间隔（秒，默认50ms）
        """
        self.tcp_client    = tcp_client
        self.send_interval = send_interval
        self.executing     = False

    def execute_trajectory(self, trajectory):
        """
        执行轨迹：直接逐点发送 MoveIt 原始规划点

        Args:
            trajectory (JointTrajectory): 轨迹消息

        Returns:
            bool: 执行是否成功
        """
        if not self.tcp_client.is_connected():
            rospy.logerr("TCP client not connected")
            return False

        if len(trajectory.points) == 0:
            rospy.logwarn("Empty trajectory")
            return False

        self.executing = True
        total = len(trajectory.points)

        rospy.loginfo("=" * 55)
        rospy.loginfo(f"原始路径点数 : {total} 个")
        rospy.loginfo(f"发送间隔     : {self.send_interval * 1000:.0f} ms/点")
        rospy.loginfo(f"预计总耗时   : {total * self.send_interval:.1f} s")
        rospy.loginfo("-" * 55)

        for i, point in enumerate(trajectory.points):
            if rospy.is_shutdown() or not self.executing:
                rospy.logwarn("轨迹执行被中断")
                self.executing = False
                return False

            positions = list(point.positions)

            if len(positions) != 6:
                rospy.logerr(
                    f"[点 {i+1}/{total}] 关节数量错误: "
                    f"期望6，实际{len(positions)}，跳过"
                )
                continue


            rospy.logdebug(f"[点 {i+1:>3}/{total}] 发送中...")

            # ── 发送，不等待确认 ─────────────────────────
            success = self.tcp_client.send_joint_positions(positions)

            if not success:
                rospy.logerr(f"[点 {i+1}/{total}] 发送失败，尝试重连...")
                if not self.tcp_client.reconnect():
                    rospy.logerr("重连失败，停止发送")
                    self.executing = False
                    return False
                # 重连后重试一次
                success = self.tcp_client.send_joint_positions(positions)
                if not success:
                    rospy.logwarn(f"[点 {i+1}/{total}] 重连后仍失败，跳过此点")
                    continue

            # ── 固定间隔（最后一个点不等待）────────────────
            if i < total - 1:
                rospy.sleep(self.send_interval)

        rospy.loginfo("-" * 55)
        rospy.loginfo(f"✅ 轨迹发送完毕，共 {total} 个点")
        rospy.loginfo("=" * 55)
        self.executing = False
        return True

    def stop_execution(self):
        """停止轨迹执行"""
        self.executing = False
        rospy.loginfo("轨迹执行已停止")

class TrajectoryActionServer:
    """轨迹动作服务器"""

    def __init__(self, tcp_client, action_name='joint_trajectory_action',
                 send_interval=0.05):
        """
        初始化动作服务器

        Args:
            tcp_client (HKTcpClient): TCP客户端实例
            action_name (str):        动作服务器名称
            send_interval (float):    每个点发送间隔（秒，默认50ms）
        """
        self.tcp_client = tcp_client
        self.trajectory_handler = TrajectoryHandler(
            tcp_client,
            send_interval=send_interval
        )

        self.server = actionlib.SimpleActionServer(
            action_name,
            FollowJointTrajectoryAction,
            execute_cb=self.execute_callback,
            auto_start=False
        )
        self.server.start()
        rospy.loginfo(f"Trajectory action server started: {action_name}")
        rospy.loginfo(f"  发送间隔={send_interval * 1000:.0f}ms")

    def execute_callback(self, goal):
        """执行回调"""
        rospy.loginfo("收到轨迹目标")
        result = FollowJointTrajectoryResult()

        success = self.trajectory_handler.execute_trajectory(goal.trajectory)

        if success:
            result.error_code = FollowJointTrajectoryResult.SUCCESSFUL
            self.server.set_succeeded(result)
            rospy.loginfo("✅ 轨迹执行成功")
        else:
            result.error_code = FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED
            self.server.set_aborted(result)
            rospy.logerr("❌ 轨迹执行失败")
