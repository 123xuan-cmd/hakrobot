#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
轨迹处理器
负责处理MoveIt规划的轨迹并发送给机械臂
"""

import rospy
from trajectory_msgs.msg import JointTrajectory
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryResult,
    FollowJointTrajectoryFeedback,
)
import actionlib

class TrajectoryHandler:
    """轨迹处理器"""

    def __init__(self, tcp_client, wait_ack=True, ack_timeout=10.0):
        """
        初始化轨迹处理器

        Args:
            tcp_client: HKTcpClient实例
            wait_ack (bool): 是否等待海康端确认
            ack_timeout (float): 确认超时时间（秒）
        """
        self.tcp_client = tcp_client
        self.wait_ack = wait_ack
        self.ack_timeout = ack_timeout
        self.current_trajectory = None
        self.executing = False

    def execute_trajectory(self, trajectory):
        """
        执行轨迹

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
        self.current_trajectory = trajectory
        total = len(trajectory.points)

        rospy.loginfo(f"开始执行轨迹，共 {total} 个点")
        rospy.loginfo("-" * 50)

        for i, point in enumerate(trajectory.points):
            if rospy.is_shutdown() or not self.executing:
                rospy.logwarn("轨迹执行被中断")
                self.executing = False
                return False

            positions = list(point.positions)

            # 检查关节数量
            if len(positions) != 6:
                rospy.logerr(
                    f"[点 {i+1}/{total}] 关节数量错误: "
                    f"期望6，实际{len(positions)}，跳过"
                )
                continue

            rospy.loginfo(
                f"[点 {i+1:>3}/{total}] t={point.time_from_start.to_sec():.2f}s"
            )

            # ✅ 发送并等待确认
            success = self.tcp_client.send_joint_positions(
                positions,
                wait_ack=self.wait_ack,
                ack_timeout=self.ack_timeout
            )

            if not success:
                rospy.logerr(f"[点 {i+1}/{total}] 发送失败或超时，尝试重连...")
                if not self.tcp_client.reconnect():
                    rospy.logerr("重连失败，停止发送")
                    self.executing = False
                    return False
                # 重连后重试
                success = self.tcp_client.send_joint_positions(
                    positions,
                    wait_ack=self.wait_ack,
                    ack_timeout=self.ack_timeout
                )
                if not success:
                    rospy.logerr(f"[点 {i+1}/{total}] 重连后仍然失败，停止发送")
                    self.executing = False
                    return False

            # ✅ 不再需要固定间隔，海康端确认后自动发下一个点

        rospy.loginfo("-" * 50)
        rospy.loginfo(f"✅ 轨迹执行完毕，共 {total} 个点")
        self.executing = False
        return True

    def stop_execution(self):
        """停止轨迹执行"""
        self.executing = False
        rospy.loginfo("轨迹执行已停止")

class TrajectoryActionServer:
    """轨迹动作服务器"""

    def __init__(self, tcp_client, action_name='joint_trajectory_action', 
                 wait_ack=True, ack_timeout=10.0):
        """
        初始化动作服务器

        Args:
            tcp_client: HKTcpClient实例
            action_name (str): 动作服务器名称
            wait_ack (bool): 是否等待海康端确认
            ack_timeout (float): 确认超时时间（秒）
        """
        self.tcp_client = tcp_client
        self.trajectory_handler = TrajectoryHandler(
            tcp_client, 
            wait_ack=wait_ack, 
            ack_timeout=ack_timeout
        )

        self.server = actionlib.SimpleActionServer(
            action_name,
            FollowJointTrajectoryAction,
            execute_cb=self.execute_callback,
            auto_start=False
        )
        self.server.start()
        rospy.loginfo(f"Trajectory action server started: {action_name}")

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
