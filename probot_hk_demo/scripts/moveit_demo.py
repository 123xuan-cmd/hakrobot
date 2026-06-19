#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from copy import deepcopy

class MoveItCartesianDemo:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('moveit_cartesian_demo', anonymous=True)

        # ── MoveIt 初始化 ──────────────────────────────
        arm = MoveGroupCommander('hkrobot')
        arm.allow_replanning(True)
        arm.set_pose_reference_frame('base_link')
        arm.set_goal_position_tolerance(0.001)
        arm.set_goal_orientation_tolerance(0.001)
        arm.set_max_acceleration_scaling_factor(0.5)
        arm.set_max_velocity_scaling_factor(0.5)

        end_effector_link = arm.get_end_effector_link()
        start_pose = arm.get_current_pose(end_effector_link).pose
        print(start_pose)

        # ── 发布到 hk_driver_node 订阅的话题 ──────────
        self.traj_pub = rospy.Publisher(
            'joint_path_command',
            JointTrajectory,
            queue_size=1
        )
        rospy.sleep(0.5)  # 等待Publisher建立连接

        # ── 构造路点 ───────────────────────────────────
        waypoints = []
        waypoints.append(start_pose)

        wpose = deepcopy(start_pose)
        wpose.position.z -= 0.20
        waypoints.append(deepcopy(wpose))

        wpose.position.x += 0.20
        waypoints.append(deepcopy(wpose))

        wpose.position.y += 0.20
        waypoints.append(deepcopy(wpose))

        # ── 路径规划 ───────────────────────────────────
        fraction = 0.0
        maxtries = 100
        attempts = 0
        arm.set_start_state_to_current_state()

        while fraction < 1.0 and attempts < maxtries:
            (plan, fraction) = arm.compute_cartesian_path(
                waypoints,
                0.01,
                True,
            )
            attempts += 1

        if fraction < 1.0:
            rospy.logwarn(f"路径规划失败，覆盖率: {fraction:.2f}")
            moveit_commander.roscpp_shutdown()
            return

        # ── 检查关节数量 ───────────────────────────────
        joint_names = plan.joint_trajectory.joint_names
        num_joints  = len(joint_names)
        rospy.loginfo(f"规划关节数量: {num_joints}")
        rospy.loginfo(f"关节名称: {joint_names}")

        if num_joints != 6:
            rospy.logwarn(
                f"⚠️  关节数量为 {num_joints}，"
                f"hk_driver 要求6个，请检查 planning group 配置"
            )

        # ── 执行运动（MoveIt控制仿真/真实机器人）────────
        rospy.loginfo("开始执行运动...")
        arm.execute(plan, wait=True)
        rospy.loginfo("运动执行完毕")

        # ── 发布完整轨迹至 hk_driver_node ─────────────
        self.publish_to_driver(plan)

        rospy.sleep(1)
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

    def publish_to_driver(self, plan):
        """
        将规划轨迹的全部点发布到 joint_path_command 话题。
        hk_driver_node 收到后会做样条插值再逐点发送给海康端。

        Args:
            plan: MoveIt 规划结果
        """
        all_points  = plan.joint_trajectory.points
        joint_names = plan.joint_trajectory.joint_names
        total       = len(all_points)

        rospy.loginfo(f"轨迹共 {total} 个点，全部发布至 hk_driver_node")

        # ── 构造 JointTrajectory 消息 ──────────────────
        traj_msg = JointTrajectory()
        traj_msg.header.stamp = rospy.Time.now()
        traj_msg.joint_names  = joint_names

        for i, src_point in enumerate(all_points):
            point = JointTrajectoryPoint()
            point.positions       = src_point.positions
            point.velocities      = src_point.velocities
            point.accelerations   = src_point.accelerations
            point.time_from_start = src_point.time_from_start
            traj_msg.points.append(point)

            # 打印每个点内容
            csv_str = ",".join([f"{p:.4f}" for p in src_point.positions])
            rospy.loginfo(
                f"[点 {i+1:>3}/{total}] "
                f"t={src_point.time_from_start.to_sec():.2f}s  "
                f"{csv_str}"
            )

        # ── 一次性发布给 hk_driver_node ────────────────
        self.traj_pub.publish(traj_msg)
        rospy.loginfo(
            f"✓ 已发布全部 {total} 个点至 "
            f"joint_path_command → hk_driver_node → 海康端"
        )

if __name__ == "__main__":
    try:
        MoveItCartesianDemo()
    except rospy.ROSInterruptException:
        pass
