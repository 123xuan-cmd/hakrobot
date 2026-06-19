#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys
import math
import moveit_commander
from moveit_commander import MoveGroupCommander
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class MoveItCartesianDemo:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('moveit_cartesian_demo', anonymous=True)

        arm = MoveGroupCommander('hkrobot')
        arm.allow_replanning(True)
        arm.set_max_acceleration_scaling_factor(0.5)
        arm.set_max_velocity_scaling_factor(0.5)

        self.traj_pub = rospy.Publisher(
            'joint_path_command',
            JointTrajectory,
            queue_size=10
        )
        rospy.sleep(1.0)

        current_joints = arm.get_current_joint_values()
        rospy.loginfo(f"当前关节角度（弧度）: {current_joints}")
        rospy.loginfo(f"当前关节角度（度）:   {[round(math.degrees(j), 2) for j in current_joints]}")

        target_deg = [-3.78, -57.5, 8.0, -4.0, -65.0, 1.7] 
       # target_deg = [0.0, -25.0, 5.0, -0.0, -10.0, 35.0]
       # target_deg = [0.0, -15.0, 15.0, -0.0, -15.0, 50.0] 
       # target_deg = [-5.0, -13.0, 6.0, 0.0, -30.0, 35.0]
       # target_deg = [-13.0, -30.0, 6.0, 0.0, -60.0, 0.0] 
       # target_deg = [-15.0, -15.0, 15.0, 0.0, 8.0, 35.0]
       # target_deg = [0.0, -30.0, 15.0, 0.0, -45.0, 10.0] 
       # target_deg = [-15.0, -30.0, 15.0, 0.0, -50.0, 50.0]
       # target_deg = [-15.0, -30.0, 18.0, 0.0, -65.0, 45.0] 
       # target_deg = [-8.0, -50.0, 8.0, 0.0, -45.0, 30.0]
       # target_deg = [-0.0, -50.0, 8.0, 0.0, -65.0, 65.0]
       # target_deg = [-13.0, -35.0, 15.0, 12.0, -60.0, 35.0]
       # target_deg = [-8.0, -25.0, 10.0, 0.0, -25.0, 60.0] 
       # target_deg = [-0.0, -30.0, 15.0, 0.0, -60.0, 45.0]
       # target_deg = [-15.0, -38.0, 25.0, 0.0, -60.0, 15.0] 
       # target_deg = [-17.0, -30.0, 16.0, 10.0, -35.0, 12.0]
       # target_deg = [-0.0, -25.0, 8.0, 0.0, -35.0, 60.0] 
      
        target_rad = [math.radians(d) for d in target_deg]
        rospy.loginfo(f"目标关节角度（度）: {target_deg}")

        arm.set_start_state_to_current_state()
        arm.set_joint_value_target(target_rad)

        success, plan, planning_time, error_code = arm.plan()

        if not success:
            rospy.logerr("规划失败，请检查目标角度是否超过限位或存在碰撞")
            moveit_commander.roscpp_shutdown()
            return

        rospy.loginfo(f"规划成功（耗时: {planning_time:.2f}s）")

        joint_names = plan.joint_trajectory.joint_names
        num_joints  = len(joint_names)
        rospy.loginfo(f"规划关节数量: {num_joints}")
        rospy.loginfo(f"关节名称: {joint_names}")

        if num_joints != 6:
            rospy.logwarn(
                f"⚠️  关节数量为 {num_joints}，"
                f"hk_driver 要求6个，请检查 planning group 配置"
            )

        # ── 同步执行：MoveIt + 海康 同时启动 ──────────
        rospy.loginfo("MoveIt 开始执行（非阻塞）...")
        arm.execute(plan, wait=False)        # ← MoveIt 同步动画，不阻塞
        self.publish_to_driver(plan)         # ← 立即发给海康端

        # 等待轨迹执行完成（取轨迹最后一个点的时间 + 缓冲）
        if plan.joint_trajectory.points:
            duration = plan.joint_trajectory.points[-1].time_from_start.to_sec()
            rospy.loginfo(f"等待轨迹执行完成（预计 {duration:.2f}s）...")
            rospy.sleep(duration + 1.0)
        else:
            rospy.sleep(3.0)

        rospy.loginfo("运动完成")
        moveit_commander.roscpp_shutdown()
        sys.exit(0)

    def publish_to_driver(self, plan):
        all_points  = plan.joint_trajectory.points
        joint_names = plan.joint_trajectory.joint_names
        total       = len(all_points)

        rospy.loginfo(f"轨迹共 {total} 个点，全部发布至 hk_driver_node")

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

            csv_str = ",".join([f"{p:.4f}" for p in src_point.positions])
            rospy.loginfo(
                f"[点 {i+1:>3}/{total}] "
                f"t={src_point.time_from_start.to_sec():.2f}s  "
                f"{csv_str}"
            )

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
