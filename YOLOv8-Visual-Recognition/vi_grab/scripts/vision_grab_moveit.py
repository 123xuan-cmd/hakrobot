#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys
import numpy as np
import minimalmodbus
import threading
from std_msgs.msg import String
from vi_msgs.msg import ObjectInfo
import moveit_commander
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from copy import deepcopy

# ============================================================
# 参数配置
# ============================================================
qx = -0.7252377665226762
qy = -0.6873142029357756
qz = -0.027661831421397748
qw =  0.029397134798712445

rotation_matrix = np.array([
    [1 - 2*(qy**2 + qz**2),   2*(qx*qy - qz*qw),       2*(qx*qz + qy*qw)],
    [2*(qx*qy + qz*qw),       1 - 2*(qx**2 + qz**2),    2*(qy*qz - qx*qw)],
    [2*(qx*qz - qy*qw),       2*(qy*qz + qx*qw),        1 - 2*(qx**2 + qy**2)]
])

translation_vector = np.array([0.8069257484410287, -0.21426010259993233, 1.3317410126411309])

GRASP_APPROACH_Z = 0.10
GRASP_DOWN_Z     = 0.02
LIFT_Z           = 0.15
READY_JOINT_VALUES = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

TOOL_OFFSET_Z = 0.27   # 单位：米

# ============================================================
# 夹爪 Modbus 寄存器地址
# ============================================================
ENABLE           = 0x0100
POSITION_HIGH_8  = 0x0102
POSITION_LOW_8   = 0x0103
SPEED            = 0x0104
FORCE            = 0x0105
ACCELERATION     = 0x0106
DEACCELERATION   = 0x0107
MOTION_TRIGGER   = 0x0108
RETURN_ZERO      = 0x0402

# 夹爪参数
GRIPPER_PORT           = '/dev/ttyUSB0'
GRIPPER_BAUD           = 115200
GRIPPER_SLAVE          = 1
GRIPPER_OPEN_POSITION  = 100
GRIPPER_CLOSE_POSITION = 9000
GRIPPER_SPEED          = 100
GRIPPER_FORCE_FULL     = 100
GRIPPER_FORCE_GENTLE   = 30

# 全局变量
object_msg   = None
is_busy      = False
move_group   = None
traj_pub     = None
instrument   = None
modbus_lock  = threading.Lock()

# ============================================================
# 夹爪底层控制
# ============================================================
def gripper_init():
    """初始化夹爪 Modbus 通信"""
    global instrument
    instrument = minimalmodbus.Instrument(GRIPPER_PORT, GRIPPER_SLAVE)
    instrument.serial.baudrate = GRIPPER_BAUD
    instrument.serial.timeout  = 1

    with modbus_lock:
        instrument.write_register(ENABLE, 1, functioncode=6)

    rospy.loginfo("✅ 夹爪 Modbus 初始化完成")
    rospy.sleep(0.5)

def gripper_set_force(force_percent):
    """设置夹爪力度 (0-100)"""
    global instrument
    with modbus_lock:
        instrument.write_register(FORCE, force_percent, functioncode=6)
    rospy.loginfo(f"   力度设置: {force_percent}%")

def gripper_set_speed(speed):
    """设置夹爪速度 (0-100)"""
    global instrument
    with modbus_lock:
        instrument.write_register(SPEED, speed, functioncode=6)

def gripper_move_to(position):
    """
    命令夹爪移动到指定位置并触发运动
    position: 0(全开) ~ 9000(全关)
    """
    global instrument
    with modbus_lock:
        instrument.write_long(POSITION_HIGH_8, position)
        instrument.write_register(MOTION_TRIGGER, 1, functioncode=6)
    rospy.loginfo(f"   夹爪目标位置: {position}")

def gripper_open():
    """打开夹爪到最大"""
    rospy.loginfo("📢 夹爪：打开到最大")
    gripper_set_force(GRIPPER_FORCE_FULL)
    gripper_set_speed(GRIPPER_SPEED)
    gripper_move_to(GRIPPER_OPEN_POSITION)
    rospy.sleep(2.0)
    rospy.loginfo("✓ 夹爪已完全打开")

def gripper_grasp_gentle():
    """力度受限闭合：碰到矿泉水瓶时自动停止"""
    rospy.loginfo("📢 夹爪：力度受限闭合")
    rospy.loginfo(f"   力度={GRIPPER_FORCE_GENTLE}%，碰到物体将自动停止")

    gripper_set_force(GRIPPER_FORCE_GENTLE)
    gripper_set_speed(50)
    gripper_move_to(GRIPPER_CLOSE_POSITION)
    rospy.sleep(4.0)

    rospy.loginfo("✓ 夹爪已碰到物体并保持当前位置")
    rospy.loginfo("  （电机持续保持扭矩，不会松开）")

# ============================================================
# 辅助函数
# ============================================================
def convert_camera_to_base(x, y, z):
    """相机坐标系 → 机器人基座坐标系"""
    p_cam  = np.array([x, y, z]).reshape(3, 1)
    p_base = rotation_matrix.dot(p_cam) + translation_vector.reshape(3, 1)
    return p_base.flatten()

def tcp_to_flange(target_x, target_y, target_z):
    """将夹爪TCP目标位置 → 转换为法兰（末端执行器）目标位置"""
    global move_group

    current_pose = move_group.get_current_pose().pose
    q = current_pose.orientation
    ex, ey, ez, ew = q.x, q.y, q.z, q.w

    R_ee = np.array([
        [1 - 2*(ey**2 + ez**2),   2*(ex*ey - ez*ew),       2*(ex*ez + ey*ew)],
        [2*(ex*ey + ez*ew),       1 - 2*(ex**2 + ez**2),    2*(ey*ez - ex*ew)],
        [2*(ex*ez - ey*ew),       2*(ey*ez + ex*ew),        1 - 2*(ex**2 + ey**2)]
    ])

    tool_offset_base = R_ee @ np.array([0.0, 0.0, TOOL_OFFSET_Z])

    rospy.loginfo(
        f"🔧 TCP补偿: offset=["
        f"{tool_offset_base[0]:.3f}, "
        f"{tool_offset_base[1]:.3f}, "
        f"{tool_offset_base[2]:.3f}]"
    )

    fx = target_x - tool_offset_base[0]
    fy = target_y - tool_offset_base[1]
    fz = target_z - tool_offset_base[2]

    rospy.loginfo(f"   TCP目标:  ({target_x:.3f}, {target_y:.3f}, {target_z:.3f})")
    rospy.loginfo(f"   法兰目标: ({fx:.3f}, {fy:.3f}, {fz:.3f})")

    return fx, fy, fz

# ============================================================
# ✅ 轨迹采样函数（只发送终点）
# ============================================================
def downsample_trajectory_points(all_points):
    """
    只发送轨迹的终点（最后一个点）
    
    策略：每段运动只需要知道目标位置，中间过程由机械臂自己插值
    """
    total = len(all_points)

    if total == 0:
        rospy.logwarn("   轨迹点数为 0")
        return []

    # ✅ 只取最后一个点（终点）
    sampled = [all_points[-1]]

    rospy.loginfo(f"   ✂️ 只发送终点: {total} → 1 个点（索引 [{total-1}]）")

    return sampled

# ============================================================
# ✅ 轨迹发布
# ============================================================
def publish_to_hk_driver(plan):
    global traj_pub, move_group

    all_points  = plan.joint_trajectory.points
    joint_names = plan.joint_trajectory.joint_names
    total_orig  = len(all_points)

    if total_orig == 0:
        rospy.logerr("❌ 规划路径点数为 0")
        return False

    rospy.loginfo(f"MoveIt 原始轨迹: {total_orig} 个点")

    sampled_points = downsample_trajectory_points(all_points)
    total = len(sampled_points)

    if total == 0:
        rospy.logerr("❌ 采样后点数为 0")
        return False

    rospy.loginfo(f"实际发送: {total} 个点")

    traj_msg = JointTrajectory()
    traj_msg.header.stamp = rospy.Time.now()
    traj_msg.joint_names  = joint_names

    for i, src_point in enumerate(sampled_points):
        point = JointTrajectoryPoint()
        point.positions       = src_point.positions
        point.velocities      = src_point.velocities
        point.accelerations   = src_point.accelerations
        point.time_from_start = src_point.time_from_start
        traj_msg.points.append(point)

        csv_str = ",".join([f"{p:.4f}" for p in src_point.positions])
        rospy.loginfo(
            f"[终点] t={src_point.time_from_start.to_sec():.2f}s  {csv_str}"
        )

    rospy.loginfo("🔄 MoveIt 虚拟机器人开始同步运动...")
    move_group.execute(plan, wait=False)

    traj_pub.publish(traj_msg)
    rospy.loginfo(f"✓ 已发布 {total} 个点至 joint_path_command → hk_driver_node")

    wait_time = sampled_points[-1].time_from_start.to_sec() + 1.0
    rospy.loginfo(f"⏳ 等待执行 {wait_time:.1f}s...")
    rospy.sleep(wait_time)

    return True

# ============================================================
# 运动规划
# ============================================================
def move_to_pose_joint_space(x, y, z):
    global move_group

    rospy.loginfo(f"🎯 TCP目标位置: X={x:.3f}, Y={y:.3f}, Z={z:.3f}")

    fx, fy, fz = tcp_to_flange(x, y, z)

    current_pose = move_group.get_current_pose().pose
    rospy.loginfo(
        f"📍 当前法兰位置: X={current_pose.position.x:.3f}, "
        f"Y={current_pose.position.y:.3f}, "
        f"Z={current_pose.position.z:.3f}"
    )

    target_pose = deepcopy(current_pose)
    target_pose.position.x = fx
    target_pose.position.y = fy
    target_pose.position.z = fz

    move_group.set_start_state_to_current_state()
    move_group.set_pose_target(target_pose)

    rospy.loginfo("🔄 开始规划...")
    result = move_group.plan()

    if isinstance(result, tuple):
        success, plan, planning_time, error_code = result
    else:
        success = (len(result.joint_trajectory.points) > 0)
        plan    = result

    move_group.clear_pose_targets()

    if success:
        rospy.loginfo("✓ 规划成功")
        return publish_to_hk_driver(plan)
    else:
        rospy.logerr("❌ 规划失败")
        return False

def move_to_joint_values(joint_values):
    global move_group

    rospy.loginfo(f"🎯 目标关节角度: {[f'{j:.3f}' for j in joint_values]}")

    move_group.set_start_state_to_current_state()
    move_group.set_joint_value_target(joint_values)

    result = move_group.plan()
    if isinstance(result, tuple):
        success, plan, _, _ = result
    else:
        success = (len(result.joint_trajectory.points) > 0)
        plan    = result

    if success:
        rospy.loginfo("✓ 关节规划成功")
        return publish_to_hk_driver(plan)
    else:
        rospy.logerr("❌ 关节规划失败")
        return False

# ============================================================
# 抓取流程（恢复两段运动：先到上方，再下探）
# ============================================================
def pick_and_place_task(base_xyz):
    x, y, z = base_xyz[0], base_xyz[1], base_xyz[2]

    rospy.loginfo("=" * 60)
    rospy.loginfo("🤖 开始执行抓取任务")
    rospy.loginfo("=" * 60)

    # Step 1: 夹爪开到最大
    rospy.loginfo("📍 Step 1: 夹爪打开到最大")
    gripper_open()

    # Step 2: 机械臂移动到目标上方 0.1m（只发送终点）
    rospy.loginfo("📍 Step 2: 机械臂移动到目标上方 0.1m")
    if not move_to_pose_joint_space(x, y, z + GRASP_APPROACH_Z):
        rospy.logerr("❌ Step 2 失败")
        return False

    # Step 3: 机械臂下探到目标点（只发送终点）
    rospy.loginfo("📍 Step 3: 机械臂下探到目标点")
    if not move_to_pose_joint_space(x, y, z + GRASP_DOWN_Z):
        rospy.logerr("❌ Step 3 失败")
        return False

    # Step 4: 到达目标点，等待 8 秒
    rospy.loginfo("📍 Step 4: 到达目标点，等待 5 秒...")
    for i in range(5, 0, -1):
        rospy.loginfo(f"   ⏳ 闭合前倒计时 {i} 秒...")
        rospy.sleep(1.0)

    # Step 5: 力度受限闭合（碰到矿泉水瓶自动停止）
    rospy.loginfo("📍 Step 5: 夹爪闭合，碰到物体自动停止")
    gripper_grasp_gentle()

    # Step 6: 夹爪保持不动，机械臂回零位（只发送终点）
    rospy.loginfo("📍 Step 6: 夹爪保持夹紧，带着瓶子回零位")
    move_to_joint_values(READY_JOINT_VALUES)

    # Step 7: 到达零位后，等待 30 秒
    rospy.loginfo("📍 Step 7: 已到达零位，保持夹紧，等待 30 秒...")
    for i in range(30, 0, -1):
        rospy.loginfo(f"   ⏳ 松开前倒计时 {i} 秒...")
        rospy.sleep(1.0)

    # Step 8: 夹爪松开
    rospy.loginfo("📍 Step 8: 夹爪松开")
    gripper_open()

    rospy.loginfo("=" * 60)
    rospy.loginfo("✅ 任务完成")
    rospy.loginfo("=" * 60)
    return True

# ============================================================
# 回调函数
# ============================================================
def object_pose_callback(data):
    global object_msg, is_busy

    if object_msg is None:
        return
    if is_busy:
        return
    if data.object_class != object_msg.data:
        return

    is_busy = True
    rospy.loginfo("=" * 60)
    rospy.loginfo(f"✨ 检测到目标: {data.object_class}")
    rospy.loginfo(f"📷 相机坐标: X={data.x:.3f}, Y={data.y:.3f}, Z={data.z:.3f}")

    try:
        base_xyz = convert_camera_to_base(data.x, data.y, data.z)
        rospy.loginfo(
            f"🔄 基座坐标: X={base_xyz[0]:.3f}, "
            f"Y={base_xyz[1]:.3f}, "
            f"Z={base_xyz[2]:.3f}"
        )

        if base_xyz[2] < 0 or base_xyz[2] > 0.8:
            rospy.logwarn(f"⚠️ Z 坐标 {base_xyz[2]:.3f} 超出安全范围，跳过")
            is_busy = False
            return

        if not pick_and_place_task(base_xyz):
            rospy.logerr("❌ 任务执行失败")

    except Exception as e:
        rospy.logerr(f"❌ 回调异常: {e}")
        import traceback
        traceback.print_exc()
    finally:
        is_busy = False

# ============================================================
# 主程序
# ============================================================
if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('vision_grab_joint_space')

    rospy.loginfo("=" * 60)
    rospy.loginfo("🚀 视觉抓取系统启动（关节空间规划 + TCP补偿）")
    rospy.loginfo(f"   工具偏移: Z = {TOOL_OFFSET_Z * 100:.0f} cm")
    rospy.loginfo(f"   抓取力度: {GRIPPER_FORCE_GENTLE}%（碰触自停）")
    rospy.loginfo(f"   轨迹策略: 每段只发送终点（共3段：上方→目标→零位）")
    rospy.loginfo("=" * 60)

    # 步骤 1: 初始化夹爪
    rospy.loginfo("步骤 1: 初始化夹爪")
    try:
        gripper_init()
        gripper_open()
    except Exception as e:
        rospy.logerr(f"❌ 夹爪初始化失败: {e}")
        sys.exit(1)

    # 步骤 2: 初始化 MoveIt
    rospy.loginfo("步骤 2: 初始化 MoveIt")
    try:
        move_group = moveit_commander.MoveGroupCommander("hkrobot")
        move_group.allow_replanning(True)
        move_group.set_pose_reference_frame('base_link')
        move_group.set_goal_position_tolerance(0.001)
        move_group.set_goal_orientation_tolerance(0.001)
        move_group.set_max_acceleration_scaling_factor(0.5)
        move_group.set_max_velocity_scaling_factor(0.5)
        move_group.set_planning_time(5.0)
        rospy.loginfo("✅ MoveIt 就绪")
        rospy.loginfo(f"   末端执行器: {move_group.get_end_effector_link()}")
    except Exception as e:
        rospy.logerr(f"❌ MoveIt 初始化失败: {e}")
        sys.exit(1)

    # 步骤 3: 创建轨迹发布器
    rospy.loginfo("步骤 3: 创建轨迹发布器")
    traj_pub = rospy.Publisher('joint_path_command', JointTrajectory, queue_size=1)
    rospy.sleep(1.0)

    # 步骤 4: 等待目标选择
    rospy.loginfo("=" * 60)
    rospy.loginfo("步骤 5: 等待目标选择")
    rospy.loginfo("请在另一个终端运行:")
    rospy.loginfo("  rostopic pub /choice_object std_msgs/String \"data: 'bottle'\" -1")
    rospy.loginfo("=" * 60)

    object_msg = rospy.wait_for_message('/choice_object', String)
    rospy.loginfo(f"🎯 已选择目标: {object_msg.data}")

    # 步骤 5: 开始监听
    rospy.loginfo("步骤 6: 开始监听目标检测")
    rospy.Subscriber("/object_pose", ObjectInfo, object_pose_callback, queue_size=1)

    rospy.loginfo("=" * 60)
    rospy.loginfo("✅ 系统就绪，等待检测目标...")
    rospy.loginfo("=" * 60)

    rospy.spin()
