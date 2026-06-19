#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
碰撞管理器
管理场景中的障碍物和碰撞检测
"""

import rospy
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, PoseStamped          # ← 加入 PoseStamped
import moveit_commander

class CollisionManager:
    """碰撞管理器"""

    def __init__(self, scene=None):
        """
        初始化碰撞管理器

        Args:
            scene: PlanningSceneInterface实例（可选）
        """
        if scene is None:
            self.scene = moveit_commander.PlanningSceneInterface()
        else:
            self.scene = scene

        rospy.sleep(0.5)  # 等待场景初始化
        rospy.loginfo("Collision Manager initialized")

    def _to_pose_stamped(self, pose, frame_id="base_link"):
        """
        将 Pose 转换为 PoseStamped（内部工具方法）

        Args:
            pose: geometry_msgs/Pose 或 PoseStamped
            frame_id: 坐标系名称
        Returns:
            PoseStamped
        """
        if isinstance(pose, PoseStamped):
            return pose                       
        
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = frame_id  # ← 关键：加上 header
        pose_stamped.pose = pose
        return pose_stamped

    def add_box(self, name, pose, size):
        """
        添加立方体障碍物

        Args:
            name (str): 障碍物名称
            pose (Pose 或 PoseStamped): 位姿
            size (list): [x, y, z] 尺寸列表
        """
        pose_stamped = self._to_pose_stamped(pose)   # ← 自动转换
        self.scene.add_box(name, pose_stamped, size)
        rospy.sleep(0.5)
        rospy.loginfo(f"✓ Added box: {name} with size {size}")

    def add_cylinder(self, name, pose, height, radius):
        """
        添加圆柱体障碍物

        Args:
            name (str): 障碍物名称
            pose (Pose): geometry_msgs/Pose
            height (float): 高度
            radius (float): 半径
        """
        collision_object = CollisionObject()
        collision_object.header.frame_id = "base_link"
        collision_object.id = name

        cylinder = SolidPrimitive()
        cylinder.type = SolidPrimitive.CYLINDER
        cylinder.dimensions = [height, radius]

        collision_object.primitives.append(cylinder)
        collision_object.primitive_poses.append(pose)
        collision_object.operation = CollisionObject.ADD

        self.scene.add_object(collision_object)
        rospy.sleep(0.5)
        rospy.loginfo(f"✓ Added cylinder: {name} (h={height}, r={radius})")

    def add_sphere(self, name, pose, radius):
        """
        添加球体障碍物

        Args:
            name (str): 障碍物名称
            pose (Pose): geometry_msgs/Pose
            radius (float): 半径
        """
        collision_object = CollisionObject()
        collision_object.header.frame_id = "base_link"
        collision_object.id = name

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [radius]

        collision_object.primitives.append(sphere)
        collision_object.primitive_poses.append(pose)
        collision_object.operation = CollisionObject.ADD

        self.scene.add_object(collision_object)
        rospy.sleep(0.5)
        rospy.loginfo(f"✓ Added sphere: {name} (r={radius})")

    def remove_object(self, name):
        """
        移除障碍物

        Args:
            name (str): 障碍物名称
        """
        self.scene.remove_world_object(name)
        rospy.sleep(0.5)
        rospy.loginfo(f"✓ Removed object: {name}")

    def clear_all_objects(self):
        """清除所有障碍物"""
        known_objects = self.scene.get_known_object_names()
        for obj in known_objects:
            self.scene.remove_world_object(obj)
        rospy.sleep(0.5)
        rospy.loginfo("✓ Cleared all collision objects")

    def add_table(self, height=0.0):
        """
        添加工作台

        Args:
            height (float): 工作台高度
        """
        pose_stamped = PoseStamped()                       
        pose_stamped.header.frame_id = "base_link"           
        pose_stamped.pose.position.x = 0.0
        pose_stamped.pose.position.y = 0.0
        pose_stamped.pose.position.z = height - 0.025
        pose_stamped.pose.orientation.w = 1.0

        self.add_box("table", pose_stamped, [1.0, 1.0, 0.05])

    def add_walls(self):
        """添加墙壁"""

        # 后墙
        pose_back = PoseStamped()                             
        pose_back.header.frame_id = "base_link"
        pose_back.pose.position.x = -0.5
        pose_back.pose.position.y = 0.0
        pose_back.pose.position.z = 0.5
        pose_back.pose.orientation.w = 1.0
        self.add_box("wall_back", pose_back, [0.05, 2.0, 1.0])

        # 左墙
        pose_left = PoseStamped()
        pose_left.header.frame_id = "base_link"
        pose_left.pose.position.x = 0.0
        pose_left.pose.position.y = 0.5
        pose_left.pose.position.z = 0.5
        pose_left.pose.orientation.w = 1.0
        self.add_box("wall_left", pose_left, [2.0, 0.05, 1.0])

        # 右墙
        pose_right = PoseStamped()
        pose_right.header.frame_id = "base_link"
        pose_right.pose.position.x = 0.0
        pose_right.pose.position.y = -0.5
        pose_right.pose.position.z = 0.5
        pose_right.pose.orientation.w = 1.0
        self.add_box("wall_right", pose_right, [2.0, 0.05, 1.0])

    def get_known_objects(self):
        """
        获取已知障碍物列表

        Returns:
            list: 障碍物名称列表
        """
        return self.scene.get_known_object_names()
