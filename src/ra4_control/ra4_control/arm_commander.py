"""arm_controller'a FollowJointTrajectory action'i ile hedef gonderen ortak yardimci node.

fk_eval ve sequence node'lari bu sinifi kullanir. Butun hedefler gonderilmeden once
eklem limitlerine gore dogrulanir (gecersiz hedef kontrolcuye hic gitmez).
"""
from __future__ import annotations

import math
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from ra4_control.kinematics import JOINT_NAMES, load_geometry, validate_joints

ACTION_NAME = '/arm_controller/follow_joint_trajectory'
HOME = (0.0, 0.0, 0.0, 0.0)


class ArmCommander(Node):

    def __init__(self, node_name: str):
        # Bu node'lar yalnizca simulasyonla kullanilir: varsayilan olarak /clock (sim saati)
        super().__init__(node_name, parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.geom = load_geometry()
        # Hareket suresi: en buyuk eklem farki / bu hiz (rad/s), en az min_move_time saniye
        self.move_speed = float(self.declare_parameter('move_speed', 0.5).value)
        self.min_move_time = float(self.declare_parameter('min_move_time', 2.0).value)
        if not (0.05 <= self.move_speed <= 2.0):
            raise ValueError('move_speed 0.05..2.0 rad/s araliginda olmali')

        self._client = ActionClient(self, FollowJointTrajectory, ACTION_NAME)
        self.joint_state: JointState | None = None
        self.ee_pose_gz: PoseStamped | None = None
        self.create_subscription(JointState, '/joint_states', self._on_js, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/ra4/ee_pose', self._on_ee,
                                 qos_profile_sensor_data)

    # ------------------------------------------------------------ abonelikler
    def _on_js(self, msg: JointState):
        self.joint_state = msg

    def _on_ee(self, msg: PoseStamped):
        self.ee_pose_gz = msg

    # ------------------------------------------------------------ yardimcilar
    def measured_joints(self) -> np.ndarray | None:
        js = self.joint_state
        if js is None:
            return None
        index = {n: i for i, n in enumerate(js.name)}
        if not all(n in index for n in JOINT_NAMES):
            return None
        return np.array([js.position[index[n]] for n in JOINT_NAMES])

    def wait_for_system(self, timeout_s: float = 60.0) -> bool:
        """Action sunucusu ve /joint_states hazir olana kadar bekler (duvar saati)."""
        if not self._client.wait_for_server(timeout_sec=timeout_s):
            self.get_logger().error(f'{ACTION_NAME} bulunamadi. Simulasyon calisiyor mu?')
            return False
        deadline = time.monotonic() + timeout_s
        while self.measured_joints() is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.measured_joints() is None:
            self.get_logger().error('/joint_states gelmiyor (joint_state_broadcaster yuklu mu?)')
            return False
        return True

    def wait_sim(self, seconds: float, wall_timeout_s: float = 120.0):
        """Simulasyon saatine gore bekler (RTF < 1 olsa da dogru sure)."""
        start = self.get_clock().now()
        deadline = time.monotonic() + wall_timeout_s
        while (self.get_clock().now() - start).nanoseconds * 1e-9 < seconds:
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.monotonic() > deadline:
                self.get_logger().warning('wait_sim: duvar saati zaman asimi')
                break

    def move_to(self, q_target, duration: float | None = None) -> bool:
        """Kolu tek noktali bir yorungeyle q_target'a goturur, bitene kadar bekler."""
        q_target = validate_joints(q_target, self.geom)
        q_now = self.measured_joints()
        if duration is None:
            delta = 0.0 if q_now is None else float(np.max(np.abs(q_target - q_now)))
            duration = max(self.min_move_time, delta / self.move_speed)

        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in q_target]
        point.velocities = [0.0] * len(JOINT_NAMES)
        sec = int(math.floor(duration))
        point.time_from_start = Duration(sec=sec, nanosec=int((duration - sec) * 1e9))

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINT_NAMES)
        goal.trajectory.points = [point]

        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=15.0)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error('Hedef kontrolcu tarafindan kabul edilmedi')
            return False

        result_future = handle.get_result_async()
        # Simulasyon gercek zamandan yavas olabilir; duvar saati zaman asimini genis tut.
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration * 10 + 30)
        wrapped = result_future.result()
        if wrapped is None:
            self.get_logger().error('Hareket sonucu zaman asimina ugradi, hedef iptal ediliyor')
            handle.cancel_goal_async()
            return False
        if wrapped.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(
                f'Hareket basarisiz: kod={wrapped.result.error_code} {wrapped.result.error_string}')
            return False
        return True
