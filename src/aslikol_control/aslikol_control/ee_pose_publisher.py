"""Uc efektorun robot tabanina gore konumunu topic olarak yayinlar.

Abonelikler:
  /aslikol/gz_link_poses (tf2_msgs/TFMessage)  Gazebo PosePublisher -> ros_gz_bridge
  /joint_states          (sensor_msgs/JointState)

Yayinlar:
  /aslikol/ee_pose     (geometry_msgs/PoseStamped)  Gazebo fizik motorundan okunan konum
  /aslikol/ee_pose_fk  (geometry_msgs/PoseStamped)  olculen eklem acilarindan duz kinematik

Her iki mesajin frame_id'si base_link'tir.
"""
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage

from aslikol_control.kinematics import (JOINT_NAMES, forward_kinematics, load_geometry,
                                        matrix_to_quaternion, pose_to_matrix)


def _short_name(frame_id: str) -> str:
    """'aslikol::tool0' -> 'tool0'."""
    return frame_id.split('::')[-1]


def _matrix_to_pose_msg(T: np.ndarray, stamp, frame_id: str) -> PoseStamped:
    msg = PoseStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (float(v) for v in T[:3, 3])
    (msg.pose.orientation.x, msg.pose.orientation.y,
     msg.pose.orientation.z, msg.pose.orientation.w) = matrix_to_quaternion(T[:3, :3])
    return msg


class EePosePublisher(Node):

    def __init__(self):
        super().__init__('ee_pose_publisher')
        self.base_link = self.declare_parameter('base_link', 'base_link').value
        self.ee_link = self.declare_parameter('ee_link', 'tool0').value
        self.geom = load_geometry()

        self.pub_gz = self.create_publisher(PoseStamped, '/aslikol/ee_pose', 10)
        self.pub_fk = self.create_publisher(PoseStamped, '/aslikol/ee_pose_fk', 10)
        self.create_subscription(TFMessage, '/aslikol/gz_link_poses', self.on_link_poses,
                                 qos_profile_sensor_data)
        self.create_subscription(JointState, '/joint_states', self.on_joint_states,
                                 qos_profile_sensor_data)
        self._warned_base = False
        self._warned_ee = False
        self.get_logger().info(
            f'{self.base_link} -> {self.ee_link} konumu /aslikol/ee_pose uzerinden yayinlaniyor')

    def on_link_poses(self, msg: TFMessage):
        poses = {}
        stamp = None
        for tf in msg.transforms:
            name = _short_name(tf.child_frame_id)
            t, r = tf.transform.translation, tf.transform.rotation
            poses[name] = pose_to_matrix((t.x, t.y, t.z), (r.x, r.y, r.z, r.w))
            stamp = tf.header.stamp

        T_tool = poses.get(self.ee_link)
        if T_tool is None:
            if not self._warned_ee:
                self.get_logger().warn(
                    f"Gazebo pozlari icinde '{self.ee_link}' bulunamadi. Gelen linkler: "
                    f'{sorted(poses)}')
                self._warned_ee = True
            return

        T_base = poses.get(self.base_link)
        if T_base is None:
            # base_link modelin kok linkidir; model frame'i ile cakistigi kabul edilir.
            if not self._warned_base:
                self.get_logger().warn(
                    f"'{self.base_link}' pozu gelmedi, model frame'i taban kabul ediliyor")
                self._warned_base = True
            T_base = np.eye(4)

        T_base_tool = np.linalg.inv(T_base) @ T_tool
        self.pub_gz.publish(_matrix_to_pose_msg(T_base_tool, stamp, self.base_link))

    def on_joint_states(self, msg: JointState):
        index = {n: i for i, n in enumerate(msg.name)}
        if not all(n in index for n in JOINT_NAMES):
            return
        q = [msg.position[index[n]] for n in JOINT_NAMES]
        T = forward_kinematics(q, self.geom)
        self.pub_fk.publish(_matrix_to_pose_msg(T, msg.header.stamp, self.base_link))


def main(args=None):
    rclpy.init(args=args)
    node = EePosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
