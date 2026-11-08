"""Kol uzerindeki sensorleri canli gorsellestirir (matplotlib penceresi).

Abonelikler:
  /joint_states          eklem acilari          -> grafik
  /aslikol/imu           bilek IMU acisal hizi  -> grafik
  /aslikol/camera/image  bilek kamerasi         -> goruntu
  /aslikol/ee_pose       uc efektor konumu      -> grafik

Kullanim (simulasyon calisirken, ayri bir terminalde):
  ros2 run aslikol_control visualizer
  ros2 run aslikol_control visualizer --ros-args -p window:=60.0
"""
from __future__ import annotations

import threading
from collections import deque

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from matplotlib.animation import FuncAnimation
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, JointState

from aslikol_control.kinematics import JOINT_NAMES

MAX_IMAGE_SIDE = 4096  # bozuk/asiri buyuk goruntu mesajlarina karsi ust sinir


def image_to_array(msg: Image) -> np.ndarray | None:
    """sensor_msgs/Image -> (H, W, 3) veya (H, W) uint8 dizi. Desteklenmeyen/bozuk ise None."""
    channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1}.get(msg.encoding.lower())
    h, w, step = int(msg.height), int(msg.width), int(msg.step)
    if channels is None or not (0 < h <= MAX_IMAGE_SIDE and 0 < w <= MAX_IMAGE_SIDE):
        return None
    if step < w * channels or len(msg.data) < h * step:
        return None
    arr = np.frombuffer(bytes(msg.data[:h * step]), dtype=np.uint8).reshape(h, step)
    arr = arr[:, :w * channels].reshape(h, w, channels)
    if channels == 1:
        return arr[:, :, 0].copy()
    arr = arr[:, :, :3]
    if msg.encoding.lower().startswith('bgr'):
        arr = arr[:, :, ::-1]
    return arr.copy()


class SensorVisualizer(Node):

    def __init__(self):
        super().__init__('visualizer', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.window = float(self.declare_parameter('window', 30.0).value)
        if not (5.0 <= self.window <= 600.0):
            raise ValueError('window 5..600 s araliginda olmali')

        self.lock = threading.Lock()
        self.t0: float | None = None
        self.joints = deque(maxlen=20000)  # (t, q1, q2, q3, q4)
        self.gyro = deque(maxlen=20000)    # (t, wx, wy, wz)
        self.ee = deque(maxlen=20000)      # (t, x, y, z)
        self.image: np.ndarray | None = None

        self.create_subscription(JointState, '/joint_states', self.on_js, qos_profile_sensor_data)
        self.create_subscription(Imu, '/aslikol/imu', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Image, '/aslikol/camera/image', self.on_image,
                                 qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/aslikol/ee_pose', self.on_ee,
                                 qos_profile_sensor_data)

    # ------------------------------------------------------------ zaman
    def _time(self, stamp) -> float:
        t = stamp.sec + stamp.nanosec * 1e-9
        if t == 0.0:
            t = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = t
        return t - self.t0

    # ------------------------------------------------------------ abonelikler
    def on_js(self, msg: JointState):
        index = {n: i for i, n in enumerate(msg.name)}
        if not all(n in index for n in JOINT_NAMES):
            return
        with self.lock:
            t = self._time(msg.header.stamp)
            self.joints.append((t, *(msg.position[index[n]] for n in JOINT_NAMES)))

    def on_imu(self, msg: Imu):
        w = msg.angular_velocity
        with self.lock:
            self.gyro.append((self._time(msg.header.stamp), w.x, w.y, w.z))

    def on_ee(self, msg: PoseStamped):
        p = msg.pose.position
        with self.lock:
            self.ee.append((self._time(msg.header.stamp), p.x, p.y, p.z))

    def on_image(self, msg: Image):
        arr = image_to_array(msg)
        if arr is None:
            self.get_logger().warn(f'Goruntu cozulemedi (encoding={msg.encoding})',
                                   throttle_duration_sec=5.0)
            return
        with self.lock:
            self.image = arr

    # ------------------------------------------------------------ cizim
    def show(self):
        fig, axes = plt.subplots(2, 2, figsize=(12, 7))
        fig.canvas.manager.set_window_title('aslikol - sensor gorsellestirme')
        ax_j, ax_img, ax_g, ax_ee = axes[0, 0], axes[0, 1], axes[1, 0], axes[1, 1]

        lines_j = [ax_j.plot([], [], label=n)[0] for n in JOINT_NAMES]
        ax_j.set(title='Eklem acilari (/joint_states)', xlabel='t [s]', ylabel='[rad]')
        lines_g = [ax_g.plot([], [], label=f'w{a}')[0] for a in 'xyz']
        ax_g.set(title='Bilek IMU acisal hizi (/aslikol/imu)', xlabel='t [s]', ylabel='[rad/s]')
        lines_e = [ax_ee.plot([], [], label=a)[0] for a in 'xyz']
        ax_ee.set(title='Uc efektor konumu, tabana gore (/aslikol/ee_pose)',
                  xlabel='t [s]', ylabel='[m]')
        for ax in (ax_j, ax_g, ax_ee):
            ax.grid(True, alpha=0.3)
            ax.legend(loc='upper left', fontsize=8)

        ax_img.set_title('Bilek kamerasi (/aslikol/camera/image)')
        ax_img.axis('off')
        img_artist = ax_img.imshow(np.zeros((240, 320, 3), dtype=np.uint8))
        fig.tight_layout()

        def set_series(lines, data, ax):
            if not data:
                return
            arr = np.asarray(data)
            t = arr[:, 0]
            for i, line in enumerate(lines):
                line.set_data(t, arr[:, i + 1])
            t_end = t[-1]
            ax.set_xlim(max(0.0, t_end - self.window), max(self.window, t_end))
            ax.relim()
            ax.autoscale_view(scalex=False, scaley=True)

        def update(_frame):
            with self.lock:
                joints, gyro, ee = list(self.joints), list(self.gyro), list(self.ee)
                image = self.image
            set_series(lines_j, joints, ax_j)
            set_series(lines_g, gyro, ax_g)
            set_series(lines_e, ee, ax_ee)
            if image is not None:
                img_artist.set_data(image)
                img_artist.set_extent((0, image.shape[1], image.shape[0], 0))
            return []

        self._anim = FuncAnimation(fig, update, interval=200, cache_frame_data=False)
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = SensorVisualizer()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        node.show()  # pencere kapatilana kadar bloklar
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
