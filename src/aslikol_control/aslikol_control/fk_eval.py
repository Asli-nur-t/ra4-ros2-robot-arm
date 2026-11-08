"""Duz kinematik hata analizi.

10 farkli eklem konfigurasyonu icin:
  1. Kolu konfigurasyona goturur ve oturmasini bekler.
  2. Uc efektor konumunu Gazebo fizik motorundan okur (/aslikol/ee_pose, birkac ornegin ortalamasi).
  3. Konumu 0T5 duz kinematik denklemiyle hesaplar:
       - komut edilen eklem acilariyla       -> p_fk_cmd
       - /joint_states'ten olculen acilarla  -> p_fk_meas
  4. Oklid uzakliklarini hesaplar:
       e_cmd  = |p_fk_cmd  - p_gz|  (kontrolcu takip hatasi + model farki; raporun ana metrigi)
       e_meas = |p_fk_meas - p_gz|  (denklemin dogrulugu; ~0 olmasi beklenir)
  5. Ortalama ve standart sapmayi (orneklem, ddof=1) raporlar, CSV dosyasina yazar.

Kullanim (simulasyon calisirken):
  ros2 run aslikol_control fk_eval
  ros2 run aslikol_control fk_eval --ros-args -p settle_time:=2.0 -p samples:=30
"""
from __future__ import annotations

import csv
import os
import time
from datetime import datetime

import numpy as np
import rclpy

from aslikol_control.arm_commander import HOME, ArmCommander
from aslikol_control.kinematics import JOINT_NAMES, ee_position

# Test konfigurasyonlari (radyan). Hepsi eklem limitleri icinde ve kol zeminin ustunde kalir.
CONFIGURATIONS = (
    (0.0, 0.0, 0.0, 0.0),
    (0.5, 0.4, 0.6, 0.3),
    (-0.8, 0.7, 0.9, 0.5),
    (1.6, -0.5, -0.8, -0.6),
    (2.4, 0.9, 0.4, 1.1),
    (-2.1, 0.3, 1.4, -0.9),
    (0.9, 1.2, -0.6, 0.8),
    (-1.3, -0.9, -0.7, 1.2),
    (2.9, 0.6, 1.0, -1.4),
    (-0.3, 1.0, 1.1, 0.9),
)


class FkEvaluator(ArmCommander):

    def __init__(self):
        super().__init__('fk_eval')
        self.settle_time = float(self.declare_parameter('settle_time', 1.5).value)
        self.samples = int(self.declare_parameter('samples', 20).value)
        default_dir = os.path.join(os.path.expanduser('~'), 'aslikol_results')
        self.output_dir = str(self.declare_parameter('output_dir', default_dir).value)
        if not (0.0 <= self.settle_time <= 30.0):
            raise ValueError('settle_time 0..30 s araliginda olmali')
        if not (1 <= self.samples <= 500):
            raise ValueError('samples 1..500 araliginda olmali')

    def sample_gz_position(self) -> np.ndarray | None:
        """Gazebo'dan gelen son 'samples' adet uc efektor konumunun ortalamasi."""
        values = []
        last = None
        deadline = time.monotonic() + 30.0
        while len(values) < self.samples and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            msg = self.ee_pose_gz
            if msg is not None and msg is not last:
                p = msg.pose.position
                values.append((p.x, p.y, p.z))
                last = msg
        if not values:
            return None
        return np.mean(np.asarray(values), axis=0)

    def run(self) -> int:
        if not self.wait_for_system():
            return 1

        rows = []
        for k, q_cmd in enumerate(CONFIGURATIONS, start=1):
            self.get_logger().info(f'[{k}/{len(CONFIGURATIONS)}] hedef q = {q_cmd}')
            if not self.move_to(q_cmd):
                self.get_logger().error('Hareket basarisiz, analiz durduruluyor')
                return 1
            self.wait_sim(self.settle_time)

            p_gz = self.sample_gz_position()
            q_meas = self.measured_joints()
            if p_gz is None or q_meas is None:
                self.get_logger().error('/aslikol/ee_pose veya /joint_states gelmiyor')
                return 1

            p_cmd = ee_position(q_cmd, self.geom)
            p_meas = ee_position(q_meas, self.geom)
            e_cmd = float(np.linalg.norm(p_cmd - p_gz))
            e_meas = float(np.linalg.norm(p_meas - p_gz))
            rows.append({
                'config': k,
                **{f'q{i + 1}_cmd': q_cmd[i] for i in range(len(JOINT_NAMES))},
                **{f'q{i + 1}_meas': float(q_meas[i]) for i in range(len(JOINT_NAMES))},
                'x_fk': p_cmd[0], 'y_fk': p_cmd[1], 'z_fk': p_cmd[2],
                'x_gz': p_gz[0], 'y_gz': p_gz[1], 'z_gz': p_gz[2],
                'err_cmd_m': e_cmd, 'err_meas_m': e_meas,
            })
            self.get_logger().info(
                f'    FK=({p_cmd[0]:.4f}, {p_cmd[1]:.4f}, {p_cmd[2]:.4f})  '
                f'Gazebo=({p_gz[0]:.4f}, {p_gz[1]:.4f}, {p_gz[2]:.4f})  '
                f'hata={e_cmd * 1000:.3f} mm')

        self.get_logger().info('Baslangic konumuna donuluyor')
        self.move_to(HOME)

        self.report(rows)
        return 0

    def report(self, rows):
        e_cmd = np.array([r['err_cmd_m'] for r in rows])
        e_meas = np.array([r['err_meas_m'] for r in rows])
        ddof = 1 if len(rows) > 1 else 0

        lines = [
            '',
            '| # | q1 | q2 | q3 | q4 | FK (x, y, z) [m] | Gazebo (x, y, z) [m] | Hata [mm] |',
            '|---|----|----|----|----|------------------|----------------------|-----------|',
        ]
        for r in rows:
            lines.append(
                f"| {r['config']} | {r['q1_cmd']:.2f} | {r['q2_cmd']:.2f} | {r['q3_cmd']:.2f} "
                f"| {r['q4_cmd']:.2f} | ({r['x_fk']:.4f}, {r['y_fk']:.4f}, {r['z_fk']:.4f}) "
                f"| ({r['x_gz']:.4f}, {r['y_gz']:.4f}, {r['z_gz']:.4f}) "
                f"| {r['err_cmd_m'] * 1000:.3f} |")
        lines += [
            '',
            f'Komut edilen acilarla FK - Gazebo: ortalama = {e_cmd.mean() * 1000:.4f} mm, '
            f'std = {e_cmd.std(ddof=ddof) * 1000:.4f} mm',
            f'Olculen acilarla FK - Gazebo     : ortalama = {e_meas.mean() * 1000:.4f} mm, '
            f'std = {e_meas.std(ddof=ddof) * 1000:.4f} mm',
            '(std: orneklem standart sapmasi, ddof=1)',
        ]
        print('\n'.join(lines), flush=True)

        os.makedirs(self.output_dir, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        path = os.path.join(self.output_dir, f'fk_eval_{stamp}.csv')
        with open(path, 'w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)
        self.get_logger().info(f'Sonuclar kaydedildi: {path}')


def main(args=None):
    rclpy.init(args=args)
    node = FkEvaluator()
    code = 1
    try:
        code = node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(code)


if __name__ == '__main__':
    main()
