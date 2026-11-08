"""Hareket dizisi: baslangic -> A -> B -> C -> baslangic.

Kol, FollowJointTrajectory action'i ile sirayla uc konfigurasyona gider ve sonunda
baslangic konumuna (tum eklemler 0 rad) doner. Konfigurasyonlar parametreyle degistirilebilir.

Kullanim (simulasyon calisirken):
  ros2 run aslikol_control sequence
  ros2 run aslikol_control sequence --ros-args -p pause:=2.0 -p loops:=2
  ros2 run aslikol_control sequence --ros-args -p config_a:="[0.5, 0.6, 0.8, 0.7]"
  (Degerleri ondalikli yaz: 1 yerine 1.0, aksi halde parametre tipi tamsayi dizisi olur.)
"""
import rclpy

from aslikol_control.arm_commander import HOME, ArmCommander
from aslikol_control.kinematics import ee_position, validate_joints

DEFAULT_CONFIGS = {
    'config_a': [0.7, 0.8, 0.9, 0.9],
    'config_b': [-1.2, 0.5, 1.2, 1.0],
    'config_c': [2.0, -0.6, -1.0, -1.1],
}


class SequenceRunner(ArmCommander):

    def __init__(self):
        super().__init__('sequence')
        self.pause = float(self.declare_parameter('pause', 1.5).value)
        self.loops = int(self.declare_parameter('loops', 1).value)
        if not (0.0 <= self.pause <= 30.0):
            raise ValueError('pause 0..30 s araliginda olmali')
        if not (1 <= self.loops <= 20):
            raise ValueError('loops 1..20 araliginda olmali')

        self.targets = []
        for name, default in DEFAULT_CONFIGS.items():
            value = self.declare_parameter(name, default).value
            # Parametre yanlis girilirse (eksik/fazla eleman, limit disi) hareketten ONCE durur
            q = validate_joints(value, self.geom)
            self.targets.append((name[-1].upper(), q))

    def run(self) -> int:
        if not self.wait_for_system():
            return 1

        plan = []
        for _ in range(self.loops):
            plan += [('Baslangic', HOME)] + self.targets
        plan.append(('Baslangic', HOME))

        for label, q in plan:
            p = ee_position(q, self.geom)
            self.get_logger().info(
                f'-> {label}: q = [{", ".join(f"{v:.2f}" for v in q)}] rad, '
                f'beklenen uc efektor = ({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}) m')
            if not self.move_to(q):
                self.get_logger().error('Hareket basarisiz, dizi durduruldu')
                return 1
            self.wait_sim(self.pause)

        self.get_logger().info('Hareket dizisi tamamlandi')
        return 0


def main(args=None):
    rclpy.init(args=args)
    node = SequenceRunner()
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
