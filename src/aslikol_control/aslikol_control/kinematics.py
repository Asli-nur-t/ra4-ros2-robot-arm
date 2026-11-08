"""aslikol robot kolunun duz kinematigi (forward kinematics).

Geometrik sinif: R ⊢ R ∥ R ∥ R

Frame yerlesimi (her frame bir onceki frame'e gore yalnizca z boyunca otelenir,
ardindan eklem ekseni etrafinda doner; robot q = 0 iken dik yukari bakar):

    0 (base_link)   : zemin, z yukari
    1 (turret_link) : 0T1  = Trans(0, 0, h0) * Rot_z(q1)
    2 (upper_arm)   : 1T2  = Trans(0, 0, h1) * Rot_y(q2)
    3 (forearm)     : 2T3  = Trans(0, 0, l2) * Rot_y(q3)
    4 (wrist)       : 3T4  = Trans(0, 0, l3) * Rot_y(q4)
    5 (tool0)       : 4T5  = Trans(0, 0, l4)

    0T5 = 0T1 * 1T2 * 2T3 * 3T4 * 4T5

Kapali form (phi2 = q2, phi3 = q2 + q3, phi4 = q2 + q3 + q4):

    r = l2*sin(phi2) + l3*sin(phi3) + l4*sin(phi4)
    x = cos(q1) * r
    y = sin(q1) * r
    z = h0 + h1 + l2*cos(phi2) + l3*cos(phi3) + l4*cos(phi4)

Olculer aslikol_description/config/geometry.yaml dosyasindan okunur (URDF ile ayni kaynak).
"""
from __future__ import annotations

import argparse
import math
import os
from dataclasses import dataclass
from typing import Sequence

import numpy as np
import yaml

JOINT_NAMES = ('joint1', 'joint2', 'joint3', 'joint4')
NUM_JOINTS = len(JOINT_NAMES)


@dataclass(frozen=True)
class Geometry:
    h0: float  # zemin -> joint1
    h1: float  # joint1 -> joint2
    l2: float  # joint2 -> joint3
    l3: float  # joint3 -> joint4
    l4: float  # joint4 -> tool0
    lower: tuple
    upper: tuple


def default_geometry_path() -> str:
    """geometry.yaml dosyasinin kurulu paket icindeki yolunu dondurur."""
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('aslikol_description'),
                        'config', 'geometry.yaml')


def load_geometry(path: str | None = None) -> Geometry:
    path = path or default_geometry_path()
    with open(path, 'r', encoding='utf-8') as f:
        g = yaml.safe_load(f)
    try:
        joints = g['joints']
        return Geometry(
            h0=float(g['base']['height']),
            h1=float(g['turret']['height']),
            l2=float(g['upper_arm']['length']),
            l3=float(g['forearm']['length']),
            l4=float(g['wrist']['length']),
            lower=tuple(float(joints[n]['lower']) for n in JOINT_NAMES),
            upper=tuple(float(joints[n]['upper']) for n in JOINT_NAMES),
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f'geometry.yaml beklenen yapida degil: {exc}') from exc


# ---------------------------------------------------------------- temel matrisler
def trans(x: float, y: float, z: float) -> np.ndarray:
    T = np.eye(4)
    T[:3, 3] = (x, y, z)
    return T


def rot_z(q: float) -> np.ndarray:
    c, s = math.cos(q), math.sin(q)
    T = np.eye(4)
    T[:2, :2] = ((c, -s), (s, c))
    return T


def rot_y(q: float) -> np.ndarray:
    c, s = math.cos(q), math.sin(q)
    T = np.eye(4)
    T[0, 0], T[0, 2] = c, s
    T[2, 0], T[2, 2] = -s, c
    return T


# ---------------------------------------------------------------- dogrulama
def validate_joints(q: Sequence[float], geom: Geometry) -> np.ndarray:
    """Eklem vektorunu kontrol eder: uzunluk, sonlu deger ve eklem limitleri."""
    q = np.asarray(q, dtype=float).reshape(-1)
    if q.size != NUM_JOINTS:
        raise ValueError(f'{NUM_JOINTS} eklem degeri bekleniyordu, {q.size} geldi')
    if not np.all(np.isfinite(q)):
        raise ValueError('Eklem degerleri sonlu sayi olmali (NaN/inf olamaz)')
    for i, name in enumerate(JOINT_NAMES):
        if not geom.lower[i] <= q[i] <= geom.upper[i]:
            raise ValueError(
                f'{name}={q[i]:.4f} rad limit disinda [{geom.lower[i]}, {geom.upper[i]}]')
    return q


# ---------------------------------------------------------------- duz kinematik
def link_transforms(q: Sequence[float], geom: Geometry) -> list[np.ndarray]:
    """[0T1, 1T2, 2T3, 3T4, 4T5] ardisik donusum matrislerini dondurur."""
    q1, q2, q3, q4 = (float(v) for v in q)
    return [
        trans(0.0, 0.0, geom.h0) @ rot_z(q1),
        trans(0.0, 0.0, geom.h1) @ rot_y(q2),
        trans(0.0, 0.0, geom.l2) @ rot_y(q3),
        trans(0.0, 0.0, geom.l3) @ rot_y(q4),
        trans(0.0, 0.0, geom.l4),
    ]


def forward_kinematics(q: Sequence[float], geom: Geometry) -> np.ndarray:
    """0T5: uc efektorun (tool0) tabana gore homojen donusum matrisi."""
    T = np.eye(4)
    for Ti in link_transforms(q, geom):
        T = T @ Ti
    return T


def ee_position(q: Sequence[float], geom: Geometry) -> np.ndarray:
    return forward_kinematics(q, geom)[:3, 3].copy()


def ee_position_closed_form(q: Sequence[float], geom: Geometry) -> np.ndarray:
    """Ayni konumun kapali form ifadesi (matris carpimini dogrulamak icin)."""
    q1, q2, q3, q4 = (float(v) for v in q)
    p2, p3, p4 = q2, q2 + q3, q2 + q3 + q4
    r = geom.l2 * math.sin(p2) + geom.l3 * math.sin(p3) + geom.l4 * math.sin(p4)
    z = geom.h0 + geom.h1 + geom.l2 * math.cos(p2) + geom.l3 * math.cos(p3) \
        + geom.l4 * math.cos(p4)
    return np.array([math.cos(q1) * r, math.sin(q1) * r, z])


def joint_positions(q: Sequence[float], geom: Geometry) -> list[np.ndarray]:
    """Her frame orijininin tabana gore konumu [p1, p2, p3, p4, p5]."""
    T = np.eye(4)
    out = []
    for Ti in link_transforms(q, geom):
        T = T @ Ti
        out.append(T[:3, 3].copy())
    return out


# ---------------------------------------------------------------- yardimci
def matrix_to_quaternion(R: np.ndarray) -> tuple[float, float, float, float]:
    """3x3 rotasyon matrisini (x, y, z, w) quaternion'a cevirir."""
    m = np.asarray(R, dtype=float)[:3, :3]
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    return (x, y, z, w)


def quaternion_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def pose_to_matrix(position, orientation) -> np.ndarray:
    """(x, y, z) ve (qx, qy, qz, qw) -> 4x4 homojen matris."""
    T = np.eye(4)
    T[:3, :3] = quaternion_to_matrix(*orientation)
    T[:3, 3] = position
    return T


# ---------------------------------------------------------------- komut satiri
def _fmt(T: np.ndarray) -> str:
    return '\n'.join('  [' + '  '.join(f'{v: .5f}' for v in row) + ']' for row in T)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='aslikol duz kinematik hesaplayici (rapor icin ara matrisleri de yazar)')
    parser.add_argument('q', nargs=NUM_JOINTS, type=float, metavar='q',
                        help='joint1 joint2 joint3 joint4 (radyan)')
    parser.add_argument('--geometry', default=None, help='geometry.yaml yolu (istege bagli)')
    args, _ = parser.parse_known_args(argv)

    geom = load_geometry(args.geometry)
    q = validate_joints(args.q, geom)

    names = ['0T1', '1T2', '2T3', '3T4', '4T5']
    for name, Ti in zip(names, link_transforms(q, geom)):
        print(f'{name} =\n{_fmt(Ti)}\n')
    T = forward_kinematics(q, geom)
    print(f'0T5 =\n{_fmt(T)}\n')
    p = T[:3, 3]
    pc = ee_position_closed_form(q, geom)
    print(f'Uc efektor (matris)    : x={p[0]:.5f}  y={p[1]:.5f}  z={p[2]:.5f} m')
    print(f'Uc efektor (kapali form): x={pc[0]:.5f}  y={pc[1]:.5f}  z={pc[2]:.5f} m')


if __name__ == '__main__':
    main()
