"""Duz kinematik birim testleri (ROS calistirmadan: colcon test veya pytest)."""
import math
import os

import numpy as np
import pytest

from ra4_control.kinematics import (ee_position, ee_position_closed_form,
                                        forward_kinematics, load_geometry,
                                        matrix_to_quaternion, quaternion_to_matrix,
                                        validate_joints)

GEOMETRY = os.path.join(os.path.dirname(__file__), '..', '..', 'ra4_description',
                        'config', 'geometry.yaml')


@pytest.fixture(scope='module')
def geom():
    return load_geometry(GEOMETRY)


def test_home_is_straight_up(geom):
    p = ee_position((0.0, 0.0, 0.0, 0.0), geom)
    expected_z = geom.h0 + geom.h1 + geom.l2 + geom.l3 + geom.l4
    assert np.allclose(p, (0.0, 0.0, expected_z))


def test_shoulder_horizontal(geom):
    # Omuz +90 derece: kolun tamami +x yonunde yatay
    p = ee_position((0.0, math.pi / 2, 0.0, 0.0), geom)
    assert np.allclose(p, (geom.l2 + geom.l3 + geom.l4, 0.0, geom.h0 + geom.h1))


def test_base_yaw_rotates_about_z(geom):
    q = (0.0, 0.6, 0.4, 0.2)
    p0 = ee_position(q, geom)
    p90 = ee_position((math.pi / 2, *q[1:]), geom)
    assert np.allclose(p90, (-p0[1], p0[0], p0[2]))


def test_matrix_matches_closed_form(geom):
    rng = np.random.default_rng(42)
    for _ in range(500):
        q = rng.uniform(geom.lower, geom.upper)
        assert np.allclose(ee_position(q, geom), ee_position_closed_form(q, geom), atol=1e-12)


def test_rotation_is_orthonormal(geom):
    T = forward_kinematics((0.3, -0.7, 1.1, 0.4), geom)
    R = T[:3, :3]
    assert np.allclose(R @ R.T, np.eye(3), atol=1e-12)
    assert math.isclose(np.linalg.det(R), 1.0, abs_tol=1e-12)


def test_quaternion_roundtrip(geom):
    R = forward_kinematics((1.0, 0.5, -0.8, 1.2), geom)[:3, :3]
    assert np.allclose(quaternion_to_matrix(*matrix_to_quaternion(R)), R, atol=1e-12)


@pytest.mark.parametrize('q', [
    (0.0, 0.0, 0.0),                    # eksik eleman
    (0.0, 0.0, 0.0, 0.0, 0.0),          # fazla eleman
    (0.0, 5.0, 0.0, 0.0),               # limit disi
    (0.0, float('nan'), 0.0, 0.0),      # gecersiz sayi
])
def test_invalid_joints_rejected(geom, q):
    with pytest.raises(ValueError):
        validate_joints(q, geom)
