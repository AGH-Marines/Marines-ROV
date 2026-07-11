import math
import os
import xml.etree.ElementTree as ET
from dataclasses import dataclass

import numpy as np
from ament_index_python.packages import get_package_share_directory


@dataclass
class Thruster:
    index: int
    name: str
    position: np.ndarray
    thrust_dir: np.ndarray


def default_model_path() -> str:
    return os.path.join(
        get_package_share_directory('rov_simulation'), 'scenarios', 'rov_robot.scn')


def _rotation_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def load_thrusters(path: str = None) -> list:
    path = path or default_model_path()
    root = ET.parse(path).getroot()

    thrusters = []
    for actuator in root.iter('actuator'):
        if actuator.get('type') != 'thruster':
            continue
        origin = actuator.find('origin')
        x, y, z = (float(v) for v in origin.get('xyz').split())
        roll, pitch, yaw = (float(v) for v in origin.get('rpy').split())
        position = np.array([x, y, z], dtype=float)
        thrust_dir = _rotation_matrix(roll, pitch, yaw) @ np.array([1.0, 0.0, 0.0])
        thrusters.append(Thruster(
            index=len(thrusters), name=actuator.get('name'),
            position=position, thrust_dir=thrust_dir,
        ))
    return thrusters
