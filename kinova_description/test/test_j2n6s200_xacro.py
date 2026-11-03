import os
import re
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory

SHARE = get_package_share_directory('kinova_description')
XACRO = os.path.join(SHARE, 'urdf', 'j2n6s200_standalone.xacro')
P = 'j2n6s200'
JOINTS = ['%s_joint_%d' % (P, i) for i in range(1, 7)] + [
    P + '_joint_finger_1', P + '_joint_finger_2',
    P + '_joint_finger_tip_1', P + '_joint_finger_tip_2']


def render(*args):
    out = subprocess.run(['xacro', XACRO, *args], check=True,
                         capture_output=True, text=True).stdout
    return out


def block(*args):
    root = ET.fromstring(render(*args))
    blocks = root.findall('ros2_control')
    assert len(blocks) == 1
    return blocks[0]


def ifaces(joint, tag):
    return [i.get('name') for i in joint.findall(tag)]


def test_ros2_control_block_matches_contract():
    b = block()
    assert b.get('name') == 'j2n6s200_system'
    assert b.get('type') == 'system'
    assert b.find('hardware/plugin').text.strip() == 'kinova_driver/KinovaSystem'
    assert [j.get('name') for j in b.findall('joint')] == JOINTS


def test_interfaces_per_joint():
    joints = {j.get('name'): j for j in block().findall('joint')}
    for i in range(1, 7):
        j = joints['%s_joint_%d' % (P, i)]
        assert ifaces(j, 'command_interface') == ['velocity']
        assert ifaces(j, 'state_interface') == ['position', 'velocity', 'effort']
    f1 = joints[P + '_joint_finger_1']
    assert ifaces(f1, 'command_interface') == ['position']
    assert ifaces(f1, 'state_interface') == ['position', 'velocity']
    f2 = joints[P + '_joint_finger_2']
    assert ifaces(f2, 'command_interface') == []
    assert ifaces(f2, 'state_interface') == ['position', 'velocity']
    for n in (1, 2):
        t = joints['%s_joint_finger_tip_%d' % (P, n)]
        assert ifaces(t, 'command_interface') == []
        assert ifaces(t, 'state_interface') == ['position']


def _limits(joint):
    params = {p.get('name'): float(p.text) for p in
              joint.find('command_interface').findall('param')}
    return params['min'], params['max']


def test_velocity_limits():
    joints = {j.get('name'): j for j in block().findall('joint')}
    lo, hi = _limits(joints[P + '_joint_1'])
    assert abs(hi - 0.6283) < 1e-3 and lo == -hi
    lo, hi = _limits(joints[P + '_joint_4'])
    assert abs(hi - 0.8378) < 1e-3 and lo == -hi
    lo, hi = _limits(joints[P + '_joint_finger_1'])
    assert lo == 0.0 and abs(hi - 1.3963) < 1e-9


def hw(*args):
    return {p.get('name'): p.text.strip()
            for p in block(*args).findall('hardware/param')}


def test_fake_flag_passthrough():
    assert hw()['use_fake_device'] == 'true'
    assert hw('use_fake_hardware:=false')['use_fake_device'] == 'false'
    p = hw('serial_number:=PJ123')
    assert p['serial_number'] == 'PJ123'
    assert p['robot_type'] == 'j2n6s200'
    assert p['connection_type'] == 'USB'
    assert p['finger_max_turn'] == '6800'


def test_meshes_resolve():
    names = re.findall(r'package://kinova_description/meshes/([^"]+\.dae)', render())
    assert names
    for n in set(names):
        assert os.path.isfile(os.path.join(SHARE, 'meshes', n)), n
