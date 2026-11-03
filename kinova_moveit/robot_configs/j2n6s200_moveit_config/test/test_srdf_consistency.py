"""Static checks: the j2n6s200 SRDF agrees with the URDF rendered from kinova_description."""

import os
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest

PREFIX = 'j2n6s200'
FINGER_JOINTS = {f'{PREFIX}_joint_finger_1', f'{PREFIX}_joint_finger_2'}
ARM_JOINTS = {f'{PREFIX}_joint_{i}' for i in range(1, 7)}


@pytest.fixture(scope='module')
def urdf():
    xacro_file = os.path.join(
        get_package_share_directory('kinova_description'), 'urdf', f'{PREFIX}_standalone.xacro')
    out = subprocess.run(
        ['xacro', xacro_file, 'use_fake_hardware:=true'],
        check=True, capture_output=True, text=True).stdout
    return ET.fromstring(out)


@pytest.fixture(scope='module')
def srdf():
    path = os.path.join(
        get_package_share_directory(f'{PREFIX}_moveit_config'), 'config', f'{PREFIX}.srdf')
    return ET.parse(path).getroot()


def urdf_links(urdf):
    return {e.get('name') for e in urdf.findall('link')}


def urdf_joints(urdf):
    return {e.get('name'): e for e in urdf.findall('joint')}


def chain_joints(urdf, base, tip):
    """Movable joints on the URDF path base -> tip."""
    by_child = {j.find('child').get('link'): j for j in urdf.findall('joint')}
    joints = []
    link = tip
    while link != base:
        j = by_child[link]
        joints.append(j)
        link = j.find('parent').get('link')
    return {j.get('name') for j in joints if j.get('type') not in ('fixed', 'floating')}


def group_joints(urdf, group):
    if group.findall('link'):
        raise AssertionError('groups are not expected to list links')
    joints = {j.get('name') for j in group.findall('joint')}
    for c in group.findall('chain'):
        joints |= chain_joints(urdf, c.get('base_link'), c.get('tip_link'))
    return joints


def test_every_srdf_link_exists_in_urdf(urdf, srdf):
    links = urdf_links(urdf)
    referenced = set()
    for e in srdf.findall('disable_collisions'):
        referenced |= {e.get('link1'), e.get('link2')}
    for e in srdf.findall('end_effector'):
        referenced.add(e.get('parent_link'))
    for e in srdf.iter('chain'):
        referenced |= {e.get('base_link'), e.get('tip_link')}
    for e in srdf.iter('link'):
        referenced.add(e.get('name'))
    missing = referenced - links
    assert not missing, f'SRDF links missing from URDF: {sorted(missing)}'
    joints = set(urdf_joints(urdf))
    for e in srdf.iter('joint'):
        assert e.get('name') in joints, e.get('name')


def test_collision_pair_count(srdf):
    assert len(srdf.findall('disable_collisions')) == 56


def test_gripper_group_is_two_finger_joints(srdf):
    groups = {g.get('name'): g for g in srdf.findall('group')}
    assert set(groups) == {'arm', 'gripper'}
    gripper = groups['gripper']
    assert not gripper.findall('link')
    assert not gripper.findall('chain')
    assert {j.get('name') for j in gripper.findall('joint')} == FINGER_JOINTS


def test_named_states_cover_group_joints(urdf, srdf):
    groups = {g.get('name'): group_joints(urdf, g) for g in srdf.findall('group')}
    assert groups['arm'] == ARM_JOINTS
    states = {}
    for s in srdf.findall('group_state'):
        names = [j.get('name') for j in s.findall('joint')]
        assert len(names) == len(set(names)), s.get('name')
        assert set(names) == groups[s.get('group')], s.get('name')
        states[s.get('name')] = s.get('group')
    assert states == {'Home': 'arm', 'Retract': 'arm', 'Vertical': 'arm',
                      'Open': 'gripper', 'Close': 'gripper'}


def test_no_virtual_joint(srdf):
    assert not srdf.findall('virtual_joint')
