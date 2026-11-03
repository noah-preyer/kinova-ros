#!/usr/bin/env python3
"""Hardware bring-up helper: move one arm joint relative to its current position.

Reads /joint_states, sends a FollowJointTrajectory goal to /arm_controller that
moves --joint by --delta rad over --duration s (other joints hold), and prints
the peak joint velocity seen on /joint_states against delta/duration.

    python3 docs/arm_goal.py --joint 6 --delta 0.35 --duration 2
"""
import argparse
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

p = argparse.ArgumentParser()
p.add_argument('--joint', type=int, default=6)
p.add_argument('--delta', type=float, default=0.35)
p.add_argument('--duration', type=float, default=2.0)
a = p.parse_args()
names = [f'j2n6s200_joint_{i}' for i in range(1, 7)]

rclpy.init()
n = Node('arm_goal')
state = {}
peak = [0.0]


def cb(m):
    d = dict(zip(m.name, zip(m.position, m.velocity)))
    state.update(d)
    v = d.get(names[a.joint - 1], (0, 0))[1]
    if abs(v) > abs(peak[0]):
        peak[0] = v


n.create_subscription(JointState, '/joint_states', cb, 10)
t0 = time.time()
while not all(j in state for j in names) and time.time() - t0 < 10:
    rclpy.spin_once(n, timeout_sec=0.1)
cur = [state[j][0] for j in names]
tgt = list(cur)
tgt[a.joint - 1] += a.delta
cli = ActionClient(n, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
cli.wait_for_server(timeout_sec=10)
g = FollowJointTrajectory.Goal()
g.trajectory.joint_names = names
g.trajectory.points = [JointTrajectoryPoint(
    positions=tgt, time_from_start=Duration(sec=int(a.duration), nanosec=int((a.duration % 1) * 1e9)))]
fut = cli.send_goal_async(g)
rclpy.spin_until_future_complete(n, fut)
res = fut.result().get_result_async()
while not res.done():
    rclpy.spin_once(n, timeout_sec=0.05)
print('result error_code:', res.result().result.error_code)
print(f'peak velocity {peak[0]:.4f} rad/s, expected about {a.delta / a.duration:.4f} rad/s (average)')
