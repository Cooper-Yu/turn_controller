"""Bounded isolated ROS node acceptance and fault injection; no hardware topics."""

import json
import math
import os
import re
from pathlib import Path
import signal
import subprocess
import time

os.environ['ROS_DOMAIN_ID'] = '174'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock

from ament_index_python.packages import get_package_prefix

ROOT = Path(os.environ.get('TURN_TEST_OUTPUT', '/tmp/turn_controller_tests'))
OUT = ROOT / f'task3_fixture_{time.time_ns()}'
OUT.mkdir(parents=True)
EXE = str(Path(get_package_prefix('turn_controller')) / 'lib/turn_controller/turn_controller')
rclpy.init()
results = []


def verify_waypoints(text):
    points = re.findall(r'Waypoint (\d+): x=([-\d.]+) y=([-\d.]+) yaw=([-\d.]+)', text)
    targets = re.findall(r'Turn \d+/\d+: start=[-\d.]+ delta=[-\d.]+ target=([-\d.]+)', text)
    assert len(points) == len(targets) == 2, text
    assert [p[3] for p in points] == targets, text
    assert abs(float(targets[-1]) - 3.10) < 1e-6, targets
    assert all(float(p[1]) == 0 and float(p[2]) == 0 for p in points)


def verify_result(case, exit_code, text, commands, yaw):
    assert commands and all(x == 0 and y == 0 for x, y, _ in commands)
    assert abs(commands[-1][2]) < 1e-9
    if case == 'single':
        assert 'Initialization complete' not in text, text
    if case in ('wrap', 'slow_clock'):
        assert exit_code == 0 and text.count('Reached turn ') == 2, text
        assert abs(yaw - 3.10) < 0.02, yaw
        verify_waypoints(text)
    else:
        assert exit_code == 2, text
        expected = {
            'loss': 'ODOM_TIMEOUT',
            'jump': 'ODOM_YAW_JUMP',
            'no_odom': 'INITIAL_ODOM_TIMEOUT',
            'stale': 'INITIAL_ODOM_TIMEOUT',
            'frame': 'ODOM_FRAME_CHANGED',
            'moving': 'INITIAL_STOP_TIMEOUT',
            'single': 'ODOM_TIMEOUT',
            'stalled': 'TURN_TIMEOUT',
            'clock_pause': 'ODOM_TIMEOUT',
        }[case]
        assert expected in text, text


def feedback(node, case, elapsed, yaw, wz):
    """Build valid feedback or one deliberate fault for the selected scenario."""
    m = Odometry()
    m.header.frame_id, m.child_frame_id = 'odom', 'base_link'
    if case == 'frame' and elapsed > 1.2:
        m.header.frame_id = 'changed'
    m.header.stamp = node.get_clock().now().to_msg()
    if case in ('slow_clock', 'clock_pause'):
        elapsed = min(elapsed, 1.0) if case == 'clock_pause' else elapsed
        m.header.stamp.sec = 100 + int(elapsed)
        m.header.stamp.nanosec = int(elapsed * 5) % 5 * 200000000
    if case == 'stale':
        m.header.stamp.sec -= 2
    angle = yaw + (0.8 if case == 'jump' and elapsed > 1.2 else 0)
    m.pose.pose.orientation.z, m.pose.pose.orientation.w = (
        math.sin(angle / 2),
        math.cos(angle / 2),
    )
    m.twist.twist.angular.z = 0.1 if case == 'moving' else wz
    return m


def run(case):
    node = rclpy.create_node('turn_test_' + case)
    pub = node.create_publisher(Odometry, '/' + case + '/odom', 10)
    clock_pub = node.create_publisher(Clock, '/clock', 10)
    commands = []
    node.create_subscription(
        Twist,
        '/' + case + '/cmd',
        lambda m: commands.append((m.linear.x, m.linear.y, m.angular.z)),
        10,
    )
    args = [
        EXE,
        '--ros-args',
        '-p',
        'use_sim_time:=false',
        '-p',
        'odom_topic:=/' + case + '/odom',
        '-p',
        'cmd_vel_topic:=/' + case + '/cmd',
        '-p',
        'turn_angles:=[0.3, -0.3]',
        '-p',
        'dwell_duration:=0.05',
        '-p',
        'settle_duration:=0.1',
        '-p',
        'startup_timeout:=2.0',
    ]
    if case in ('slow_clock', 'clock_pause'):
        args.extend(['-p', 'use_sim_time:=true'])
    if case == 'stalled':
        args.extend(['-p', 'segment_timeout:=1.5'])
    path = OUT / (case + '.log')
    log = path.open('w')
    proc = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT)
    yaw = 3.10
    start = previous = time.monotonic()
    sent_single = False
    try:
        while proc.poll() is None and time.monotonic() - start < 18:
            now = time.monotonic()
            dt = now - previous
            previous = now
            wz = commands[-1][2] if commands else 0.0
            if case != 'stalled':
                yaw += wz * dt
            elapsed = now - start
            send = case != 'no_odom' and not (case == 'loss' and elapsed > 1.2)
            if case == 'single':
                send = not sent_single and pub.get_subscription_count() > 0
                sent_single = sent_single or send
            if send:
                m = feedback(node, case, elapsed, yaw, wz)
                if case in ('slow_clock', 'clock_pause'):
                    clock_pub.publish(Clock(clock=m.header.stamp))
                pub.publish(m)
            rclpy.spin_once(node, timeout_sec=0.01)
            time.sleep(0.01)
        assert proc.poll() is not None, 'bounded execution failed'
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.02)
        text = path.read_text()
        verify_result(case, proc.returncode, text, commands, yaw)
        results.append({'case': case, 'passed': True, 'exit': proc.returncode})
    finally:
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            proc.wait(timeout=3)
        log.close()
        node.destroy_node()


try:
    for case in os.environ.get(
        'TURN_TEST_CASES',
        'wrap loss jump no_odom stale frame moving single stalled slow_clock clock_pause',
    ).split():
        run(case)
    invalid = subprocess.run(
        [EXE, '--ros-args', '-p', 'kp:=-1.0'], capture_output=True, text=True, timeout=5
    )
    assert invalid.returncode == 1 and 'must be finite and nonnegative' in invalid.stderr
    results.append({'case': 'invalid_gain', 'passed': True})
finally:
    rclpy.shutdown()
    (OUT / 'results.json').write_text(json.dumps(results, indent=2))
print(json.dumps({'output': str(OUT), 'results': results}))
