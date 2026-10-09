#!/usr/bin/env python3
"""Run optional wall preparation, stop its process, then run angular-only Task3.

Preparation delegates to distance_controller's existing start_paused interface.
No route-resume command is issued. This coordinator owns only its child processes.
"""

import argparse
import math
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from distance_controller.srv import ExecuteStep
from geometry_msgs.msg import Twist
import rclpy
from rclpy.signals import SignalHandlerOptions


def executable(package):
    """Resolve a controller through the sourced ament index.

    @param[in] package Name from preparation_command() or main(); selects the install prefix.
    @return Absolute executable path consumed by subprocess.Popen(); no files are modified.
    """
    return str(Path(get_package_prefix(package)) / 'lib' / package / package)


def options():
    """Read CLI settings; reject nonfinite distances and nonpositive time budgets."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scan-topic', default='/scan')
    parser.add_argument('--rear-distance', type=float, default=0.28)
    parser.add_argument('--timeout', type=float, default=75.0)
    parser.add_argument('--prepare-only', action='store_true')
    args = parser.parse_args()
    if not args.scan_topic or not math.isfinite(args.rear_distance) or args.rear_distance <= 0:
        parser.error('scan topic and positive finite rear distance required')
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('timeout must be finite and positive')
    return args


def request(node, client, action):
    """Send a bounded service request to the owned preparation node.

    @param[in] node Coordinator from main(), used to spin until the reply arrives.
    @param[in] client Service client from main(), targeting task3_preparation/step.
    @param[in] action status or finish from wait_prepared(); never requests route motion.
    @return Service reply, or None when unavailable/timed out; caller checks accepted.
    """
    if not client.wait_for_service(timeout_sec=0.2):
        return None
    message = ExecuteStep.Request()
    message.action = action
    future = client.call_async(message)
    rclpy.spin_until_future_complete(node, future, timeout_sec=0.5)
    if not future.done():
        client.remove_pending_request(future)
        return None
    return future.result()


def wait_prepared(node, client, process, timeout):
    """Finish preparation only after readiness and wait for exclusive handoff.

    @param[in] node Coordinator from main(), used for service spinning and stage logging.
    @param[in] client Preparation service client from main(), used by request().
    @param[in,out] process Child handle from main(); read exit state, then reap successful exit.
    @param[in] timeout CLI budget from main(), seconds on the steady wall clock.
    @return None only after accepted finish and clean exit; raises on fault or timeout.
    @note main() must not start the turn child until this function returns successfully.
    """
    deadline = time.monotonic() + timeout
    while rclpy.ok() and time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(f'Preparation exited prematurely: {process.returncode}')
        reply = request(node, client, 'status')
        if reply and reply.accepted:
            if 'state=FAULT' in reply.message:
                raise RuntimeError(reply.message)
            if 'state=WAITING ' in reply.message:
                finish = request(node, client, 'finish')
                if finish and finish.accepted:
                    node.get_logger().info('Preparation accepted finish: ' + reply.message)
                    if process.wait(timeout=5) != 0:
                        raise RuntimeError('Preparation did not exit successfully')
                    return
        time.sleep(0.1)
    raise RuntimeError('Preparation deadline expired; turn controller will not start')


def terminate(process):
    """Terminate and reap only the coordinator-owned child during cleanup.

    @param[in,out] process Handle retained by main(); signals and wait update child exit state.
    @note Escalates SIGINT to terminate/kill with bounded waits; never targets unrelated nodes.
    """
    if process is None or process.poll() is not None:
        return
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=3)
    except subprocess.TimeoutExpired:
        process.terminate()
        try:
            process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=2)


def preparation_command(args):
    """Compose the command for simulation-only wall preparation.

    @param[in] args Parsed options from main(); reads scan_topic and rear_distance.
    @return Argument list passed to Popen() by main(); does not launch a process here.
    @note Manual start_paused prevents executing the configured AB preview.
    """
    return [
        executable('distance_controller'),
        '2',
        '--ros-args',
        '-r',
        '__node:=task3_preparation',
        '-p',
        'use_sim_time:=true',
        '-p',
        'manual_mode:=true',
        '-p',
        'start_paused:=true',
        '-p',
        'route:=[AB]',
        '-p',
        'scan_topic:=' + args.scan_topic,
        '-p',
        'wall_heading_half_angle:=' + str(math.radians(20.0)),
        '-p',
        'rear_target_distance:=' + str(args.rear_distance),
    ]


def interrupt(_signal, _frame):
    """Convert termination into the same owned-child cleanup path as Ctrl+C."""
    raise KeyboardInterrupt


def main():
    """Own sequential process lifecycle; failure never starts the following turn stage."""
    args = options()
    signal.signal(signal.SIGTERM, interrupt)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('task3_sequence')
    client = node.create_client(ExecuteStep, '/task3_preparation/step')
    stop = node.create_publisher(Twist, '/cmd_vel', 10)
    child = None
    try:
        rclpy.spin_once(node, timeout_sec=0.5)
        if node.count_publishers('/cmd_vel') > 1:
            raise RuntimeError('Another /cmd_vel publisher exists; stop it before preparation')
        child = subprocess.Popen(preparation_command(args))
        wait_prepared(node, client, child, args.timeout)
        if args.prepare_only:
            node.get_logger().info('Preparation complete; no turn requested')
            return 0
        node.get_logger().info('Preparation process exited; starting angular-only turn controller')
        child = subprocess.Popen([executable('turn_controller')])
        return child.wait(timeout=145)
    except (RuntimeError, subprocess.TimeoutExpired) as error:
        node.get_logger().error(str(error))
        return 2
    except KeyboardInterrupt:
        return 130
    finally:
        terminate(child)
        if child is not None and rclpy.ok():
            for _ in range(5):
                stop.publish(Twist())
                rclpy.spin_once(node, timeout_sec=0.02)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
