"""Run against a launched ROS system; assert observable behavior and log evidence."""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from visualization_msgs.msg import MarkerArray
from a2_interfaces.msg import ArmState, MachineState, TaskStatus, WorkpieceState
from .core import HOME


class Probe(Node):
    def __init__(self):
        super().__init__('a2_acceptance')
        self.status = self.arm = self.machine = self.piece = None
        self.phases = set()
        self.scene_time = 0.0
        self.create_subscription(TaskStatus, '/a2/task/status', self.status_cb, 10)
        self.create_subscription(ArmState, '/a2/arm/state', lambda msg: setattr(self, 'arm', msg), 10)
        self.create_subscription(MachineState, '/a2/machine/state', lambda msg: setattr(self, 'machine', msg), 10)
        self.create_subscription(WorkpieceState, '/a2/workpiece', lambda msg: setattr(self, 'piece', msg), 10)
        self.create_subscription(MarkerArray, '/a2/scene', self.scene_cb, 10)
        self.task_clients = {name: self.create_client(Trigger, '/a2/task/'+name) for name in ('start', 'cancel', 'reset')}

    def status_cb(self, msg):
        self.status = msg
        self.phases.add(msg.phase)

    def scene_cb(self, msg):
        if msg.markers:
            self.scene_time = time.monotonic()

    def until(self, condition, timeout=60.0):
        deadline = time.monotonic()+timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if condition():
                return
        state = self.status.phase if self.status else 'no task data'
        raise AssertionError(f'Condition timed out; latest task state: {state}')

    def call(self, name):
        client = self.task_clients[name]
        if not client.wait_for_service(timeout_sec=5.0):
            raise AssertionError(f'Service unavailable: {name}')
        future = client.call_async(Trigger.Request())
        self.until(future.done, 5.0)
        response = future.result()
        return response

    def run(self, scenario, output):
        self.until(lambda: self.status and self.arm and self.machine and self.piece and self.scene_time, 15.0)
        deadline = time.monotonic()+10.0
        while True:
            response = self.call('start')
            if response.success:
                break
            assert 'not ready' in response.message or 'stale' in response.message, response.message
            assert time.monotonic() < deadline, response.message
            rclpy.spin_once(self, timeout_sec=0.1)
        self.until(lambda: self.status.active, 5.0)
        run_id = self.status.run_id
        assert not self.call('start').success, 'Duplicate start must be rejected'
        assert not self.call('reset').success, 'Reset during execution must be rejected'
        if scenario == 'cancel':
            self.until(lambda: self.status.phase == 'TRANSFER_LOAD')
            assert self.call('cancel').success
        self.until(lambda: self.status.phase in ('COMPLETE', 'FAILED', 'CANCELED'), 75.0)
        expected = {'normal': 'COMPLETE', 'busy': 'COMPLETE', 'timeout': 'FAILED',
                    'station_timeout': 'FAILED', 'cancel': 'CANCELED'}[scenario]
        assert self.status.phase == expected, (self.status.phase, self.status.error)
        assert time.monotonic()-self.scene_time < 1.0, 'Scene node stopped publishing'
        evidence = json.loads((output/f'{run_id}.json').read_text(encoding='utf-8'))
        assert evidence['phase'] == expected
        assert (output/f'{run_id}.csv').stat().st_size > 100
        if expected == 'COMPLETE':
            self.until(lambda: self.piece.location == 'TRAY' and self.piece.processed
                       and not self.arm.busy and not self.arm.gripper_closed, 3.0)
            assert self.status.completed == 1 and self.status.success
            assert math.dist((self.arm.tcp.x, self.arm.tcp.y, self.arm.tcp.z), HOME) < 0.005
            assert {'PICK_GRIP', 'LOAD_RELEASE', 'WAIT_PROCESS', 'UNLOAD_GRIP',
                    'RETURN_RELEASE'}.issubset(self.phases), self.phases
        else:
            assert self.status.completed == 0 and not self.status.success
            if scenario in ('timeout', 'station_timeout'):
                assert 'timeout' in self.status.error
        if scenario == 'busy':
            assert 'WAIT_READY' in self.phases
        self.until(lambda: not self.arm.busy, 5.0)
        # Let outstanding action results and abort replies finish before resetting.
        deadline = time.monotonic()+5.0
        while True:
            reset = self.call('reset')
            if reset.success:
                break
            if time.monotonic() >= deadline:
                raise AssertionError(reset.message)
            rclpy.spin_once(self, timeout_sec=0.1)
        self.until(lambda: self.status.phase == 'IDLE' and self.piece.location == 'TRAY'
                   and not self.piece.processed and not self.arm.gripper_closed, 5.0)
        print(f'PASS {scenario}: {run_id}; terminal={expected}; reset=IDLE')


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', choices=['normal', 'busy', 'timeout', 'station_timeout', 'cancel'], default='normal')
    parser.add_argument('--output-dir', default='results')
    options, ros_args = parser.parse_known_args(args)
    rclpy.init(args=ros_args)
    node = Probe()
    try:
        node.run(options.scenario, Path(options.output_dir).resolve())
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
