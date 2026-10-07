"""Acknowledged task state machine, safety interlocks and persistent run evidence."""
import csv
import json
import math
from pathlib import Path
import time
import uuid

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger
from a2_interfaces.action import ArmMove
from a2_interfaces.msg import ArmState, MachineState, TaskStatus, WorkpieceState
from a2_interfaces.srv import MachineCommand
from .core import Cycle, HOME


class TaskNode(Node):
    def __init__(self):
        super().__init__('task_manager')
        for name, value in [('motion_seconds', 1.4), ('station_timeout', 12.0),
                            ('process_timeout', 12.0), ('output_dir', 'results')]:
            self.declare_parameter(name, value)
        self.motion_seconds = float(self.get_parameter('motion_seconds').value)
        self.station_timeout = float(self.get_parameter('station_timeout').value)
        self.process_timeout = float(self.get_parameter('process_timeout').value)
        if not (0.1 <= self.motion_seconds <= 30.0 and
                all(math.isfinite(x) and x > 0 for x in (self.station_timeout, self.process_timeout))):
            raise ValueError('Invalid task timing parameters')
        self.output = Path(self.get_parameter('output_dir').value).expanduser().resolve()
        self.output.mkdir(parents=True, exist_ok=True)
        self.cycle = Cycle()
        self.arm = None
        self.machine = None
        self.arm_time = self.machine_time = 0.0
        self.run_id = ''
        self.begin = self.step_begin = 0.0
        self.end = None
        self.pending = None
        self.goal_handle = None
        self.goal_future = None
        self.abort_future = None
        self.reset_futures = None
        self.reset_begin = 0.0
        self.stream = None
        self.writer = None
        self.epoch = 0
        self.arm_client = ActionClient(self, ArmMove, '/a2/arm/move')
        self.machine_client = self.create_client(MachineCommand, '/a2/machine/command')
        self.arm_reset = self.create_client(Trigger, '/a2/arm/reset')
        self.status_pub = self.create_publisher(TaskStatus, '/a2/task/status', 10)
        self.piece_pub = self.create_publisher(WorkpieceState, '/a2/workpiece', 10)
        self.create_subscription(ArmState, '/a2/arm/state', self.arm_cb, 10)
        self.create_subscription(MachineState, '/a2/machine/state', self.machine_cb, 10)
        self.create_service(Trigger, '/a2/task/start', self.start)
        self.create_service(Trigger, '/a2/task/cancel', self.cancel)
        self.create_service(Trigger, '/a2/task/reset', self.reset)
        self.create_timer(0.05, self.tick)
        self.create_timer(0.2, self.publish)
        self.get_logger().info(f'Run evidence will be saved to {self.output}')

    def arm_cb(self, msg):
        self.arm, self.arm_time = msg, time.monotonic()

    def machine_cb(self, msg):
        self.machine, self.machine_time = msg, time.monotonic()

    def ready(self):
        now = time.monotonic()
        return (self.arm is not None and self.machine is not None
                and now-self.arm_time < 1.0 and now-self.machine_time < 1.0
                and self.arm_client.server_is_ready() and self.machine_client.service_is_ready())

    def start(self, _, response):
        if self.reset_futures is not None or self.cycle.phase != 'IDLE':
            response.message = 'Reset to IDLE before starting a new cycle'
            return response
        if not self.ready() or self.arm.busy:
            response.message = 'Arm/machine not ready or state data is stale'
            return response
        if self.arm.gripper_closed or math.dist((self.arm.tcp.x, self.arm.tcp.y, self.arm.tcp.z), HOME) > 0.015:
            response.message = 'Initial arm pose changed; reset the simulation first'
            return response
        self.begin = self.step_begin = time.monotonic()
        self.end = None
        self.epoch += 1
        self.run_id = time.strftime('%Y%m%d-%H%M%S') + '-' + uuid.uuid4().hex[:8]
        self.stream = (self.output / f'{self.run_id}.csv').open('w', encoding='utf-8', newline='')
        self.writer = csv.writer(self.stream)
        self.writer.writerow(['elapsed_s', 'phase', 'event', 'piece', 'processed', 'detail'])
        self.cycle.start()
        self.record('ENTER', self.cycle.step.detail)
        self.publish()
        response.success = True
        response.message = f'Started {self.run_id}'
        return response

    def cancel(self, _, response):
        if not self.cycle.active:
            response.message = 'No active task'
            return response
        self.fail('Canceled by operator', canceled=True)
        response.success = True
        response.message = 'Cancellation requested; wait for arm to stop before reset'
        return response

    def reset(self, _, response):
        if (self.cycle.active or self.reset_futures is not None or self.goal_future is not None
                or self.goal_handle is not None or self.abort_future is not None
                or self.pending is not None or self.arm is None or self.arm.busy):
            response.message = 'Task/arm is still active; wait for cancellation and commands to finish'
            return response
        if not self.machine_client.service_is_ready() or not self.arm_reset.service_is_ready():
            response.message = 'Reset services are unavailable'
            return response
        self.epoch += 1
        self.cycle.phase = 'RESETTING'
        request = MachineCommand.Request()
        request.command = 'RESET'
        self.reset_futures = [self.arm_reset.call_async(Trigger.Request()), self.machine_client.call_async(request)]
        self.reset_begin = time.monotonic()
        response.success = True
        response.message = 'Reset accepted; wait for task status IDLE'
        return response

    def record(self, event, detail):
        if self.writer:
            self.writer.writerow([round(time.monotonic()-self.begin, 3), self.cycle.phase,
                                  event, self.cycle.piece.location, self.cycle.piece.processed, detail])
            self.stream.flush()
        self.get_logger().info(f'{self.cycle.phase}: {detail}')

    def finish_evidence(self):
        self.end = time.monotonic()
        evidence = {
            'run_id': self.run_id, 'phase': self.cycle.phase, 'success': self.cycle.success,
            'completed': self.cycle.completed, 'elapsed_s': round(self.end-self.begin, 3),
            'workpiece_location': self.cycle.piece.location,
            'processed': self.cycle.piece.processed, 'error': self.cycle.error,
            'backend': 'SCARA joint interpolation; RViz kinematic simulation',
        }
        (self.output / f'{self.run_id}.json').write_text(
            json.dumps(evidence, indent=2, ensure_ascii=False), encoding='utf-8')
        if self.stream:
            self.stream.close()
            self.stream = self.writer = None

    def fail(self, reason, canceled=False):
        if not self.cycle.active:
            return
        self.cycle.fail(reason, canceled)
        self.record('TERMINAL', reason)
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        if self.machine_client.service_is_ready():
            request = MachineCommand.Request()
            request.command = 'ABORT'
            self.abort_future = self.machine_client.call_async(request)
        self.finish_evidence()
        self.publish()

    def advance(self):
        try:
            tcp = (self.arm.tcp.x, self.arm.tcp.y, self.arm.tcp.z)
            self.cycle.acknowledge(tcp, self.arm.gripper_closed)
        except ValueError as error:
            self.fail(str(error))
            return
        self.pending = None
        self.step_begin = time.monotonic()
        if self.cycle.active:
            self.record('ENTER', self.cycle.step.detail)
        else:
            self.record('TERMINAL', 'Finished workpiece verified in tray; arm returned home')
            self.finish_evidence()
        self.publish()

    def goal_response(self, future, epoch):
        self.goal_future = None
        try:
            handle = future.result()
        except Exception as error:
            self.pending = None
            self.fail(f'Action goal transport error: {error}')
            return
        if not handle.accepted:
            self.pending = None
            self.fail('Arm rejected the action goal')
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(lambda result: self.arm_result(result, epoch))
        if epoch != self.epoch or not self.cycle.active:
            handle.cancel_goal_async()

    def arm_result(self, future, epoch):
        self.goal_handle = None
        if epoch != self.epoch or not self.cycle.active:
            self.pending = None
            return
        try:
            result = future.result()
        except Exception as error:
            self.pending = None
            self.fail(f'Action result transport error: {error}')
            return
        if result.status != GoalStatus.STATUS_SUCCEEDED or not result.result.success:
            self.pending = None
            self.fail('Arm action failed: ' + result.result.message)
            return
        # The result can arrive before the next ArmState. Wait for the measured
        # target and gripper state before acknowledging attachment or placement.
        self.pending = 'verify_arm'

    def tick(self):
        now = time.monotonic()
        if self.abort_future is not None and self.abort_future.done():
            self.abort_future = None
        if not self.cycle.active and self.pending is not None and self.goal_future is None and self.goal_handle is None:
            if self.pending == 'verify_arm' or (hasattr(self.pending, 'done') and self.pending.done()):
                self.pending = None
        if self.reset_futures is not None:
            if all(f.done() for f in self.reset_futures):
                try:
                    replies = [f.result() for f in self.reset_futures]
                    if not all(r.success for r in replies):
                        raise ValueError('; '.join(r.message for r in replies if not r.success))
                    self.cycle.reset()
                    self.run_id = ''
                    self.begin = 0.0
                    self.end = None
                except Exception as error:
                    self.cycle.fail('Reset failed: ' + str(error))
                self.reset_futures = None
                self.publish()
            elif now-self.reset_begin > 5.0:
                # Keep the futures until resolved, so a late reset cannot affect a new run.
                self.cycle.phase = 'RESETTING'
                self.cycle.error = 'Reset response delayed; check node connectivity'
            return
        if not self.cycle.active:
            return
        if not self.ready():
            self.fail('Communication lost: missing arm/machine heartbeat')
            return
        if self.machine.state == 'FAULT':
            self.fail('Machine reported FAULT')
            return
        step = self.cycle.step
        elapsed = now-self.step_begin
        limit = (self.station_timeout if step.kind == 'ready' else
                 self.process_timeout if step.kind == 'process' else self.motion_seconds+5.0)
        if elapsed > limit:
            self.fail(f'{step.phase} timeout after {limit:.1f}s')
            return
        if step.kind == 'ready':
            if self.machine.state == 'READY':
                self.advance()
            return
        if step.kind == 'process':
            if self.machine.state == 'DONE' and self.machine.workpiece_present:
                self.advance()
            return
        if self.pending == 'verify_arm':
            point = (self.arm.tcp.x, self.arm.tcp.y, self.arm.tcp.z)
            grip_ok = step.gripper == -1 or self.arm.gripper_closed == bool(step.gripper)
            if not self.arm.busy and grip_ok and math.dist(point, step.target) < 0.005:
                self.advance()
            return
        if self.pending is not None:
            if step.kind == 'start_machine' and self.pending.done():
                try:
                    result = self.pending.result()
                    if not result.success:
                        raise ValueError(result.message)
                    self.advance()
                except Exception as error:
                    self.pending = None
                    self.fail('Machine START rejected: ' + str(error))
            return
        if step.kind == 'arm':
            if self.arm.busy:
                return
            goal = ArmMove.Goal()
            goal.target.x, goal.target.y, goal.target.z = step.target
            goal.gripper = step.gripper
            goal.duration = 0.5 if step.location else self.motion_seconds
            self.pending = 'arm'
            epoch = self.epoch
            self.goal_future = self.arm_client.send_goal_async(goal)
            self.goal_future.add_done_callback(lambda f: self.goal_response(f, epoch))
        elif step.kind == 'start_machine':
            if not self.machine.workpiece_present:
                return  # Let the published placement reach the machine before START.
            request = MachineCommand.Request()
            request.command = 'START'
            self.pending = self.machine_client.call_async(request)

    def publish(self):
        status = TaskStatus()
        status.run_id, status.phase = self.run_id, self.cycle.phase
        status.active, status.success = self.cycle.active, self.cycle.success
        status.completed, status.error = self.cycle.completed, self.cycle.error
        end = self.end if self.end is not None else time.monotonic()
        status.elapsed = max(0.0, end-self.begin) if self.begin else 0.0
        status.progress = self.cycle.index / len(self.cycle.plan)
        status.detail = self.cycle.step.detail if self.cycle.active else {
            'IDLE': 'Ready: call /a2/task/start', 'COMPLETE': 'Finished workpiece returned to tray',
            'RESETTING': 'Restoring simulation scene', 'FAILED': self.cycle.error,
            'CANCELED': self.cycle.error}.get(self.cycle.phase, '')
        self.status_pub.publish(status)
        piece = WorkpieceState()
        piece.location, piece.processed = self.cycle.piece.location, self.cycle.piece.processed
        self.piece_pub.publish(piece)


def main(args=None):
    rclpy.init(args=args)
    node = TaskNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.cycle.active:
            node.fail('Task manager interrupted')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
