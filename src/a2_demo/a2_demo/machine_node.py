import math
import time

import rclpy
from rclpy.node import Node
from a2_interfaces.msg import ArmState, MachineState, WorkpieceState
from a2_interfaces.srv import MachineCommand
from .core import Machine, STATION


class MachineNode(Node):
    def __init__(self):
        super().__init__('machine_sim')
        self.declare_parameter('process_seconds', 5.0)
        self.declare_parameter('busy_seconds', 0.0)
        self.declare_parameter('stall', False)
        self.begin = time.monotonic()
        self.model = Machine(
            float(self.get_parameter('process_seconds').value),
            float(self.get_parameter('busy_seconds').value),
            bool(self.get_parameter('stall').value))
        self.present = False
        self.arm = None
        self.arm_time = 0.0
        self.piece_time = 0.0
        self.pub = self.create_publisher(MachineState, '/a2/machine/state', 10)
        self.create_subscription(WorkpieceState, '/a2/workpiece', self.piece_cb, 10)
        self.create_subscription(ArmState, '/a2/arm/state', self.arm_cb, 10)
        self.create_service(MachineCommand, '/a2/machine/command', self.command)
        self.create_timer(0.1, self.tick)

    def piece_cb(self, msg):
        self.present = msg.location == 'MACHINE'
        self.piece_time = time.monotonic()

    def arm_cb(self, msg):
        self.arm = msg
        self.arm_time = time.monotonic()

    def command(self, request, response):
        now = time.monotonic()
        clear = (self.arm is not None and now-self.arm_time < 1.0
                 and not self.arm.busy and not self.arm.gripper_closed
                 and math.hypot(self.arm.tcp.x-STATION[0], self.arm.tcp.y-STATION[1]) > 0.18)
        present = self.present and now-self.piece_time < 1.0
        response.success, response.message = self.model.command(
            request.command, now-self.begin, present, clear)
        self.tick()
        return response

    def tick(self):
        now = time.monotonic()-self.begin
        msg = MachineState()
        msg.state = self.model.state(now)
        msg.workpiece_present = self.present
        msg.elapsed = max(0.0, now-self.model.started) if self.model.started is not None else 0.0
        msg.progress = (1.0 if msg.state == 'DONE' else
                        min(0.95 if self.model.stall else 1.0,
                            msg.elapsed/self.model.process_seconds) if msg.state == 'PROCESSING' else 0.0)
        msg.detail = {'READY': 'Ready for loading', 'BUSY': 'Station occupied',
                      'PROCESSING': 'Machining', 'DONE': 'Finished', 'FAULT': 'Reset required'}[msg.state]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MachineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
