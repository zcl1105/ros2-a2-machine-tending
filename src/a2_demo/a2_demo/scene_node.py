"""RViz scene, measured TCP trace, fixture signal lights and recording HUD."""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from a2_interfaces.msg import ArmState, MachineState, TaskStatus, WorkpieceState
from .core import HOME, STATION, TRAY

NAVY = (0.08, 0.13, 0.20, 1.0)
TEAL = (0.05, 0.75, 0.82, 1.0)
GREEN = (0.20, 0.86, 0.43, 1.0)
AMBER = (1.0, 0.63, 0.15, 1.0)
RED = (0.97, 0.23, 0.30, 1.0)
WHITE = (0.85, 0.93, 1.0, 1.0)


class SceneNode(Node):
    def __init__(self):
        super().__init__('scene_view')
        self.arm = None
        self.machine = None
        self.task = None
        self.piece = None
        self.trace = []
        self.run_id = ''
        self.pub = self.create_publisher(MarkerArray, '/a2/scene', 10)
        for message, topic, callback in [
            (ArmState, '/a2/arm/state', self.arm_cb),
            (MachineState, '/a2/machine/state', self.machine_cb),
            (TaskStatus, '/a2/task/status', self.task_cb),
            (WorkpieceState, '/a2/workpiece', self.piece_cb),
        ]:
            self.create_subscription(message, topic, callback, 10)
        self.create_timer(0.1, self.draw)

    def arm_cb(self, msg):
        self.arm = msg
        if self.task and self.task.active:
            position = (msg.tcp.x, msg.tcp.y, msg.tcp.z)
            if not self.trace or math.dist(position, self.trace[-1]) > 0.003:
                self.trace.append(position)
                self.trace = self.trace[-2500:]

    def machine_cb(self, msg):
        self.machine = msg

    def task_cb(self, msg):
        if msg.run_id != self.run_id or msg.phase == 'IDLE':
            self.trace = []
            self.run_id = msg.run_id
        self.task = msg

    def piece_cb(self, msg):
        self.piece = msg

    def marker(self, key, kind, xyz, scale, color, text=''):
        msg = Marker()
        msg.header.frame_id = 'world'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.ns = 'a2_cell'
        msg.id = key
        msg.type = kind
        msg.action = Marker.ADD
        msg.pose.orientation.w = 1.0
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, xyz)
        msg.scale.x, msg.scale.y, msg.scale.z = map(float, scale)
        # In Humble, TEXT_VIEW_FACING uses scale.x as explicit space width.
        # Its default is calculated before character height is updated, so a
        # zero value can leave spaces enormous and push the HUD off screen.
        if kind == Marker.TEXT_VIEW_FACING:
            msg.scale.x = float(scale[2])*0.55
        msg.color.r, msg.color.g, msg.color.b, msg.color.a = map(float, color)
        msg.text = text
        return msg

    def draw(self):
        phase = self.task.phase if self.task else 'CONNECTING'
        machine = self.machine.state if self.machine else 'OFFLINE'
        state_color = RED if phase in ('FAILED', 'CANCELED') else GREEN if phase == 'COMPLETE' else TEAL
        lamp = {'READY': GREEN, 'BUSY': AMBER, 'PROCESSING': TEAL, 'DONE': GREEN, 'FAULT': RED}.get(machine, RED)
        items = [
            self.marker(0, Marker.CUBE, (0.28, 0, 0.265), (1.10, 1.00, 0.07), NAVY),
            self.marker(1, Marker.CUBE, (0.42, -0.25, 0.305), (0.20, 0.19, 0.01), (0.25, 0.32, 0.42, 1.0)),
            self.marker(2, Marker.CUBE, (0.42, 0.25, 0.305), (0.18, 0.17, 0.01), (0.33, 0.43, 0.53, 1.0)),
            self.marker(3, Marker.CUBE, (0.62, 0.25, 0.47), (0.035, 0.38, 0.34), (0.25, 0.31, 0.38, 1.0)),
            self.marker(4, Marker.CUBE, (0.47, 0.45, 0.47), (0.33, 0.025, 0.34), (0.25, 0.31, 0.38, 1.0)),
            self.marker(5, Marker.CUBE, (0.62, 0.25, 0.66), (0.06, 0.38, 0.03), (0.25, 0.31, 0.38, 1.0)),
            self.marker(6, Marker.CYLINDER, (0.60, 0.42, 0.73), (0.045, 0.045, 0.09), lamp),
            self.marker(7, Marker.TEXT_VIEW_FACING, (0.42, -0.41, 0.34), (0.0, 0.0, 0.034), WHITE, 'MATERIAL TRAY'),
            self.marker(8, Marker.TEXT_VIEW_FACING, (0.49, 0.35, 0.81), (0.0, 0.0, 0.034), lamp, 'MACHINE | '+machine),
            self.marker(9, Marker.TEXT_VIEW_FACING, (0.25, -0.55, 1.40), (0.0, 0.0, 0.050), state_color, 'A2 MACHINE TENDING'),
        ]
        if self.piece and self.piece.location == 'GRIPPER' and self.arm:
            xyz = (self.arm.tcp.x, self.arm.tcp.y, self.arm.tcp.z-0.035)
        else:
            point = STATION if self.piece and self.piece.location == 'MACHINE' else TRAY
            xyz = (point[0], point[1], point[2]-0.035)
        processed = self.piece.processed if self.piece else False
        items.append(self.marker(10, Marker.CUBE, xyz, (0.035, 0.05, 0.07), GREEN if processed else AMBER))
        detail = self.task.detail if self.task else 'Waiting for ROS nodes'
        elapsed = self.task.elapsed if self.task else 0.0
        completed = self.task.completed if self.task else 0
        grip = 'CLOSED' if self.arm and self.arm.gripper_closed else 'OPEN'
        piece_location = self.piece.location if self.piece else 'TRAY'
        hud = f'{phase}\n{detail}\nTime {elapsed:05.1f}s  |  Completed {completed}/1\nGripper {grip}  |  Part {piece_location}\nPart: '+('PROCESSED' if processed else 'RAW')
        items.append(self.marker(11, Marker.TEXT_VIEW_FACING, (0.25, -0.55, 1.18), (0.0, 0.0, 0.033), WHITE, hud))
        progress = self.task.progress if self.task else 0.0
        items.append(self.marker(12, Marker.CUBE, (0.25, -0.55, 0.77), (0.66, 0.012, 0.018), NAVY))
        width = max(0.001, 0.66*progress)
        items.append(self.marker(13, Marker.CUBE, (-0.08+width/2, -0.56, 0.77), (width, 0.012, 0.020), state_color))
        trace = self.marker(14, Marker.LINE_STRIP, (0.0, 0.0, 0.0), (0.004, 0.0, 0.0), (0.05, 0.80, 0.95, 0.65))
        trace.points = [Point(x=p[0], y=p[1], z=p[2]) for p in self.trace]
        # Empty LINE_STRIP is permitted but avoid RViz's warning for a single point.
        if len(trace.points) < 2:
            trace.action = Marker.DELETE
        items.append(trace)
        machine_progress = self.machine.progress if self.machine else 0.0
        items.append(self.marker(15, Marker.TEXT_VIEW_FACING, (0.46, 0.18, 0.73), (0.0, 0.0, 0.025), lamp,
                                 f'Machining {machine_progress*100:3.0f}%'))
        # The guarded front panel is visible only while the process is running.
        door = self.marker(16, Marker.CUBE, (0.47, 0.06, 0.47), (0.33, 0.015, 0.34), (0.10, 0.60, 0.70, 0.30))
        if machine != 'PROCESSING':
            door.action = Marker.DELETE
        items.append(door)
        self.pub.publish(MarkerArray(markers=items))


def main(args=None):
    rclpy.init(args=args)
    node = SceneNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
