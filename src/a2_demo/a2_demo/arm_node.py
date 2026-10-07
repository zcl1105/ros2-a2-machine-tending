"""SCARA joint motion action server; interpolated joints, no contact physics."""
import math
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from a2_interfaces.action import ArmMove
from a2_interfaces.msg import ArmState
from .core import HOME, forward, interpolate, inverse


class ArmNode(Node):
    def __init__(self):
        super().__init__('arm_sim')
        self.lock = threading.Lock()
        self.joints = inverse(HOME)
        self.closed = False
        self.busy = False
        group = ReentrantCallbackGroup()
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.state_pub = self.create_publisher(ArmState, '/a2/arm/state', 10)
        self.server = ActionServer(
            self, ArmMove, '/a2/arm/move', execute_callback=self.execute,
            goal_callback=self.goal, cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=group)
        self.create_service(Trigger, '/a2/arm/reset', self.reset, callback_group=group)
        self.create_timer(0.04, self.publish)

    def goal(self, request):
        try:
            inverse((request.target.x, request.target.y, request.target.z))
            if (request.gripper not in (-1, 0, 1) or not math.isfinite(request.duration)
                    or not 0.1 <= request.duration <= 30.0):
                raise ValueError('Invalid gripper or duration')
        except ValueError as error:
            self.get_logger().warning(str(error))
            return GoalResponse.REJECT
        with self.lock:
            if self.busy:
                return GoalResponse.REJECT
            self.busy = True
        return GoalResponse.ACCEPT

    def execute(self, handle):
        result = ArmMove.Result()
        request = handle.request
        target = inverse((request.target.x, request.target.y, request.target.z))
        with self.lock:
            start = self.joints
        begin = time.monotonic()
        try:
            while rclpy.ok():
                if handle.is_cancel_requested:
                    handle.canceled()
                    result.message = 'Canceled; stopped at the current pose'
                    return result
                fraction = min(1.0, (time.monotonic()-begin)/request.duration)
                joints = interpolate(start, target, fraction)
                with self.lock:
                    self.joints = joints
                feedback = ArmMove.Feedback()
                feedback.progress = float(fraction)
                feedback.tcp.x, feedback.tcp.y, feedback.tcp.z = forward(joints)
                handle.publish_feedback(feedback)
                if fraction >= 1.0:
                    with self.lock:
                        if request.gripper != -1:
                            self.closed = bool(request.gripper)
                    handle.succeed()
                    result.success = True
                    result.message = 'Target reached'
                    return result
                time.sleep(0.02)
            handle.abort()
            result.message = 'ROS shutdown'
            return result
        finally:
            with self.lock:
                self.busy = False

    def reset(self, _, response):
        with self.lock:
            if self.busy:
                response.message = 'Arm is moving; wait for cancellation to finish'
                return response
            self.joints = inverse(HOME)
            self.closed = False
        response.success = True
        response.message = 'Simulation arm restored to initial pose'
        return response

    def publish(self):
        with self.lock:
            joints, closed, busy = self.joints, self.closed, self.busy
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['shoulder_joint', 'elbow_joint', 'slide_joint', 'wrist_joint',
                    'left_finger_joint', 'right_finger_joint']
        gap = 0.018 if closed else 0.045
        msg.position = list(joints) + [gap, gap]
        self.joint_pub.publish(msg)
        state = ArmState()
        state.tcp.x, state.tcp.y, state.tcp.z = forward(joints)
        state.gripper_closed = closed
        state.busy = busy
        self.state_pub.publish(state)


def main(args=None):
    rclpy.init(args=args)
    node = ArmNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
