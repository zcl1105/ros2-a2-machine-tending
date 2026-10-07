"""Deterministic kinematics and task logic, independent of ROS and wall time."""
import math
from dataclasses import dataclass

L1, L2 = 0.36, 0.30
TCP_BASE_Z = 0.70
TRAY = (0.42, -0.25, 0.37)
STATION = (0.42, 0.25, 0.37)
HOME = (0.50, 0.0, 0.56)
SAFE_Z = 0.56


def inverse(point):
    x, y, z = point
    if not all(math.isfinite(v) for v in point):
        raise ValueError('Target must contain finite coordinates')
    c2 = (x*x + y*y - L1*L1 - L2*L2) / (2*L1*L2)
    if not -1.0 <= c2 <= 1.0:
        raise ValueError('Target is outside the arm workspace')
    q2 = math.acos(c2)
    q1 = math.atan2(y, x) - math.atan2(L2*math.sin(q2), L1+L2*math.cos(q2))
    slide = TCP_BASE_Z - z
    wrist = -(q1 + q2)
    if not (-math.pi <= q1 <= math.pi and 0 <= q2 <= 2.8
            and -1e-9 <= slide <= 0.40+1e-9 and -math.pi <= wrist <= math.pi):
        raise ValueError('Target violates a joint limit')
    return (q1, q2, max(0.0, min(0.40, slide)), wrist)


def forward(joints):
    q1, q2, slide, _ = joints
    return (L1*math.cos(q1)+L2*math.cos(q1+q2),
            L1*math.sin(q1)+L2*math.sin(q1+q2), TCP_BASE_Z-slide)


def interpolate(start, target, fraction):
    u = max(0.0, min(1.0, fraction))
    u = u*u*(3.0-2.0*u)
    return tuple(a+(b-a)*u for a, b in zip(start, target))


@dataclass(frozen=True)
class Step:
    phase: str
    kind: str
    detail: str
    target: tuple = HOME
    gripper: int = -1
    location: str = ''


def cycle_plan():
    above_tray = (TRAY[0], TRAY[1], SAFE_Z)
    above_machine = (STATION[0], STATION[1], SAFE_Z)
    return (
        Step('WAIT_READY', 'ready', 'Waiting for the machining station'),
        Step('APPROACH_TRAY', 'arm', 'Move above the raw material tray', above_tray, 0),
        Step('PICK_DESCEND', 'arm', 'Lower the gripper to the workpiece', TRAY),
        Step('PICK_GRIP', 'arm', 'Close gripper and attach raw workpiece', TRAY, 1, 'GRIPPER'),
        Step('PICK_LIFT', 'arm', 'Lift the workpiece to the safe transfer height', above_tray),
        Step('TRANSFER_LOAD', 'arm', 'Transfer the workpiece above the machine', above_machine),
        Step('LOAD_DESCEND', 'arm', 'Lower the workpiece into the fixture', STATION),
        Step('LOAD_RELEASE', 'arm', 'Open gripper and place the workpiece', STATION, 0, 'MACHINE'),
        Step('LOAD_RETREAT', 'arm', 'Lift clear of the machining station', above_machine),
        Step('CLEAR_MACHINE', 'arm', 'Park outside the machining area', HOME),
        Step('START_MACHINE', 'start_machine', 'Request machining after the arm retreats'),
        Step('WAIT_PROCESS', 'process', 'Wait for the machining completion signal'),
        Step('APPROACH_MACHINE', 'arm', 'Move above the finished workpiece', above_machine),
        Step('UNLOAD_DESCEND', 'arm', 'Lower gripper to the finished workpiece', STATION),
        Step('UNLOAD_GRIP', 'arm', 'Grip and attach the finished workpiece', STATION, 1, 'GRIPPER'),
        Step('UNLOAD_LIFT', 'arm', 'Lift the finished workpiece', above_machine),
        Step('TRANSFER_RETURN', 'arm', 'Return the finished workpiece above the tray', above_tray),
        Step('RETURN_DESCEND', 'arm', 'Lower the finished workpiece into its tray slot', TRAY),
        Step('RETURN_RELEASE', 'arm', 'Release the finished workpiece', TRAY, 0, 'TRAY'),
        Step('RETURN_LIFT', 'arm', 'Lift clear of the tray', above_tray),
        Step('RETURN_HOME', 'arm', 'Return home with gripper open', HOME, 0),
    )


class Workpiece:
    def __init__(self):
        self.location = 'TRAY'
        self.processed = False

    def transfer(self, destination, tcp, closed):
        if destination == 'GRIPPER':
            expected = TRAY if self.location == 'TRAY' else STATION
            if self.location not in ('TRAY', 'MACHINE') or not closed:
                raise ValueError('Cannot grasp: missing workpiece or open gripper')
        else:
            expected = TRAY if destination == 'TRAY' else STATION
            if destination not in ('TRAY', 'MACHINE') or self.location != 'GRIPPER' or closed:
                raise ValueError('Cannot place: no held workpiece or gripper still closed')
        if math.dist(tcp, expected) > 0.015:
            raise ValueError('TCP is not at the workpiece/fixture position')
        self.location = destination


class Machine:
    def __init__(self, process_seconds=5.0, busy_seconds=0.0, stall=False):
        if not math.isfinite(process_seconds) or process_seconds <= 0:
            raise ValueError('process_seconds must be positive and finite')
        if not math.isfinite(busy_seconds) or busy_seconds < 0:
            raise ValueError('busy_seconds must be nonnegative and finite')
        self.process_seconds = process_seconds
        self.initial_busy = busy_seconds
        self.stall = stall
        self.busy_until = busy_seconds
        self.manual_busy = False
        self.fault = False
        self.started = None
        self.done = False

    def state(self, now):
        if self.fault:
            return 'FAULT'
        if self.started is not None:
            if not self.stall and now-self.started >= self.process_seconds:
                self.started = None
                self.done = True
            else:
                return 'PROCESSING'
        if self.done:
            return 'DONE'
        if self.manual_busy or now < self.busy_until:
            return 'BUSY'
        return 'READY'

    def command(self, command, now, present, arm_clear):
        state = self.state(now)
        if command == 'START':
            if state != 'READY' or not present or not arm_clear:
                return False, 'START requires READY, workpiece in fixture, and arm clear'
            self.started = now
            return True, 'Machining started'
        if command == 'ABORT':
            self.started = None
            self.fault = True
            self.done = False
            return True, 'Machining stopped; reset required'
        if command == 'RESET':
            self.started = None
            self.fault = False
            self.done = False
            self.manual_busy = False
            self.busy_until = now + self.initial_busy
            return True, 'Machine reset'
        if command in ('BUSY_ON', 'BUSY_OFF'):
            if state in ('PROCESSING', 'DONE', 'FAULT'):
                return False, 'Busy injection is only allowed in READY/BUSY'
            self.manual_busy = command == 'BUSY_ON'
            if not self.manual_busy:
                self.busy_until = now
            return True, command
        if command == 'FAULT_ON':
            self.fault = True
            self.started = None
            self.done = False
            return True, 'Fault injected'
        if command == 'FAULT_OFF':
            self.fault = False
            return True, 'Fault cleared'
        return False, 'Unknown machine command'


class Cycle:
    """Commands are acknowledged before advancing; no time-based fake success."""
    def __init__(self):
        self.plan = cycle_plan()
        self.piece = Workpiece()
        self.index = 0
        self.phase = 'IDLE'
        self.active = False
        self.success = False
        self.completed = 0
        self.error = ''

    @property
    def step(self):
        return self.plan[self.index] if self.active else None

    def start(self):
        if self.phase != 'IDLE':
            raise ValueError('Reset to IDLE before starting another cycle')
        self.active = True
        self.phase = self.step.phase

    def acknowledge(self, tcp=None, closed=None):
        if not self.active:
            raise ValueError('No active cycle')
        step = self.step
        if step.location:
            self.piece.transfer(step.location, tcp, closed)
        if step.kind == 'process':
            self.piece.processed = True
        self.index += 1
        if self.index == len(self.plan):
            if self.piece.location != 'TRAY' or not self.piece.processed:
                self.fail('Final workpiece verification failed')
                return
            self.active = False
            self.success = True
            self.completed = 1
            self.phase = 'COMPLETE'
        else:
            self.phase = self.step.phase

    def fail(self, reason, canceled=False):
        self.active = False
        self.success = False
        self.error = reason
        self.phase = 'CANCELED' if canceled else 'FAILED'

    def reset(self):
        if self.active:
            raise ValueError('Cancel the active cycle before reset')
        self.__init__()
