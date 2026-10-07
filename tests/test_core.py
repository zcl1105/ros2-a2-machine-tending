"""Behavioral tests: transfers, interlocks, failures, reset and joint geometry."""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/a2_demo'))
from a2_demo.core import Cycle, HOME, Machine, STATION, TRAY, Workpiece, forward, inverse, interpolate


class KinematicsTests(unittest.TestCase):
    def test_forward_inverse_roundtrip_for_all_task_targets(self):
        for step in Cycle().plan:
            if step.kind == 'arm':
                actual = forward(inverse(step.target))
                self.assertLess(math.dist(actual, step.target), 1e-9)

    def test_unreachable_nan_and_joint_limit_targets_are_rejected(self):
        for point in [(2.0, 0, 0.4), (0, 0, 0.4), (0.4, 0, 0.1), (0.4, 0, 0.8), (math.nan, 0, 0.4)]:
            with self.subTest(point=point), self.assertRaises(ValueError):
                inverse(point)

    def test_transfer_moves_remain_at_safe_height(self):
        plan = Cycle().plan
        for a, b in zip(plan, plan[1:]):
            if a.kind == b.kind == 'arm' and a.target[:2] != b.target[:2]:
                for i in range(101):
                    position = forward(interpolate(inverse(a.target), inverse(b.target), i/100))
                    self.assertGreaterEqual(position[2], 0.55)

    def test_interpolation_clamps_and_reaches_endpoints(self):
        self.assertEqual(interpolate((0.0, 1.0), (2.0, 3.0), -1), (0.0, 1.0))
        self.assertEqual(interpolate((0.0, 1.0), (2.0, 3.0), 2), (2.0, 3.0))


class MachineTests(unittest.TestCase):
    def test_start_requires_present_part_and_arm_clear(self):
        machine = Machine()
        for present, clear in [(False, True), (True, False), (False, False)]:
            self.assertFalse(machine.command('START', 0, present, clear)[0])
        self.assertTrue(machine.command('START', 0, True, True)[0])

    def test_busy_station_waits_then_becomes_ready(self):
        machine = Machine(busy_seconds=4)
        self.assertEqual(machine.state(3.9), 'BUSY')
        self.assertFalse(machine.command('START', 3.9, True, True)[0])
        self.assertEqual(machine.state(4), 'READY')

    def test_processing_completion_requires_elapsed_process_time(self):
        machine = Machine(process_seconds=5)
        machine.command('START', 0, True, True)
        self.assertEqual(machine.state(4.9), 'PROCESSING')
        self.assertEqual(machine.state(5), 'DONE')
        self.assertFalse(machine.command('START', 6, True, True)[0])

    def test_stall_does_not_report_done(self):
        machine = Machine(stall=True)
        machine.command('START', 0, True, True)
        self.assertEqual(machine.state(1000), 'PROCESSING')

    def test_abort_fault_and_reset(self):
        machine = Machine()
        machine.command('START', 0, True, True)
        machine.command('ABORT', 2, True, True)
        self.assertEqual(machine.state(10), 'FAULT')
        self.assertFalse(machine.command('START', 10, True, True)[0])
        machine.command('RESET', 11, True, True)
        self.assertEqual(machine.state(11), 'READY')

    def test_busy_injection_cannot_disguise_processing(self):
        machine = Machine()
        machine.command('START', 0, True, True)
        self.assertFalse(machine.command('BUSY_ON', 1, True, True)[0])
        self.assertEqual(machine.state(1), 'PROCESSING')

    def test_fault_injection_requires_new_start_after_clearing(self):
        machine = Machine()
        machine.command('START', 0, True, True)
        machine.command('FAULT_ON', 1, True, True)
        self.assertEqual(machine.state(1), 'FAULT')
        machine.command('FAULT_OFF', 2, True, True)
        self.assertEqual(machine.state(2), 'READY')
        self.assertFalse(machine.done)


class WorkpieceTests(unittest.TestCase):
    def test_cannot_grasp_from_wrong_pose(self):
        piece = Workpiece()
        with self.assertRaises(ValueError):
            piece.transfer('GRIPPER', HOME, True)
        self.assertEqual(piece.location, 'TRAY')

    def test_cannot_release_without_holding_or_with_closed_gripper(self):
        piece = Workpiece()
        with self.assertRaises(ValueError):
            piece.transfer('MACHINE', STATION, False)
        piece.transfer('GRIPPER', TRAY, True)
        with self.assertRaises(ValueError):
            piece.transfer('MACHINE', STATION, True)
        self.assertEqual(piece.location, 'GRIPPER')


class CycleTests(unittest.TestCase):
    def test_complete_cycle_verified_at_tray(self):
        cycle = Cycle()
        cycle.start()
        closed = False
        tcp = HOME
        while cycle.active:
            step = cycle.step
            if step.kind == 'arm':
                tcp = step.target
                if step.gripper != -1:
                    closed = bool(step.gripper)
            cycle.acknowledge(tcp, closed)
        self.assertEqual(cycle.phase, 'COMPLETE')
        self.assertTrue(cycle.success)
        self.assertEqual(cycle.completed, 1)
        self.assertTrue(cycle.piece.processed)
        self.assertEqual(cycle.piece.location, 'TRAY')
        self.assertEqual(tcp, HOME)
        self.assertFalse(closed)

    def test_duplicate_start_and_active_reset_rejected(self):
        cycle = Cycle()
        cycle.start()
        with self.assertRaises(ValueError):
            cycle.start()
        with self.assertRaises(ValueError):
            cycle.reset()

    def test_cancel_preserves_held_part_and_requires_reset(self):
        cycle = Cycle()
        cycle.start()
        closed = False
        while cycle.phase != 'PICK_LIFT':
            step = cycle.step
            if step.gripper != -1:
                closed = bool(step.gripper)
            cycle.acknowledge(step.target, closed)
        cycle.fail('Canceled by operator', canceled=True)
        self.assertEqual(cycle.phase, 'CANCELED')
        self.assertEqual(cycle.piece.location, 'GRIPPER')
        self.assertEqual(cycle.completed, 0)
        with self.assertRaises(ValueError):
            cycle.start()
        cycle.reset()
        self.assertEqual(cycle.phase, 'IDLE')
        self.assertEqual(cycle.piece.location, 'TRAY')

    def test_failure_does_not_count_as_completion(self):
        cycle = Cycle()
        cycle.start()
        cycle.fail('Processing timeout')
        self.assertFalse(cycle.success)
        self.assertEqual(cycle.completed, 0)
        self.assertEqual(cycle.phase, 'FAILED')
        self.assertEqual(cycle.error, 'Processing timeout')


if __name__ == '__main__':
    unittest.main(verbosity=2)
