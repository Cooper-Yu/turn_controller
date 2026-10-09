"""Lifecycle gates for preparation; no simulated or real robot is commanded."""

from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
import subprocess
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

spec = spec_from_file_location('sequence', Path(__file__).parents[1] / 'tools/prepare_and_turn.py')
sequence = module_from_spec(spec)
spec.loader.exec_module(sequence)


class PreparationGates(unittest.TestCase):
    def test_ready_requires_finish_and_clean_exit(self):
        process = Mock()
        process.poll.return_value = None
        process.wait.return_value = 0
        replies = [
            SimpleNamespace(accepted=True, message='state=WAITING waypoint=A'),
            SimpleNamespace(accepted=True, message='Shutdown requested'),
        ]
        with (
            patch.object(sequence.rclpy, 'ok', return_value=True),
            patch.object(sequence, 'request', side_effect=replies) as call,
        ):
            sequence.wait_prepared(Mock(), Mock(), process, 1)
        self.assertEqual([c.args[2] for c in call.call_args_list], ['status', 'finish'])
        process.wait.assert_called_once_with(timeout=5)

    def test_fault_never_requests_finish_or_resume(self):
        process = Mock()
        process.poll.return_value = None
        with (
            patch.object(sequence.rclpy, 'ok', return_value=True),
            patch.object(
                sequence,
                'request',
                return_value=SimpleNamespace(accepted=True, message='state=FAULT'),
            ) as call,
        ):
            with self.assertRaisesRegex(RuntimeError, 'FAULT'):
                sequence.wait_prepared(Mock(), Mock(), process, 1)
        self.assertEqual([c.args[2] for c in call.call_args_list], ['status'])
        process.wait.assert_not_called()

    def test_premature_clean_exit_is_not_readiness(self):
        process = Mock(returncode=0)
        process.poll.return_value = 0
        with patch.object(sequence.rclpy, 'ok', return_value=True):
            with self.assertRaisesRegex(RuntimeError, 'prematurely'):
                sequence.wait_prepared(Mock(), Mock(), process, 1)

    def test_existing_publisher_is_not_interrupted(self):
        ros = Mock()
        node = ros.create_node.return_value
        node.count_publishers.return_value = 2
        with (
            patch.object(sequence, 'rclpy', ros),
            patch.object(sequence, 'options', return_value=SimpleNamespace()),
            patch.object(sequence.signal, 'signal'),
            patch.object(sequence.subprocess, 'Popen') as launch,
        ):
            self.assertEqual(sequence.main(), 2)
        launch.assert_not_called()
        node.create_publisher.return_value.publish.assert_not_called()
        ros.shutdown.assert_called_once()

    def test_stuck_child_is_reaped(self):
        process = Mock()
        process.poll.return_value = None
        process.wait.side_effect = [
            subprocess.TimeoutExpired('prep', 3),
            subprocess.TimeoutExpired('prep', 2),
            0,
        ]
        sequence.terminate(process)
        process.send_signal.assert_called_once()
        process.terminate.assert_called_once()
        process.kill.assert_called_once()
        self.assertEqual(process.wait.call_count, 3)


if __name__ == '__main__':
    unittest.main()
