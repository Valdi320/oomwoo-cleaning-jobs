"""Deterministic ROS2 ActionClient callback-race fault injection tests.

No ROS required to run these tests; the same adapter is used with a real
rclpy ActionClient in the integration harness.
"""
from __future__ import annotations
from types import SimpleNamespace
import unittest

from oomwoo_cleaning_jobs.nav_interlock import Goal
from oomwoo_cleaning_jobs.ros_nav2 import AsyncNav2Transport


class FakeFuture:
    def __init__(self):
        self.done = False
        self.value = None
        self.error = None
        self.callbacks = []
    def add_done_callback(self, callback):
        self.callbacks.append(callback)
        if self.done:
            callback(self)
    def finish(self, value=None, error=None):
        self.done = True
        self.value = value
        self.error = error
        for callback in list(self.callbacks):
            callback(self)
    def result(self):
        if self.error:
            raise self.error
        if not self.done:
            raise AssertionError("unfinished future")
        return self.value


class FakeHandle:
    def __init__(self, accepted=True):
        self.accepted = accepted
        self.result_future = FakeFuture()
        self.cancel_future = FakeFuture()
        self.cancel_calls = 0
        self.raise_cancel = False
        self.raise_result = False
    def get_result_async(self):
        if self.raise_result:
            raise RuntimeError("action lost result channel")
        return self.result_future
    def cancel_goal_async(self):
        self.cancel_calls += 1
        if self.raise_cancel:
            raise RuntimeError("cancel request transport broke")
        return self.cancel_future


class FakeActionClient:
    def __init__(self):
        self.sent = []
        self.futures = []
        self.send_error = None
    def send_goal_async(self, request):
        self.sent.append(request)
        if self.send_error is not None:
            raise self.send_error
        f = FakeFuture()
        self.futures.append(f)
        return f
    def wait_for_server(self, timeout_sec):
        return timeout_sec >= 0


class AsyncTransportTests(unittest.TestCase):
    def setUp(self):
        self.client = FakeActionClient()
        self.terminals = []
        self.faults = []
        self.port = AsyncNav2Transport(
            self.client, lambda g: ("POSE", g.target_cell),
            on_terminal=lambda token, outcome: self.terminals.append((token, outcome)),
            on_fault=lambda token, reason: self.faults.append((token, reason)),
        )
        self.goal = Goal("boot1-1", "unit", 2, (0, 1, 2), "a" * 64)

    def accepted(self, accepted=True):
        self.port.send_goal(self.goal)
        handle = FakeHandle(accepted=accepted)
        self.client.futures[-1].finish(handle)
        return handle

    def test_real_action_interface_adapter_waits_for_server(self):
        self.assertTrue(self.port.wait_for_server(timeout_sec=5))

    def test_send_uses_goal_builder_and_holds_pending(self):
        self.port.send_goal(self.goal)
        self.assertEqual(self.client.sent, [("POSE", 2)])
        self.assertEqual(self.port.outstanding_tokens(), (self.goal.token,))

    def test_accepted_success_terminal(self):
        handle = self.accepted()
        handle.result_future.finish(SimpleNamespace(status=4))
        self.assertEqual(self.terminals, [(self.goal.token, "succeeded")])
        self.assertEqual(self.port.outstanding_tokens(), ())

    def test_rejected_result_frees_without_result_future(self):
        self.accepted(accepted=False)
        self.assertEqual(self.terminals, [(self.goal.token, "rejected")])
        self.assertEqual(self.port.outstanding_tokens(), ())

    def test_cancel_before_goal_accepted_queues_until_handle_exists(self):
        self.port.send_goal(self.goal)
        self.port.request_cancel(self.goal.token)
        h = FakeHandle()
        self.client.futures[-1].finish(h)
        self.assertEqual(h.cancel_calls, 1)
        self.assertEqual(self.terminals, [])
        self.assertEqual(self.port.outstanding_tokens(), (self.goal.token,))
        h.cancel_future.finish(SimpleNamespace(goals_canceling=[object()]))
        self.assertEqual(self.terminals, [])  # Cancellation ACK != terminal.
        h.result_future.finish(SimpleNamespace(status=5))
        self.assertEqual(self.terminals, [(self.goal.token, "canceled")])

    def test_cancel_after_acceptance(self):
        h = self.accepted()
        self.port.request_cancel(self.goal.token)
        self.assertEqual(h.cancel_calls, 1)
        h.cancel_future.finish(SimpleNamespace(goals_canceling=[object()]))
        self.assertEqual(self.terminals, [])
        h.result_future.finish(SimpleNamespace(status=5))
        self.assertEqual(self.terminals[0][1], "canceled")

    def test_double_cancel_only_one_request(self):
        h = self.accepted()
        self.port.request_cancel(self.goal.token)
        self.port.request_cancel(self.goal.token)
        self.assertEqual(h.cancel_calls, 1)

    def test_cancel_unknown_token_is_noop(self):
        self.port.request_cancel("unseen")
        self.assertEqual(self.terminals, [])

    def test_duplicate_send_refused(self):
        self.port.send_goal(self.goal)
        with self.assertRaises(ValueError):
            self.port.send_goal(self.goal)
        self.assertEqual(len(self.client.sent), 1)

    def test_future_acceptance_exception_is_uncertain_fault(self):
        self.port.send_goal(self.goal)
        self.client.futures[0].finish(error=RuntimeError("communication lost"))
        self.assertEqual(self.faults, [(self.goal.token, "NAV2_ACCEPTANCE_UNCERTAIN")])
        self.assertEqual(self.port.outstanding_tokens(), (self.goal.token,))
        self.assertEqual(self.terminals, [])

    def test_result_future_missing_is_uncertain_fault(self):
        self.port.send_goal(self.goal)
        h = FakeHandle()
        h.raise_result = True
        self.client.futures[0].finish(h)
        self.assertEqual(self.faults[0][1], "NAV2_RESULT_FUTURE_UNCERTAIN")
        self.assertEqual(self.terminals, [])

    def test_cancel_ack_without_goal_reports_fault_not_result(self):
        h = self.accepted()
        self.port.request_cancel(self.goal.token)
        h.cancel_future.finish(SimpleNamespace(goals_canceling=[]))
        self.assertEqual(self.faults[0][1], "NAV2_CANCEL_NOT_ACCEPTED")
        self.assertEqual(self.terminals, [])
        self.assertEqual(self.port.outstanding_tokens(), (self.goal.token,))

    def test_cancel_response_exception_reports_fault(self):
        h = self.accepted()
        self.port.request_cancel(self.goal.token)
        h.cancel_future.finish(error=RuntimeError("cancel ack lost"))
        self.assertEqual(self.faults[0][1], "NAV2_CANCEL_UNCERTAIN")
        self.assertEqual(self.terminals, [])

    def test_cancel_transport_raise_reports_fault(self):
        h = self.accepted()
        h.raise_cancel = True
        self.port.request_cancel(self.goal.token)
        self.assertEqual(self.faults[0][1], "NAV2_CANCEL_UNCERTAIN")
        self.assertEqual(self.terminals, [])

    def test_result_future_exception_reports_fault(self):
        h = self.accepted()
        h.result_future.finish(error=RuntimeError("lost action result"))
        self.assertEqual(self.faults[0][1], "NAV2_RESULT_UNCERTAIN")
        self.assertEqual(self.terminals, [])

    def test_unknown_status_refuses_success(self):
        h = self.accepted()
        h.result_future.finish(SimpleNamespace(status=2))
        self.assertEqual(self.faults[0][1], "NAV2_RESULT_UNKNOWN_STATUS")
        self.assertEqual(self.terminals, [])

    def test_aborted_result_carries_abort(self):
        h = self.accepted()
        h.result_future.finish(SimpleNamespace(status=6))
        self.assertEqual(self.terminals, [(self.goal.token, "aborted")])

    def test_late_cancel_ack_after_terminal_ignored(self):
        h = self.accepted()
        self.port.request_cancel(self.goal.token)
        h.result_future.finish(SimpleNamespace(status=5))
        h.cancel_future.finish(SimpleNamespace(goals_canceling=[]))
        self.assertEqual(self.terminals, [(self.goal.token, "canceled")])
        self.assertEqual(self.faults, [])

    def test_late_duplicate_result_ignored(self):
        h = self.accepted()
        h.result_future.finish(SimpleNamespace(status=4))
        self.port._result(self.goal.token, h.result_future)
        self.assertEqual(len(self.terminals), 1)

    def test_explicit_fault_callback_surfaces_send_error(self):
        self.client.send_error = OSError("timeout")
        with self.assertRaises(OSError):
            self.port.send_goal(self.goal)
        # Integration with interlock will quarantine an uncertain send.
        self.assertEqual(self.port.outstanding_tokens(), (self.goal.token,))
        self.port.request_cancel(self.goal.token)
        self.assertEqual(self.terminals, [])


if __name__ == "__main__":
    unittest.main()
