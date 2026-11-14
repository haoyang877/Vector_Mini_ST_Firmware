"""常驻设备进程的生命周期、任务隔离与通道复用；全程使用模拟端口。"""

import queue
import sys
import tempfile
import threading
import time
import unittest
from contextlib import nullcontext
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools/bench"))
import can_session as session


class Port:
    opened = 0
    closed = 0

    def __init__(self, *args, **kwargs):
        pass

    def __enter__(self):
        Port.opened += 1
        return self

    def __exit__(self, *args):
        Port.closed += 1


class SessionTests(unittest.TestCase):
    def setUp(self):
        Port.opened = Port.closed = 0

    def test_port_reuse_reconfigure_and_close(self):
        with (
            patch.object(session, "ControlCanFd", Port),
            patch.object(session.os, "add_dll_directory", return_value=nullcontext()),
        ):
            connection = session.CanConnection()
            first, reused = connection.open(Path("sdk.dll"), {"channel_index": 0})
            self.assertFalse(reused)
            second, reused = connection.open(Path("sdk.dll"), {"channel_index": 0})
            self.assertTrue(reused)
            self.assertIs(first, second)
            self.assertEqual((Port.opened, Port.closed), (1, 0))
            connection.open(Path("sdk.dll"), {"channel_index": 1})
            self.assertEqual((Port.opened, Port.closed), (2, 1))
            connection.close()
            connection.close()
            self.assertEqual(Port.closed, 2)

    def test_repeated_tasks_failure_and_release(self):
        requests, events, cancel = queue.Queue(), queue.Queue(), threading.Event()
        calls = []

        def read(*args, **kwargs):
            calls.append(1)
            return dict(profile="parameters", passed=len(calls) != 3)

        with (
            tempfile.TemporaryDirectory() as folder,
            patch.object(session, "ControlCanFd", Port),
            patch.object(session.os, "add_dll_directory", return_value=nullcontext()),
            patch.object(session, "run_parameters", side_effect=read),
        ):
            worker = threading.Thread(
                target=session.run_session, args=(Path("sdk.dll"), requests, events, cancel)
            )
            worker.start()
            try:
                config = dict(
                    profile="parameters",
                    node=6,
                    expected_uid="01" * 12,
                    group="current",
                    changes=None,
                    snapshot=None,
                    bus={},
                )
                for number in range(1, 5):
                    out = Path(folder) / str(number)
                    out.mkdir()
                    requests.put((number, config, out))
                    result = events.get(timeout=2)
                    self.assertEqual(result[:2], (number, "done"))
                    self.assertGreaterEqual(result[2]["protocol_elapsed_s"], 0)
                    self.assertGreaterEqual(result[2]["queue_elapsed_s"], 0)
                    self.assertGreaterEqual(
                        result[2]["total_elapsed_s"], result[2]["open_elapsed_s"]
                    )
                    self.assertEqual(result[2]["connection_reused"], number in (2, 3))
                self.assertEqual((Port.opened, Port.closed), (2, 1))
                requests.put((5, dict(profile="can_release"), None))
                self.assertTrue(events.get(timeout=2)[2]["passed"])
                self.assertEqual((Port.opened, Port.closed), (2, 2))
            finally:
                requests.put((6, dict(profile="can_shutdown"), None))
                worker.join(2)
            self.assertFalse(worker.is_alive())

    def test_old_messages_cannot_complete_new_task(self):
        endpoint = SimpleNamespace(
            events=queue.Queue(),
            active=True,
            connected=True,
            process=SimpleNamespace(is_alive=lambda: True),
        )
        task = session.SessionTask(endpoint, 2)
        endpoint.events.put((1, "done", dict(profile="parameters", passed=True)))
        with self.assertRaises(queue.Empty):
            task.get_nowait()
        self.assertTrue(task.is_alive())
        endpoint.events.put((2, "done", dict(profile="parameters", passed=False)))
        self.assertEqual(task.get_nowait()[0], "done")
        self.assertFalse(task.is_alive())
        self.assertFalse(endpoint.connected)

    def test_submit_rejects_busy_and_dead_worker(self):
        client = session.CanSession.__new__(session.CanSession)
        client.active = True
        client.process = SimpleNamespace(is_alive=lambda: True)
        with self.assertRaises(RuntimeError):
            client.submit({}, None)
        client.active = False
        client.process = SimpleNamespace(is_alive=lambda: False)
        with self.assertRaises(RuntimeError):
            client.submit({}, None)

    def test_real_spawn_shutdown_never_opens_a_port(self):
        client = session.CanSession(Path("must-not-open.dll"))
        try:
            task = client.submit(dict(profile="can_shutdown"), None)
            deadline = time.monotonic() + 10
            while time.monotonic() < deadline:
                try:
                    tag, report = task.get_nowait()
                    break
                except queue.Empty:
                    time.sleep(0.01)
            else:
                self.fail("常驻进程没有返回关闭应答")
            self.assertEqual(tag, "done")
            self.assertTrue(report["passed"])
            client.process.join(5)
            self.assertFalse(client.process.is_alive())
        finally:
            if client.process.is_alive():
                client.process.terminate()
                client.process.join(5)
            client.dispose()
