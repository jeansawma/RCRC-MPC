"""Run: python -m unittest discover -s tests -p test_desktop.py -v"""
import ctypes
import csv
import io
import json
import os
import queue
import time
import types
from pathlib import Path
import unittest
from unittest.mock import patch

from desktop.protocol import JsonLineDecoder, Parameter, Sample, ProtocolError, History
from desktop.transport import SerialTransport


def sample(t_us=1000, **changes):
    fields = dict(t_us=t_us, v1=5.1, v2=5.0, vbus=24.0, reference=5.0,
                  duty=0.22, mode=2, armed=1, fault=0, qp_residual=1e-6,
                  qp_iterations=4)
    fields.update(changes)
    return Sample.from_message(fields)


class DecoderTests(unittest.TestCase):
    def test_fragmented_and_coalesced_frames(self):
        decoder = JsonLineDecoder()
        self.assertEqual(decoder.feed(b'{"type":"hello",'), [])
        self.assertEqual(decoder.feed(b'"protocol":1}\r\n{"type":"ack"}\n'),
                         [{"type": "hello", "protocol": 1}, {"type": "ack"}])

    def test_malformed_json_does_not_poison_next_line(self):
        decoder = JsonLineDecoder()
        result = decoder.feed(b'{broken}\n{"type":"ack"}\n')
        self.assertEqual([item["type"] for item in result], ["parse_error", "ack"])

    def test_oversized_line_resynchronizes_and_memory_is_bounded(self):
        decoder = JsonLineDecoder(maximum=32)
        self.assertEqual(decoder.feed(b'x' * 10000)[0]["type"], "parse_error")
        self.assertLessEqual(len(decoder.pending), 32)
        self.assertEqual(decoder.feed(b'\n{"type":"ack"}\n'), [{"type": "ack"}])

    def test_nonfinite_json_constants_rejected(self):
        self.assertEqual(JsonLineDecoder().feed(b'{"type":"sample","v1":NaN}\n')[0]["type"], "parse_error")


class DataTests(unittest.TestCase):
    def test_parameter_checks_units_bounds_integrality_and_command_injection(self):
        parameter = Parameter("sampling_us", 1000, 500, 10000, "us", True)
        self.assertEqual(parameter.command("2000"), "SET sampling_us 2000")
        for value in ("nan", "inf", "100.1", "2000.5", "2000\nARM", "10001"):
            with self.assertRaises(ProtocolError, msg=value):
                parameter.command(value)
        with self.assertRaises(ProtocolError):
            Parameter("mode\nARM", 0, 0, 2).command("1")

    def test_samples_reject_null_nonfinite_and_invalid_mode(self):
        for changes in ({"v1": None}, {"duty": float("nan")}, {"mode": 4}, {"armed": 2}):
            with self.assertRaises(ProtocolError):
                sample(**changes)

    def test_history_bound_csv_and_timer_wrap(self):
        history = History(maximum=2)
        history.append(sample(2**32 - 1000))
        history.append(sample(500))
        history.append(sample(1500))
        self.assertEqual(len(history.samples), 2)
        self.assertEqual(len(history.display_times), 2)
        self.assertAlmostEqual(history.display_times[-1] - history.display_times[-2], .001)
        text = io.StringIO()
        history.export(text)
        rows = list(csv.DictReader(io.StringIO(text.getvalue())))
        self.assertEqual([row["t_us"] for row in rows], ["500", "1500"])
        self.assertEqual(float(rows[0]["v2"]), 5)
        self.assertEqual(int(rows[0]["qp_iterations"]), 4)

    def test_history_reset_starts_new_session(self):
        history = History()
        history.append(sample(100000))
        history.append(sample(1000))
        self.assertEqual(len(history.samples), 1)
        self.assertEqual(history.display_times[0], .001)


class FakeSerial:
    def __init__(self, respond_after=None):
        self.respond_after = respond_after
        self.writes = []
        self.received = bytearray()
        self.closed = False

    @property
    def in_waiting(self):
        return len(self.received)

    def write(self, data):
        self.writes.append(data)
        if data == b"HELLO\n" and self.respond_after is not None:
            if self.writes.count(b"HELLO\n") >= self.respond_after:
                self.received.extend(b'{"type":"hello","protocol":1}\n')

    def read(self, count):
        time.sleep(.002)
        result = bytes(self.received[:count])
        del self.received[:count]
        return result

    def close(self):
        self.closed = True


class SerialHandshakeTests(unittest.TestCase):
    def test_gui_arm_requires_completed_handshake(self):
        from desktop.app import Console
        console = Console.__new__(Console)
        sent = []
        statuses = []
        console.connected = True
        console.handshake = False
        console.transport = types.SimpleNamespace(send=sent.append)
        console.status = types.SimpleNamespace(set=statuses.append)
        console.send("ARM")
        self.assertEqual(sent, [])
        self.assertIn("handshake", statuses[-1])
        console.handshake = True
        console.send("ARM")
        self.assertEqual(sent, ["ARM"])

    def test_dropped_first_hello_retries_before_get(self):
        fake = FakeSerial(respond_after=2)
        events = queue.Queue()
        transport = SerialTransport(events, "fake")
        with patch.dict("sys.modules", {"serial": types.SimpleNamespace(Serial=lambda *a, **k: fake)}), \
             patch.object(SerialTransport, "HELLO_INTERVAL", .02):
            transport.start()
            deadline = time.monotonic() + 1
            while b"GET\n" not in fake.writes and time.monotonic() < deadline:
                time.sleep(.005)
            transport.close()
            transport.thread.join(1)
        self.assertFalse(transport.thread.is_alive())
        self.assertGreaterEqual(fake.writes.count(b"HELLO\n"), 2)
        self.assertEqual(fake.writes[:3], [b"HELLO\n", b"HELLO\n", b"GET\n"])
        self.assertEqual(fake.writes[-1], b"STOP\n")
        self.assertTrue(fake.closed)
        received = list(events.queue)
        self.assertTrue(any(item["type"] == "hello" for item in received))

    def test_handshake_timeout_is_finite_and_requests_stop(self):
        fake = FakeSerial()
        events = queue.Queue()
        transport = SerialTransport(events, "fake")
        with patch.dict("sys.modules", {"serial": types.SimpleNamespace(Serial=lambda *a, **k: fake)}), \
             patch.object(SerialTransport, "HELLO_INTERVAL", .01), \
             patch.object(SerialTransport, "HELLO_TIMEOUT", .05):
            transport.start()
            transport.thread.join(1)
        self.assertFalse(transport.thread.is_alive())
        self.assertNotIn(b"GET\n", fake.writes)
        self.assertEqual(fake.writes[-1], b"STOP\n")
        self.assertTrue(fake.closed)
        errors = [item for item in list(events.queue) if item["type"] == "transport_error"]
        self.assertEqual(len(errors), 1)
        self.assertIn("did not answer HELLO", errors[0]["message"])

    def test_stopping_during_handshake_does_not_wait_for_timeout(self):
        fake = FakeSerial()
        transport = SerialTransport(queue.Queue(), "fake")
        with patch.dict("sys.modules", {"serial": types.SimpleNamespace(Serial=lambda *a, **k: fake)}):
            transport.start()
            time.sleep(.01)
            transport.close()
            transport.thread.join(.5)
        self.assertFalse(transport.thread.is_alive())
        self.assertEqual(fake.writes[-1], b"STOP\n")


LIBRARY = Path(os.environ.get("RCRC_SIM_LIBRARY", Path(__file__).resolve().parents[1] / "build/libsimulation.so"))


@unittest.skipUnless(LIBRARY.exists(), "make host builds the C simulation library")
class CSimulationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.lib = ctypes.CDLL(str(LIBRARY))
        cls.lib.sim_create.restype = ctypes.c_void_p
        cls.lib.sim_destroy.argtypes = [ctypes.c_void_p]
        cls.lib.sim_command.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_void_p, ctypes.c_size_t]
        cls.lib.sim_advance.argtypes = [ctypes.c_void_p, ctypes.c_uint32, ctypes.c_void_p, ctypes.c_size_t]
        cls.lib.sim_sampling_us.argtypes = [ctypes.c_void_p]
        cls.lib.sim_sampling_us.restype = ctypes.c_uint32

    def setUp(self):
        self.sim = self.lib.sim_create()
        self.assertTrue(self.sim)
        self.buffer = ctypes.create_string_buffer(16384)

    def tearDown(self):
        self.lib.sim_destroy(self.sim)

    def command(self, text):
        self.assertEqual(self.lib.sim_command(self.sim, text.encode(), self.buffer, len(self.buffer)), 1)
        return [json.loads(line) for line in self.buffer.value.splitlines()]

    def advance(self, steps):
        self.lib.sim_advance(self.sim, steps, self.buffer, len(self.buffer))
        return json.loads(self.buffer.value)

    def test_metadata_and_atomic_rejection(self):
        metadata = self.command("GET")
        names = {message["name"]: message for message in metadata if message["type"] == "param"}
        self.assertIn("sampling_us", names)
        self.assertIn("mpc_horizon", names)
        self.assertEqual(metadata[-1]["type"], "params_end")
        for command in ("SET r1 nan", "SET sampling_us 0", "SET mode 8", "SET r1 10 junk"):
            self.assertEqual(self.command(command)[0]["type"], "error")
        updated = {message["name"]: message["value"] for message in self.command("GET") if message["type"] == "param"}
        self.assertEqual(updated["r1"], names["r1"]["value"])

    def test_controller_modes_close_loop_and_stop(self):
        for mode in range(3):
            with self.subTest(mode=mode):
                self.command("STOP")
                self.assertEqual(self.command(f"SET mode {mode}")[0]["type"], "ack")
                self.command("ARM")
                result = self.advance(5_000_000 // self.lib.sim_sampling_us(self.sim))
                self.assertEqual(result["armed"], 1)
                self.assertEqual(result["fault"], 0)
                self.assertLess(abs(result["v2"] - 5), .5 if mode == 1 else .02)
                if mode == 1:
                    self.assertIn(result["duty"], (0, 1))
                self.assertEqual(self.command("SET r1 600")[0]["type"], "error")
                self.command("STOP")
                self.assertEqual(self.advance(1)["duty"], 0)


if __name__ == "__main__":
    unittest.main()
