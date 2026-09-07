"""Hardware-free telemetry contract, bounds and failure tests (standard library)."""

import json
import socket
import struct
import tempfile
import unittest
from dataclasses import replace
from itertools import pairwise
from pathlib import Path
from unittest.mock import patch

from diagnostics import ipc
from diagnostics.publisher import Budget, Publisher, bind_local, monotonic_us
from diagnostics.wire import MAX_PACKET, MOTOR_COLUMNS, Encoder


def fixture(producer=b"a" * 16, cycle=42, stamp=None):
    stamp = monotonic_us() if stamp is None else stamp
    header = ipc.HEADER.pack(b"PHI1", 1, producer, 2, 0, cycle, stamp, 5, 1)
    wheel = ipc.MOTOR.pack(3.5, 2.5, 2.25, 12, -1.5, 23.5, 40, 10, 0, 1, 1, 0)
    values = tuple([1.123456789] * 33)
    return header + wheel * 4 + ipc.TAIL.pack(*values, 129, 2, 1)


class Incoming:
    def __init__(self, packets=()):
        self.packets = list(packets)

    def recv(self, size):
        if not self.packets:
            raise BlockingIOError()
        return self.packets.pop(0)[:size]


class Outgoing:
    def __init__(self, fail=False):
        self.packets = []
        self.fail = fail

    def sendto(self, packet, target):
        if self.fail:
            raise OSError("injected network unavailable")
        self.packets.append(packet)
        return len(packet)


def publisher(packets=(), fail=False):
    return Publisher(
        Incoming(packets),
        Outgoing(fail),
        ("239.255.50.60", 50610),
        Encoder("pi-fixture"),
        {"motors": 50, "motion": 20, "health": 1, "identity": 0.2},
    )


class DiagnosticsTests(unittest.TestCase):
    def test_deadline_wait_wakes_before_next_snapshot_without_busy_idle(self):
        p = publisher()
        self.assertEqual(p.wait_timeout(1.0), 0.25)
        p.last = ipc.decode(fixture(stamp=1000000))
        p.next_due = {"motors": 1.02, "motion": 1.05, "health": 2.0, "identity": 6.0}
        self.assertAlmostEqual(p.wait_timeout(1.017), 0.003)
        self.assertEqual(p.wait_timeout(1.3), 0.25)

    def test_twenty_ms_publication_with_twelve_ms_snapshots_and_no_catchup(self):
        now = 100.0
        with (
            patch("diagnostics.publisher.time.monotonic", side_effect=lambda: now),
            patch(
                "diagnostics.publisher.monotonic_us", side_effect=lambda: int(now * 1e6)
            ),
        ):
            p = publisher()
            sample = ipc.decode(fixture(stamp=int(now * 1e6)))
            p.last = sample
            p.encoder.adopt(sample)
            p.budget.available = MAX_PACKET * 4
            next_snapshot = now + 0.012
            sent = []
            real_send = p.send

            def track_send(kind, record, data):
                if real_send(kind, record, data):
                    sent.append((kind, now))

            p.send = track_send
            for _ in range(500):
                p.publish_due()
                now = min(now + p.wait_timeout(now), next_snapshot)
                if now >= 101.0:
                    break
                if now >= next_snapshot:
                    p.last = replace(sample, acquired_us=int(now * 1e6))
                    next_snapshot = now + 0.012
            else:
                self.fail("scheduler spun instead of sleeping to a deadline")
            motors = [stamp for kind, stamp in sent if kind == "motors"]
            self.assertGreaterEqual(len(motors), 49)
            self.assertLessEqual(len(motors), 51)
            self.assertTrue(all(b - a >= 0.02 - 1e-10 for a, b in pairwise(motors)))
            # A stalled process resumes one packet per family, never a catch-up burst.
            now += 2
            p.last = replace(sample, acquired_us=int(now * 1e6))
            before = len(sent)
            p.publish_due()
            p.publish_due()
            self.assertLessEqual(len(sent) - before, 4)

    def test_events_do_not_burst_when_select_wakes_for_another_family(self):
        with (
            patch("diagnostics.publisher.time.monotonic", return_value=100.0),
            patch("diagnostics.publisher.monotonic_us", return_value=100000000),
        ):
            p = publisher()
            p.last = ipc.decode(fixture(stamp=100000000))
            p.encoder.adopt(p.last)
            p.next_due = {kind: 101.0 for kind in p.rates}
            p.events.extend([(p.last, {"event_id": "1"})] * 3)
            p.publish_due()
            p.publish_due()
            self.assertEqual(len(p.events), 2)
            self.assertAlmostEqual(p.wait_timeout(100.0), 0.01)

    def test_vision_unobserved_sentinel_is_null_but_old_valid_age_remains(self):
        sample = ipc.decode(fixture())
        values = list(sample.values)
        values[27] = 1e9
        unseen = Encoder.motion(replace(sample, values=tuple(values)))["vision"]
        self.assertFalse(unseen["available"])
        self.assertIsNone(unseen["age_s"])
        self.assertIsNone(unseen["innovation_m"])
        values[27] = 7200.5
        old = Encoder.motion(replace(sample, values=tuple(values)))["vision"]
        self.assertTrue(old["available"])
        self.assertFalse(old["alive"])
        self.assertEqual(old["age_s"], 7200.5)
        identity = Encoder("test").identity({"motors": 50})
        self.assertEqual(identity["rate_caps_hz"], {"motors": 50})
        self.assertNotIn("rates_hz", identity)

    def test_private_abi_and_missing_values(self):
        self.assertEqual(ipc.PACKET_BYTES, 668)
        sample = ipc.decode(fixture())
        self.assertEqual(
            sample.wheels[0],
            [1, 3.5, 2.5, 2.25, 12, -1.5, 23.5, 40, 10, 0, True, 0, True],
        )
        raw = bytearray(fixture())
        struct.pack_into("<I", raw, ipc.HEADER.size + 64, 0)  # replied=false
        struct.pack_into("<I", raw, ipc.HEADER.size + 68, 0)  # energized=false
        wheel = ipc.decode(raw).wheels[0]
        self.assertEqual(wheel[2:10], [None] * 8)
        self.assertFalse(wheel[10])
        self.assertIsNone(wheel[11])

    def test_reject_malformed_ipc(self):
        for raw in (b"", fixture()[:-1], fixture() + b"x", b"BAD!" + fixture()[4:]):
            with self.assertRaises(ValueError):
                ipc.decode(raw)

    def test_cpp_golden_if_supplied(self):
        path = Path("tests/fixtures/diagnostics-v1.bin")
        if not path.exists():
            self.skipTest("run hardware-free C++ fixture generator first")
        sample = ipc.decode(path.read_bytes())
        self.assertEqual(
            (sample.producer_id, sample.robot_id, sample.cycle),
            ("ab" + "00" * 15, 2, 42),
        )
        self.assertEqual(sample.wheels[0][1:8], [3.5, 2.5, 2.25, 12, -1.5, 23.5, 40])
        self.assertFalse(sample.wheels[1][10])
        self.assertEqual(sample.wheels[1][1], 4.5)
        self.assertIsNone(sample.wheels[1][2])
        self.assertFalse(sample.wheels[1][12])
        self.assertIsNone(sample.wheels[1][3])
        self.assertEqual((sample.kick_requested, sample.kick_sent), (2, 1))

    def test_wire_shape_precision_and_all_families_fit(self):
        sample = ipc.decode(fixture())
        p = publisher()
        data = {
            "identity": p.encoder.identity(p.rates),
            "motors": {"wheels": sample.wheels},
            "motion": p.encoder.motion(sample),
            "health": p.health(sample),
            "event": {
                "event_id": "1",
                "code": "motor.reply_missing",
                "severity": "warning",
                "subsystem": "motor_1",
            },
        }
        for kind, payload in data.items():
            raw = p.encoder.packet(kind, sample, sample.acquired_us + 100, payload)
            self.assertLessEqual(len(raw), MAX_PACKET)
            packet = json.loads(raw)
            self.assertEqual(packet["snapshot_age_us"], 100)
            self.assertEqual(packet["cycle"], "42")
            self.assertEqual(packet["seq"], 0)
        self.assertEqual(data["identity"]["motor_columns"], MOTOR_COLUMNS)
        self.assertIsNone(data["motion"]["command_applied_seq"])

    def test_packet_sequence_wrap_and_restart(self):
        sample = ipc.decode(fixture())
        e = Encoder("pi-fixture")
        first = json.loads(e.packet("motors", sample, sample.acquired_us, {}))
        e.sequences["motors"] = 0xFFFFFFFF
        self.assertEqual(
            json.loads(e.packet("motors", sample, sample.acquired_us, {}))["seq"],
            0xFFFFFFFF,
        )
        self.assertEqual(
            json.loads(e.packet("motors", sample, sample.acquired_us, {}))["seq"], 0
        )
        restarted = ipc.decode(fixture(producer=b"b" * 16))
        second = json.loads(e.packet("motors", restarted, restarted.acquired_us, {}))
        self.assertNotEqual(first["session_id"], second["session_id"])
        self.assertEqual(second["seq"], 0)

    def test_consumer_restart_and_retired_producer(self):
        p = publisher(
            [
                fixture(),
                fixture(cycle=41),
                fixture(producer=b"b" * 16, cycle=1),
                fixture(cycle=43),
            ]
        )
        p.receive()
        self.assertEqual(p.last.producer_id, (b"b" * 16).hex())
        self.assertEqual(p.counts["ipc_late"], 2)
        self.assertNotEqual(p.encoder.session_id, publisher().encoder.session_id)

    def test_network_failure_and_budget_remain_bounded(self):
        p = publisher([fixture()], fail=True)
        p.receive()
        self.assertFalse(p.send("motors", p.last, {"wheels": p.last.wheels}))
        self.assertEqual(p.counts["udp_send_errors"], 1)
        budget = Budget(1200, 0)
        self.assertTrue(budget.allow(1200, 0))
        self.assertFalse(budget.allow(1, 0))
        self.assertTrue(budget.allow(1200, 1))

    def test_stale_and_invalid_flood_has_fixed_work_limit(self):
        p = publisher([fixture(stamp=1)] + [b"bad"] * 1000)
        p.receive()
        self.assertIsNone(p.last)
        self.assertEqual(p.counts["ipc_received"], 256)
        self.assertEqual(p.counts["stale_dropped"], 1)

    def test_event_fault_vs_limiting_and_missing_reply(self):
        e = Encoder("pi-fixture")
        sample = ipc.decode(fixture())
        e.adopt(sample)
        self.assertEqual(e.events(sample), [])
        sample.wheels[0][9] = 104
        self.assertEqual(e.events(sample)[0]["code"], "motor.limiting_changed")
        sample.wheels[0][9] = 200
        self.assertEqual(e.events(sample)[0]["code"], "motor.unknown_code")
        sample.wheels[0][10] = False
        self.assertEqual(e.events(sample)[0]["code"], "motor.reply_missing")

    def test_device_identity_and_honest_motion_labels(self):
        for identity in ("", "robot with spaces", "x" * 65, "robot/2"):
            with self.assertRaises(ValueError):
                Encoder(identity)
        data = Encoder.motion(ipc.decode(fixture()))
        self.assertIn("controller_output_body", data)
        self.assertNotIn("safety_output_body", data)
        self.assertIn("elapsed_before_telemetry_s", data)
        self.assertNotIn("control_elapsed_s", data)

    @unittest.skipUnless(
        hasattr(socket, "AF_UNIX") and hasattr(__import__("os"), "geteuid"),
        "Linux private Unix socket permissions",
    )
    def test_local_bind_does_not_replace_active_socket(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "snapshot.sock"
            sock = bind_local(path)
            try:
                with self.assertRaises(ValueError):
                    bind_local(path)
            finally:
                sock.close()
            recovered = bind_local(path)
            recovered.close()
            path.unlink()


if __name__ == "__main__":
    unittest.main()
