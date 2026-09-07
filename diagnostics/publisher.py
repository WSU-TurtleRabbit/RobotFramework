"""Separate telemetry process. Receives local snapshots; never opens CAN/control ports."""

import argparse
import errno
import ipaddress
import json
import os
import select
import signal
import socket
import stat
import sys
import time
from collections import deque
from pathlib import Path

from .ipc import PACKET_BYTES, decode
from .wire import MAX_PACKET, Encoder


def monotonic_us():
    return time.monotonic_ns() // 1000


def device_identity():
    path = Path("/sys/firmware/devicetree/base/serial-number")
    try:
        serial = path.read_text().strip("\x00\r\n ")
    except OSError:
        serial = ""
    if not serial:
        raise ValueError("Pi serial unavailable; supply --device-id")
    return "pi-" + serial


def bind_local(path):
    path = Path(path)
    if not path.is_absolute() or len(os.fsencode(path)) >= 108:
        raise ValueError("local socket needs an absolute path shorter than 108 bytes")
    path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    parent = path.parent.lstat()
    if (
        not stat.S_ISDIR(parent.st_mode)
        or parent.st_uid != os.geteuid()
        or parent.st_mode & 0o022
    ):
        raise ValueError(
            "socket parent must be an owned directory without group/world writes"
        )
    if path.exists() or path.is_symlink():
        entry = path.lstat()
        if not stat.S_ISSOCK(entry.st_mode) or entry.st_uid != os.geteuid():
            raise ValueError(
                "refusing to replace a non-socket or another user's socket"
            )
        probe = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        try:
            probe.connect(str(path))  # address check only; never sends data
        except OSError as exc:
            if exc.errno != errno.ECONNREFUSED:
                raise
            path.unlink()  # owned, stale socket only
        else:
            raise ValueError(
                "another telemetry publisher already owns the local socket"
            )
        finally:
            probe.close()
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    try:
        sock.setblocking(False)
        sock.bind(str(path))
        path.chmod(0o600)
        return sock
    except Exception:
        sock.close()
        raise


class Budget:
    def __init__(self, bytes_per_second, now):
        self.rate = bytes_per_second
        self.available = float(MAX_PACKET)
        self.last = now

    def allow(self, size, now):
        elapsed = max(0.0, now - self.last)
        self.available = min(
            float(MAX_PACKET * 4), self.available + elapsed * self.rate
        )
        self.last = now
        if size > self.available:
            return False
        self.available -= size
        return True


class Publisher:
    def __init__(self, incoming, outgoing, target, encoder, rates, budget=65536):
        self.incoming, self.outgoing, self.target = incoming, outgoing, target
        self.encoder, self.rates = encoder, rates
        self.budget = Budget(budget, time.monotonic())
        self.last = None
        self.next_due = {}
        self.next_event_due = 0.0
        self.retired = deque(maxlen=8)
        self.events = deque(maxlen=32)
        self.counts = {
            "ipc_received": 0,
            "ipc_invalid": 0,
            "ipc_late": 0,
            "ipc_coalesced": 0,
            "udp_sent": 0,
            "udp_send_errors": 0,
            "oversize_dropped": 0,
            "budget_dropped": 0,
            "events_dropped": 0,
            "stale_dropped": 0,
        }
        self.running = True

    def receive(self):
        newest = None
        for _ in range(256):
            try:
                raw = self.incoming.recv(PACKET_BYTES + 1)
            except BlockingIOError:
                break
            self.counts["ipc_received"] += 1
            try:
                sample = decode(raw)
            except ValueError:
                self.counts["ipc_invalid"] += 1
                continue
            now = monotonic_us()
            if sample.acquired_us > now or now - sample.acquired_us > 250000:
                self.counts["stale_dropped"] += 1
                continue
            if sample.producer_id in self.retired:
                self.counts["ipc_late"] += 1
                continue
            if (
                self.last
                and sample.producer_id == self.last.producer_id
                and (
                    sample.cycle <= self.last.cycle
                    or sample.acquired_us < self.last.acquired_us
                )
            ):
                self.counts["ipc_late"] += 1
                continue
            if self.encoder.adopt(sample):
                if self.last:
                    self.retired.append(self.last.producer_id)
                self.counts["events_dropped"] += len(self.events)
                self.events.clear()
                self.next_due.clear()
                self.next_event_due = 0.0
            for event in self.encoder.events(sample):
                if len(self.events) == self.events.maxlen:
                    self.counts["events_dropped"] += 1
                else:
                    self.events.append((sample, event))
            if newest is not None:
                self.counts["ipc_coalesced"] += 1
            newest = self.last = sample

    def send(self, kind, sample, data):
        try:
            packet = self.encoder.packet(kind, sample, monotonic_us(), data)
        except (ValueError, OverflowError):
            self.counts["oversize_dropped"] += 1
            return False
        if not self.budget.allow(len(packet), time.monotonic()):
            self.counts["budget_dropped"] += 1
            return False
        try:
            size = self.outgoing.sendto(packet, self.target)
            if size != len(packet):
                raise OSError("partial datagram send")
        except OSError:
            self.counts["udp_send_errors"] += 1
            return False
        self.counts["udp_sent"] += 1
        return True

    def health(self, sample):
        bridge_active = bool(sample.flags & 128)
        return {
            **self.counts,
            "snapshot_offered": sample.offered,
            "snapshot_ipc_dropped": sample.dropped,
            "supervisor_estop": bool(sample.flags & 8),
            "control_deadline_missed": bool(sample.flags & 16),
            "arduino_connected": bool(sample.flags & 32),
            "dribbler_commanded_active": bool(sample.flags & 64)
            if bridge_active
            else None,
            "kick_requested_count": sample.kick_requested if bridge_active else None,
            "kick_bytes_sent_count": sample.kick_sent if bridge_active else None,
            "actuator_counter_scope": "MatchCtrl",
            "physical_kick_confirmation": None,
            "events_between_snapshots_observable": False,
        }

    def publish_due(self):
        sample = self.last
        now = time.monotonic()
        if sample is None or monotonic_us() - sample.acquired_us > 250000:
            return
        # Metadata/health keep their reserved priority before ordinary traces.
        for kind in ("identity", "health", "motors", "motion"):
            if now < self.next_due.get(kind, 0):
                continue
            self.next_due[kind] = now + 1.0 / self.rates[kind]
            if kind == "identity":
                data = self.encoder.identity(self.rates)
            elif kind == "health":
                data = self.health(sample)
            elif kind == "motors":
                data = {"wheels": sample.wheels}
            else:
                data = self.encoder.motion(sample)
            self.send(kind, sample, data)
        # A deadline-aware select may wake early for traces; keep events capped.
        if self.events and now >= self.next_event_due:
            self.next_event_due = now + 0.01
            event_sample, event = self.events.popleft()
            if not self.send("event", event_sample, event):
                self.counts["events_dropped"] += 1

    def wait_timeout(self, now):
        # With no fresh producer there are no publication deadlines to chase.
        if self.last is None or now - self.last.acquired_us / 1e6 >= 0.25:
            return 0.25
        deadlines = [self.next_due.get(kind, now) for kind in self.rates]
        if self.events:
            deadlines.append(self.next_event_due)
        return min(0.25, max(0.0, min(deadlines) - now))

    def run(self):
        while self.running:
            select.select([self.incoming], [], [], self.wait_timeout(time.monotonic()))
            self.receive()
            self.publish_due()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--socket", required=True, help="private local snapshot socket path"
    )
    parser.add_argument(
        "--interface", required=True, help="robot's outbound IPv4 interface address"
    )
    parser.add_argument("--group", default="239.255.50.60")
    parser.add_argument("--port", type=int, default=50610)
    parser.add_argument("--device-id")
    parser.add_argument("--motors-hz", type=float, default=50)
    parser.add_argument("--motion-hz", type=float, default=20)
    parser.add_argument("--budget-bytes", type=int, default=65536)
    args = parser.parse_args(argv)
    incoming = outgoing = None
    try:
        if not ipaddress.IPv4Address(args.group).is_multicast:
            raise ValueError("group must be IPv4 multicast")
        interface = ipaddress.IPv4Address(args.interface)
        if interface.is_unspecified or interface.is_multicast:
            raise ValueError("interface must be an explicit local IPv4 address")
        if not 1 <= args.port <= 65535:
            raise ValueError("port must be 1..65535")
        if not 0 < args.motors_hz <= 50 or not 0 < args.motion_hz <= 20:
            raise ValueError("rates must be positive, motors<=50 Hz and motion<=20 Hz")
        if not 1200 <= args.budget_bytes <= 65536:
            raise ValueError("budget must be 1200..65536 bytes/s")
        identity = args.device_id or device_identity()
        encoder = Encoder(identity)
        incoming = bind_local(args.socket)
        outgoing = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        outgoing.setblocking(False)
        outgoing.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 1)
        outgoing.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF, interface.packed)
        rates = {
            "motors": args.motors_hz,
            "motion": args.motion_hz,
            "health": 1,
            "identity": 0.2,
        }
        publisher = Publisher(
            incoming,
            outgoing,
            (args.group, args.port),
            encoder,
            rates,
            args.budget_bytes,
        )

        def stop(_signum, _frame):
            publisher.running = False

        signal.signal(signal.SIGINT, stop)
        signal.signal(signal.SIGTERM, stop)
        print(
            json.dumps(
                {
                    "status": "waiting_for_snapshots",
                    "device_id": identity,
                    "group": args.group,
                    "port": args.port,
                }
            ),
            flush=True,
        )
        publisher.run()
        return 0
    except (OSError, ValueError) as exc:
        print(f"Telemetry publisher stopped: {exc}", file=sys.stderr)
        return 1
    finally:
        if outgoing is not None:
            outgoing.close()
        if incoming is not None:
            incoming.close()
            Path(args.socket).unlink(missing_ok=True)


if __name__ == "__main__":
    raise SystemExit(main())
