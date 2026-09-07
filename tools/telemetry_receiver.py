#!/usr/bin/env python3
"""telemetry_receiver — operator-side sink for `debugger commission --stream-to`.

Listens on a UDP port, appends EVERY datagram to a local NDJSON file exactly
as received (the robot's own log stays authoritative; this is the operator's
copy), and prints a compact live summary.

Standard library only: it runs on the operator's Windows laptop with a stock
Python 3.

    python tools/telemetry_receiver.py --port 50600 --out session.ndjson

The robot side:

    sudo ./debugger commission --stream-to <laptop ip>:50600

Sequence handling: every record carries `seq`. A record that arrives with a
seq beyond the next expected one opens a gap (each missing seq is counted);
a later record that fills a gap is counted as out-of-order and closes it.
Both counts are printed live and at exit. Records that are not valid JSON are
still written to the file and counted as `bad`.
"""
from __future__ import annotations

import argparse
import datetime as _dt
import json
import socket
import sys
import time


def _utc_stamp() -> str:
    return _dt.datetime.now(_dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def _num(v, fmt="{:.2f}") -> str:
    if v is None:
        return "--"
    try:
        return fmt.format(v)
    except (TypeError, ValueError):
        return "--"


class SeqTracker:
    """Gap/out-of-order accounting over a per-session sequence number."""

    def __init__(self, remember: int = 20000):
        self.remember = remember
        self.reset()

    def reset(self):
        self.max_seq = None
        self.missing = set()
        self.gaps_opened = 0      # total seqs ever counted missing
        self.out_of_order = 0     # late arrivals that filled a gap
        self.duplicates = 0
        self.received = 0

    def observe(self, seq: int):
        self.received += 1
        if self.max_seq is None:
            self.max_seq = seq
            return
        if seq > self.max_seq:
            if seq > self.max_seq + 1:
                for s in range(self.max_seq + 1, seq):
                    self.missing.add(s)
                    self.gaps_opened += 1
            self.max_seq = seq
        elif seq in self.missing:
            self.missing.discard(seq)
            self.out_of_order += 1
        else:
            self.duplicates += 1
        if len(self.missing) > self.remember:
            # Bound memory on a long, lossy session; the count stays honest.
            for s in sorted(self.missing)[: len(self.missing) - self.remember]:
                self.missing.discard(s)

    @property
    def still_missing(self) -> int:
        return len(self.missing)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--port", type=int, default=50600, help="UDP port to listen on (default 50600)")
    ap.add_argument("--bind", default="0.0.0.0", help="address to bind (default all)")
    ap.add_argument("--out", default=None,
                    help="NDJSON file to append to (default commission_<utc>.ndjson)")
    ap.add_argument("--quiet", action="store_true", help="no live summary line")
    ap.add_argument("--duration", type=float, default=0.0,
                    help="stop after this many seconds (default: run until Ctrl-C)")
    args = ap.parse_args()

    out_path = args.out or f"commission_{_utc_stamp()}.ndjson"
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
    except OSError:
        pass
    sock.bind((args.bind, args.port))
    sock.settimeout(0.25)
    print(f"listening on udp {args.bind}:{args.port}, appending to {out_path}  (Ctrl-C to stop)")

    tracker = SeqTracker()
    bad = 0
    events = 0
    last_cycle = None
    last_header = None
    last_print = 0.0
    session_tag = None
    t_first = time.monotonic()

    def summary() -> str:
        parts = [f"rx {tracker.received}", f"gaps {tracker.still_missing}",
                 f"(opened {tracker.gaps_opened}, filled late {tracker.out_of_order})",
                 f"dup {tracker.duplicates}", f"bad {bad}", f"events {events}"]
        c = last_cycle
        if c:
            tw = c.get("twist_applied") or [None, None, None]
            st = c.get("settings") or {}
            wheels = c.get("wheels") or []
            wv = " ".join(_num(w.get("vel_rev_s")) if w.get("replied") else "NO" for w in wheels)
            wq = " ".join(_num(w.get("q_current_a"), "{:.1f}") if w.get("replied") else "--" for w in wheels)
            faults = [w.get("fault") for w in wheels if w.get("replied") and w.get("fault")]
            parts.append(f"| t {_num(c.get('t_mono_s'), '{:.1f}')}s {c.get('state','?')}"
                         f"{' HOLD' if c.get('hold') else ''}"
                         f"{' clip=' + c['clip'] if c.get('clip') else ''}")
            parts.append(f"tw({_num(tw[0])},{_num(tw[1])},{_num(tw[2])})")
            parts.append(f"vel[{wv}] q[{wq}]")
            parts.append(f"lim v{_num(st.get('velocity_limit_rev_s'))}/a{_num(st.get('accel_limit_rev_s2'))}"
                         f" body{_num(st.get('body_vel_mps'))}")
            if faults:
                parts.append(f"FAULTS {faults}")
        return "  ".join(parts)

    try:
        with open(out_path, "a", encoding="utf-8") as out:
            while True:
                try:
                    data, addr = sock.recvfrom(65535)
                except socket.timeout:
                    data = None
                except KeyboardInterrupt:
                    raise
                if data is not None:
                    text = data.decode("utf-8", errors="replace").rstrip("\r\n")
                    out.write(text + "\n")
                    out.flush()  # the operator's copy must survive a crash of this script
                    try:
                        rec = json.loads(text)
                    except ValueError:
                        bad += 1
                        rec = None
                    if isinstance(rec, dict):
                        kind = rec.get("type")
                        if kind == "event" and rec.get("event") == "header":
                            tag = (rec.get("pi_serial"), rec.get("log_path"))
                            if tag != session_tag:
                                session_tag = tag
                                tracker.reset()
                                last_header = rec
                                print(f"\n=== new session from {addr[0]}: pi {rec.get('pi_serial')} "
                                      f"robot log {rec.get('log_path')}")
                                ce = rec.get("ceiling") or {}
                                print(f"    ceiling velocity_limit {_num(ce.get('velocity_limit_rev_s'))} rev/s  "
                                      f"accel_limit {_num(ce.get('accel_limit_rev_s2'))} rev/s^2  "
                                      f"body {_num(ce.get('body_vel_mps'))} m/s / "
                                      f"{_num(ce.get('body_omega_radps'))} rad/s")
                                for c in rec.get("controllers") or []:
                                    print(f"    controller {c.get('id')}: serial {c.get('serial')} "
                                          f"fw {str(c.get('git_hash') or '?')[:8]}"
                                          f"{' DIRTY' if c.get('git_dirty') else ''}")
                                for m in rec.get("prior_profile_mismatches") or []:
                                    print(f"    WARNING drivetrain changed: {m}")
                        seq = rec.get("seq")
                        if isinstance(seq, int) and kind == "cycle":
                            tracker.observe(seq)
                            last_cycle = rec
                        elif kind == "event":
                            events += 1
                            if rec.get("event") != "header":
                                extras = {k: v for k, v in rec.items()
                                          if k not in ("type", "seq", "t_mono_s", "t_wall", "t_wall_unix_s", "event")}
                                print(f"\n[{rec.get('t_wall','')}] seq {seq} {rec.get('event')}: "
                                      f"{json.dumps(extras, ensure_ascii=False)}")
                now = time.monotonic()
                if args.duration > 0 and now - t_first >= args.duration:
                    break
                if not args.quiet and now - last_print >= 0.5:
                    last_print = now
                    line = summary()
                    sys.stdout.write("\r\x1b[K" + line[:220])
                    sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
        elapsed = time.monotonic() - t_first
        print(f"\n\nstopped after {elapsed:.0f}s: {summary()}")
        print(f"wrote {out_path}")
        if last_header:
            print(f"robot-side authoritative log: {last_header.get('log_path')}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
