"""Live interop check: the REAL phoenix-server (production ServerLoop +
encoders) drives the REAL firmware modules — compiled as sim_robot.exe from
this repo — over loopback UDP. This is the decisive end-to-end proof before
hardware: every byte the server sends is decoded by the firmware, and every
byte back decodes server-side.

Run (from the RobotFramework repo root, after building sim_robot):
    python tests/host/interop_e2e.py [path-to-phoenix-server]

Asserts (mirroring the server's own e2e):
  (a) MoveTo converges — sim robot reaches the pose through the full
      firmware cascade (wire -> skills -> estimator -> trajectory -> control);
  (b) KickBall — approach, ARM, camera-contact kick fires, ball to aim;
  (c) STOP — sim speed never exceeds the 1.0 m/s cap;
  (d) HALT — EMERGENCY reaches the sim and it ramps to a stop.
"""
from __future__ import annotations

import pathlib
import socket
import subprocess
import sys
import threading
import time

REPO = pathlib.Path(__file__).resolve().parents[2]
PHOENIX = pathlib.Path(sys.argv[1]) if len(sys.argv) > 1 else REPO.parent / "phoenix-server"
sys.path.insert(0, str(PHOENIX))

from phoenix.api import Api  # noqa: E402
from phoenix.core.geom import Vec2, angle_diff  # noqa: E402
from phoenix.core.limits import load_limits, load_robots, load_server  # noqa: E402
from phoenix.core.loop import ServerLoop  # noqa: E402
from phoenix.core.referee_fsm import Phase  # noqa: E402
from phoenix.core.ssl.referee import RefCommand  # noqa: E402
from phoenix.core.udp import UdpSocket  # noqa: E402
from phoenix.skills import SkillState  # noqa: E402
from tests.test_vision_referee_decode import make_detection_frame, make_referee  # noqa: E402

SIM_EXE = REPO / "tests" / "host" / "build" / "sim_robot.exe"
SIM_PORT = 51514
ROBOT_ID = 0
VISION_HZ = 60.0


def wait_until(cond, timeout_s: float, poll_s: float = 0.02) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if cond():
            return True
        time.sleep(poll_s)
    return False


class WireLog:
    """SKILL lines the sim prints on stderr (what the server actually sent)."""

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.lines: list[str] = []
        self.mark = 0

    def add(self, line: str) -> None:
        with self.lock:
            self.lines.append(line)

    def reset(self) -> None:
        with self.lock:
            self.mark = len(self.lines)

    def since_mark(self) -> list[str]:
        with self.lock:
            return list(self.lines[self.mark:])


class SimTruth:
    """Latest TRUTH line from the sim's stdout."""

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.pose = (0.0, 0.0, 0.0)
        self.ball = (3.0, 3.0, 0.0, 0.0)

    def set(self, vals) -> None:
        with self.lock:
            self.pose = vals[0:3]
            self.ball = vals[3:7]

    def snapshot(self) -> dict:
        with self.lock:
            x, y, th = self.pose
            bx, by, bvx, bvy = self.ball
        return {
            "pos": Vec2(x, y),
            "heading": th,
            "ball_pos": Vec2(bx, by),
            "ball_vel": Vec2(bvx, bvy),
        }


def stdout_reader(proc: subprocess.Popen, truth: SimTruth) -> None:
    for line in proc.stdout:
        parts = line.split()
        if len(parts) == 8 and parts[0] == "TRUTH":
            truth.set(tuple(float(v) for v in parts[1:]))


def stderr_reader(proc: subprocess.Popen, wire: WireLog) -> None:
    for line in proc.stderr:
        if line.startswith("SKILL"):
            wire.add(line.strip())


class Feeder(threading.Thread):
    """Synthesizes SSL vision (60 Hz) + referee (20 Hz) from sim truth."""

    def __init__(self, truth: SimTruth, vision_addr, referee_addr) -> None:
        super().__init__(daemon=True, name="feeder")
        self.truth = truth
        self.vision_addr = vision_addr
        self.referee_addr = referee_addr
        self.command = RefCommand.FORCE_START
        self._stop_evt = threading.Event()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def run(self) -> None:
        n = 0
        period = 1.0 / VISION_HZ
        next_t = time.monotonic()
        while not self._stop_evt.is_set():
            snap = self.truth.snapshot()
            t_wall = time.time()
            pkt = make_detection_frame(
                camera_id=0,
                frame_number=n,
                t_capture=t_wall - 0.010,
                t_sent=t_wall,
                balls=((1.0, snap["ball_pos"].x * 1000.0, snap["ball_pos"].y * 1000.0),),
                yellow=(
                    (ROBOT_ID, snap["pos"].x * 1000.0, snap["pos"].y * 1000.0,
                     snap["heading"]),
                ),
            )
            try:
                self.sock.sendto(pkt, self.vision_addr)
                if n % 3 == 0:
                    self.sock.sendto(
                        make_referee(command=self.command, blue_positive=False),
                        self.referee_addr,
                    )
            except OSError:
                pass
            n += 1
            next_t += period
            delay = next_t - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_t = time.monotonic()

    def stop(self) -> None:
        self._stop_evt.set()
        self.join(timeout=2.0)
        self.sock.close()


def main() -> int:
    if not SIM_EXE.exists():
        print(f"sim_robot.exe not built: {SIM_EXE}")
        return 2
    tmp = REPO / "tests" / "host" / "build" / "interop"
    tmp.mkdir(parents=True, exist_ok=True)
    (tmp / "robots.toml").write_text(
        f'[robot.{ROBOT_ID}]\nip = "127.0.0.1"\ncommand_port = {SIM_PORT}\n',
        encoding="utf-8",
    )
    (tmp / "server.toml").write_text(
        '[server]\nteam_color = "yellow"\nfeedback_bind_port = 0\nloop_hz = 100\n',
        encoding="utf-8",
    )
    (tmp / "limits.toml").write_text(
        (PHOENIX / "configs" / "limits.toml").read_text(encoding="utf-8"), encoding="utf-8"
    )

    sim = subprocess.Popen(
        [str(SIM_EXE), str(SIM_PORT), str(ROBOT_ID)],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        cwd=REPO,
    )
    truth = SimTruth()
    wire = WireLog()
    reader = threading.Thread(target=stdout_reader, args=(sim, truth), daemon=True)
    reader.start()
    err_reader = threading.Thread(target=stderr_reader, args=(sim, wire), daemon=True)
    err_reader.start()

    failures = []
    try:
        server = load_server(tmp / "server.toml")
        limits = load_limits(tmp / "limits.toml")
        robots = load_robots(tmp / "robots.toml")
        vision = UdpSocket.bind(0, "127.0.0.1")
        referee = UdpSocket.bind(0, "127.0.0.1")
        api = Api()
        loop = ServerLoop(server, limits, robots, api=api,
                          vision_sock=vision, referee_sock=referee)
        feeder = Feeder(truth, ("127.0.0.1", vision.local_port),
                        ("127.0.0.1", referee.local_port))
        feeder.start()
        thread = threading.Thread(target=loop.run, daemon=True, name="server-loop")
        thread.start()

        def phase_of():
            view = api.latest_view()
            return view.game.state.phase if view else None

        handle = api.robot(ROBOT_ID)

        if not wait_until(lambda: handle.online() and handle.visible(), 5.0):
            failures.append("link never came up (robot not online/visible)")
        if not wait_until(lambda: phase_of() is Phase.RUNNING, 2.0):
            failures.append("referee never reached RUNNING")

        # ------------------------------------------- (a) MoveTo converges
        target, target_heading = Vec2(1.0, -0.5), 1.0
        handle.move_to(target, target_heading)
        ok = wait_until(lambda: handle.skill_state() is SkillState.SUCCESS, 10.0)
        snap = truth.snapshot()
        pos_err = snap["pos"].distance_to(target)
        heading_err = abs(angle_diff(target_heading, snap["heading"]))
        print(f"[interop] MoveTo: success={ok} pos_err={pos_err * 1000:.1f} mm "
              f"heading_err={heading_err:.3f} rad")
        if not ok:
            failures.append("MoveTo never reported SUCCESS")
        if pos_err >= 0.05:
            failures.append(f"MoveTo pos_err {pos_err * 1000:.1f} mm >= 50 mm")
        if heading_err >= 0.10:
            failures.append(f"MoveTo heading_err {heading_err:.3f} rad >= 0.10")

        # ------------------------------------------- (b) KickBall fires
        sim_cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sim_cmd.sendto(b"SIM BALL 1.8 -0.5", ("127.0.0.1", SIM_PORT))
        aim = Vec2(4.0, -0.5)
        handle.kick_at(aim)
        kicked = wait_until(
            lambda: truth.snapshot()["ball_vel"].dot(
                (aim - truth.snapshot()["ball_pos"]).normalized()) > 1.0,
            15.0,
        )
        snap = truth.snapshot()
        to_aim = (aim - snap["ball_pos"]).normalized()
        ball_speed = snap["ball_vel"].dot(to_aim)
        print(f"[interop] kick: fired={kicked} ball {ball_speed:.2f} m/s toward aim")
        if not kicked:
            failures.append("kick never launched the ball toward the aim point")
        if not wait_until(lambda: handle.skill_state() is SkillState.SUCCESS, 3.0):
            failures.append("KickBall never reported SUCCESS")

        # ------------------------------------------- (c) STOP speed cap
        feeder.command = RefCommand.STOP
        wait_until(lambda: phase_of() is Phase.STOPPED, 2.0)
        wire.reset()
        handle.move_to(Vec2(-1.0, 0.0))
        # The authoritative check is on the WIRE: every GLOBAL_POS frame the
        # server sends during STOP must carry vel_max <= 1.0 m/s.
        time.sleep(2.0)
        stop_frames = [l for l in wire.since_mark() if l.startswith("SKILL 4")]
        stop_vels = [float(l.split()[-1]) for l in stop_frames]
        over = [v for v in stop_vels if v > 1.0 + 1e-6]
        print(f"[interop] STOP: {len(stop_vels)} GLOBAL_POS frames, "
              f"max wire vel_max {max(stop_vels) if stop_vels else 0.0:.3f} m/s")
        if not stop_frames:
            failures.append("no GLOBAL_POS frames observed during STOP")
        elif over:
            failures.append(f"STOP cap violated on the wire: {max(over):.3f} m/s")
        # Physical check after the brake transient settled: sim speed <= cap.
        time.sleep(1.0)
        v = _sim_speed(truth)
        print(f"[interop] STOP: settled sim speed {v:.2f} m/s")
        if v > 1.05:
            failures.append(f"STOP settled sim speed {v:.2f} m/s > 1.05")

        # ------------------------------------------- (d) HALT -> EMERGENCY
        feeder.command = RefCommand.HALT
        time.sleep(0.3)  # let a few frames flow
        # After HALT the server sends EMERGENCY: the sim must ramp to a stop.
        stopped = wait_until(
            lambda: truth.snapshot()["pos"].distance_to(Vec2(-1.0, 0.0)) > 0.0
            and _sim_speed(truth) < 0.05,
            3.0,
        )
        print(f"[interop] HALT: sim stopped={stopped}")
        if not stopped:
            failures.append("HALT did not stop the sim (EMERGENCY never landed)")
    finally:
        try:
            loop.stop()
            thread.join(timeout=5.0)
            feeder.stop()
            loop.close()
        except Exception:
            pass
        sim.terminate()
        try:
            sim.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            sim.kill()

    if failures:
        print("\nINTEROP FAILED:")
        for f in failures:
            print(f"  - {f}")
        return 1
    print("\nINTEROP OK: the real server drives the real firmware end to end.")
    return 0


def _sim_speed(truth: SimTruth) -> float:
    p1 = truth.snapshot()["pos"]
    time.sleep(0.05)
    p2 = truth.snapshot()["pos"]
    return p1.distance_to(p2) / 0.05


if __name__ == "__main__":
    sys.exit(main())
