"""Regenerate the golden MatchCtrl/MatchFeedback byte vectors pinned in
test_matchctrl.cpp. Run from the repo root:

    python tests/host/golden_matchctrl.py <path-to-phoenix-server>

Every frame is produced by the AUTHORITATIVE encoder
(phoenix-server/phoenix/core/robot/wire.py); the C++ host tests assert
byte-for-byte equality against the hex printed here. If you change the
protocol, change wire.py first, regenerate, and update BOTH sides.
"""
import math
import sys

sys.path.insert(0, sys.argv[1] if len(sys.argv) > 1 else "../../../phoenix-server")

from phoenix.core.geom import Vec2
from phoenix.core.robot import wire
from phoenix.core.robot.wire import (
    DribbleTraction,
    KickerDevice,
    KickerDribbler,
    KickerMode,
    MatchFeedback,
)


def show(name: str, frame: bytes) -> None:
    print(f"{name} ({len(frame)}B): {frame.hex()}")


# --- MatchCtrl goldens -----------------------------------------------------

# 1. GLOBAL_POS, the workhorse: vision pose + delay + limits + KD.
kd = KickerDribbler(
    kick_mode=KickerMode.ARM,
    kick_device=KickerDevice.CHIP,
    kick_speed=6.5,
    dribbler_speed=3.0,
    dribbler_force=5.0,
)
skill = wire.encode_global_pos(
    Vec2(1.240, -0.760), 1.570,
    vel_max_xy=2.5, vel_max_w=10.0, acc_max_xy=3.0, acc_max_w=30.0,
    kd=kd,
)
show("global_pos", wire.encode_match_ctrl(7, 42, skill, (Vec2(1.234, -0.567), 0.789),
                                          pos_delay_us=12_500, cam_id=2, flags=0))

# 2. GLOBAL_POS with an explicit primary direction (pi/2).
skill = wire.encode_global_pos(Vec2(0.5, 0.5), -0.4, 2.0, 4.0, 2.0, 20.0,
                               primary_direction=math.pi / 2)
show("global_pos_primdir", wire.encode_match_ctrl(1, 7, skill, (Vec2(0.0, 0.0), 0.0),
                                                  pos_delay_us=5_000, cam_id=0))

# 3. FAST_POS with the raised fast-accel limit.
skill = wire.encode_fast_pos(Vec2(1.0, 1.0), 0.0, 3.0, 10.0, 3.0, 30.0,
                             acc_max_xy_fast=6.0)
show("fast_pos", wire.encode_match_ctrl(2, 8, skill, (Vec2(0.2, -0.2), 1.0),
                                        pos_delay_us=8_000, cam_id=1))

# 4. LOCAL_VEL (body-frame velocity, brake fallback / joystick).
skill = wire.encode_local_vel(Vec2(1.5, -0.25), 2.0, acc_max_xy=4.0, acc_max_w=20.0)
show("local_vel", wire.encode_match_ctrl(3, 9, skill, (Vec2(0.0, 0.0), 0.0),
                                         pos_delay_us=2_500, cam_id=0))

# 5. GLOBAL_VEL (world-frame velocity).
skill = wire.encode_global_vel(Vec2(-0.5, 0.75), -1.5, acc_max_xy=2.0, acc_max_w=10.0,
                               jerk_max_xy=50.0, jerk_max_w=500.0)
show("global_vel", wire.encode_match_ctrl(3, 10, skill, (Vec2(0.1, 0.1), 0.2),
                                          pos_delay_us=2_500, cam_id=0))

# 6. WHEEL_VEL (commissioning), mixed signs.
skill = wire.encode_wheel_vel((10.0, -10.0, 5.0, -5.0))
show("wheel_vel", wire.encode_match_ctrl(4, 11, skill, None, 0, 0))

# 7. SINE (system ID).
skill = wire.encode_sine(Vec2(0.5, 0.0), 1.0, freq_hz=2.5)
show("sine", wire.encode_match_ctrl(5, 12, skill, None, 0, 0))

# 8. EMERGENCY, no vision (UNUSED_FIELD sentinels + delay 255).
show("emergency_novision",
     wire.encode_match_ctrl(6, 13, wire.encode_emergency(), None, 0, 0))

# 9. Vision present but posDelay saturated beyond the none-marker.
show("emergency_delay_sat",
     wire.encode_match_ctrl(6, 14, wire.encode_emergency(), (Vec2(0.0, 0.0), 0.0),
                            pos_delay_us=1_000_000, cam_id=3))

# --- MatchFeedback golden ---------------------------------------------------

fb = MatchFeedback(
    robot_id=4,
    seq=99,
    pos=Vec2(-1.5, 0.25),
    heading=math.pi / 4,
    vel=Vec2(0.5, -0.125),
    ang_vel=1.5,
    kicker_level_v=180.0,
    kicker_max_v=200.0,
    dribbler_speed=4.0,
    dribble_traction=DribbleTraction.STRONG,
    battery_v=22.6,
    battery_percent=80.0,
    barrier_interrupted=True,
    dribbler_temp_class=1,
    kick_counter=True,
    ball_state=2,
    features=wire.FEATURE_MOVE | wire.FEATURE_KICK_STRAIGHT,
    hardware_id=11,
    ball_pos_age_ms=40,
    ball_pos=Vec2(0.09, 0.0),
)
show("feedback", wire.encode_match_feedback(fb))

# Same but with no onboard ball (age 255 -> ballPos zeroed).
fb_none = MatchFeedback(
    robot_id=4, seq=100, pos=Vec2(0.0, 0.0), heading=0.0, vel=Vec2(0.0, 0.0),
    ang_vel=0.0, kicker_level_v=0.0, kicker_max_v=200.0, dribbler_speed=0.0,
    dribble_traction=DribbleTraction.OFF, battery_v=24.0, battery_percent=100.0,
    barrier_interrupted=False, dribbler_temp_class=0, kick_counter=False,
    ball_state=0, features=wire.FEATURE_MOVE, hardware_id=11,
    ball_pos_age_ms=255, ball_pos=None,
)
show("feedback_noball", wire.encode_match_feedback(fb_none))


# --- MotionParams (0x07) + the feedback profile bytes ---------------------
show(
    "motion_params_full",
    wire.encode_motion_params(wire.MotionParams(
        robot_id=7, seq=42, profile_id=1011, rl_mode=3, reload_policy=True,
        params=((wire.MotionParamKey.BODY_LONGITUDINAL_VEL_MAX, 4.0),
                (wire.MotionParamKey.RL_CONFIDENCE_THRESHOLD, 0.3)),
    )),
)
show("motion_params_min", wire.encode_motion_params(wire.MotionParams(robot_id=5, seq=1, profile_id=1001)))
fb_profile = MatchFeedback(
    robot_id=3, seq=9, pos=Vec2(0.5, -0.25), heading=1.0, vel=Vec2(0.1, 0.0), ang_vel=0.0,
    kicker_level_v=0.0, kicker_max_v=200.0, dribbler_speed=0.0,
    dribble_traction=DribbleTraction.OFF, battery_v=15.2, battery_percent=76.5,
    barrier_interrupted=False, dribbler_temp_class=0, kick_counter=False, ball_state=0,
    features=0x0B, hardware_id=11, ball_pos_age_ms=255, ball_pos=None,
    motion_profile_id=1011, rl_mode=3, adaptive_enabled=True,
)
show("feedback_profile", wire.encode_match_feedback(fb_profile))
