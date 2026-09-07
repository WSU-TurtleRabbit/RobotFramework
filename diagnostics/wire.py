"""Telemetry v1 JSON. No sockets, clocks, filesystem reads or control actions."""

import json
import re
import uuid

MAX_PACKET = 1200
MOTOR_COLUMNS = [
    "id",
    "requested_rev_s",
    "sent_rev_s",
    "vel_rev_s",
    "pos_rev",
    "q_current_a",
    "voltage_v",
    "temp_c",
    "mode",
    "fault",
    "replied",
    "age_us",
    "energized",
]
WHEEL_ORDER = {"1": "FR", "2": "RR", "3": "RL", "4": "FL"}


class Encoder:
    def __init__(self, device_id, firmware=None):
        if not isinstance(device_id, str) or not re.fullmatch(
            r"[A-Za-z0-9_.:-]{1,64}", device_id
        ):
            raise ValueError(
                "device_id must use 1..64 ASCII letters, numbers, dot, colon, dash or underscore"
            )
        self.device_id = device_id
        self.firmware = firmware or {"revision": "unknown", "dirty": None}
        self.producer_id = None
        self.session_id = uuid.uuid4().hex
        self.sequences = {}
        self.event_id = 0
        self.previous_states = None

    def adopt(self, sample):
        if sample.producer_id == self.producer_id:
            return False
        self.producer_id = sample.producer_id
        self.session_id = uuid.uuid4().hex
        self.sequences.clear()
        self.previous_states = None
        self.event_id = 0
        return True

    def packet(self, kind, sample, now_us, data):
        self.adopt(sample)
        sequence = self.sequences.get(kind, 0)
        self.sequences[kind] = (sequence + 1) & 0xFFFFFFFF
        value = {
            "schema": "phoenix.telemetry",
            "version": 1,
            "type": kind,
            "device_id": self.device_id,
            "session_id": self.session_id,
            "robot_id": sample.robot_id,
            "seq": sequence,
            "cycle": str(sample.cycle),
            "t_mono_us": str(sample.acquired_us),
            "snapshot_age_us": max(0, now_us - sample.acquired_us),
            "data": data,
        }
        raw = json.dumps(value, separators=(",", ":"), allow_nan=False).encode("utf-8")
        if len(raw) > MAX_PACKET:
            raise ValueError(f"{kind} packet exceeds {MAX_PACKET} bytes")
        return raw

    def identity(self, rates):
        return {
            "motor_columns": MOTOR_COLUMNS,
            "wheel_order": WHEEL_ORDER,
            "firmware": self.firmware,
            "capabilities": [
                "motor_reply",
                "controller_request",
                "submitted_demand",
                "imu",
                "vision_estimator",
                "kick_send_counter",
            ],
            "rate_caps_hz": rates,
            "position_unit": "output_rev",
            "current_unit": "q_axis_A",
            "temperature_source": "controller",
            "motor_timestamp_source": "CAN_cycle_completion",
            "command_acknowledgement": False,
        }

    @staticmethod
    def motion(sample):
        v = sample.values
        imu_present = bool(sample.flags & 1)
        bridge_active = bool(sample.flags & 128)
        # Estimator::output uses exactly 1e9 when no accepted vision fix exists.
        # A finite age from an old valid fix remains visible even after timeout.
        vision_available = bridge_active and v[27] is not None and v[27] != 1e9

        def vector(start):
            return list(v[start : start + 3]) if bridge_active else [None] * 3

        return {
            "target_pose_global": vector(0),
            "estimate_pose_global": vector(3),
            "estimate_velocity_global": vector(6),
            "reference_pose_global": vector(9),
            "reference_velocity_global": vector(12),
            "controller_output_body": vector(15),
            "command_age_s": v[24] if bridge_active else None,
            "command_received_seq": None,
            "command_applied_seq": None,
            "control_session_id": None,
            "elapsed_before_telemetry_s": v[25],
            "tick_dt_s": v[26],
            "imu_rate_raw_dps": list(v[18:21]) if imu_present else [None] * 3,
            "imu_accel_raw_mps2": list(v[21:24]) if imu_present else [None] * 3,
            "imu_available": imu_present,
            "estimator_precedes_motor_sample_s": v[32],
            "vision": {
                "available": vision_available,
                "alive": bool(sample.flags & 2) if vision_available else False,
                "age_s": v[27] if vision_available else None,
                "delay_s": v[28] if vision_available else None,
                "innovation_m": v[29] if vision_available else None,
                "heading_innovation_rad": v[30] if vision_available else None,
                "confidence": v[31] if vision_available and sample.flags & 4 else None,
            },
        }

    def events(self, sample):
        states = {"supervisor": bool(sample.flags & 8)}
        for wheel in sample.wheels:
            states[f"motor_{wheel[0]}"] = (wheel[10], wheel[8], wheel[9])
        events = []
        if self.previous_states is not None:
            for subsystem, state in states.items():
                old = self.previous_states.get(subsystem)
                if state == old:
                    continue
                if subsystem == "supervisor":
                    severity = "fault" if state else "info"
                    code = (
                        "supervisor.estop_set" if state else "supervisor.estop_cleared"
                    )
                else:
                    replied, mode, fault = state
                    if not replied:
                        severity, code = "warning", "motor.reply_missing"
                    elif mode == 1 or (
                        fault is not None and (1 <= fault <= 7 or 32 <= fault <= 50)
                    ):
                        severity, code = "fault", "motor.fault_changed"
                    elif mode == 11:
                        severity, code = "warning", "motor.watchdog_timeout"
                    elif fault is not None and 96 <= fault <= 105:
                        severity, code = "warning", "motor.limiting_changed"
                    elif fault not in (None, 0):
                        severity, code = "warning", "motor.unknown_code"
                    else:
                        severity, code = "info", "motor.state_changed"
                self.event_id += 1
                events.append(
                    {
                        "event_id": str(self.event_id),
                        "code": code,
                        "severity": severity,
                        "subsystem": subsystem,
                        "source": "sampled_transition",
                        "previous": old,
                        "current": state,
                    }
                )
        self.previous_states = states
        return events
