"""Convert the hardware-free C++ ABI fixture into inspectable v1 wire fixtures."""

import json
from pathlib import Path

from diagnostics.ipc import decode
from diagnostics.wire import Encoder


def main():
    root = Path(__file__).parent / "fixtures"
    sample = decode((root / "diagnostics-v1.bin").read_bytes())
    encoder = Encoder("test-fixture-pi")
    encoder.adopt(sample)
    encoder.session_id = "0123456789abcdef0123456789abcdef"
    data = {
        "identity": encoder.identity(
            {"motors": 50, "motion": 20, "health": 1, "identity": 0.2}
        ),
        "motors": {"wheels": sample.wheels},
        "motion": encoder.motion(sample),
        "health": {
            "snapshot_offered": sample.offered,
            "snapshot_ipc_dropped": sample.dropped,
            "kick_requested_count": sample.kick_requested,
            "kick_bytes_sent_count": sample.kick_sent,
            "physical_kick_confirmation": None,
        },
        "event": {
            "event_id": "1",
            "code": "motor.reply_missing",
            "severity": "warning",
            "subsystem": "motor_2",
            "source": "sampled_transition",
        },
    }
    lines = [
        encoder.packet(kind, sample, sample.acquired_us + 500, values)
        for kind, values in data.items()
    ]
    (root / "telemetry-v1.ndjson").write_bytes(b"\n".join(lines) + b"\n")
    print(json.dumps({json.loads(line)["type"]: len(line) for line in lines}))


if __name__ == "__main__":
    main()
