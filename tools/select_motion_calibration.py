"""Apply or verify a named motion calibration in config/Motion.yaml."""

from __future__ import annotations

import argparse
import json
import math
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
REGISTRY = ROOT / "config" / "motion_calibrations.json"
MOTION = ROOT / "config" / "Motion.yaml"


def load_registry() -> dict:
    value = json.loads(REGISTRY.read_text(encoding="utf-8"))
    if value.get("schema") != 1 or not isinstance(value.get("profiles"), dict):
        raise ValueError(f"{REGISTRY}: unsupported calibration registry")
    return value


def parse_motion_values(text: str) -> dict[str, float]:
    section = ""
    values: dict[str, float] = {}
    for line in text.splitlines():
        section_match = re.match(r"^([A-Za-z_][A-Za-z0-9_]*):\s*(?:#.*)?$", line)
        if section_match:
            section = section_match.group(1)
            continue
        value_match = re.match(
            r"^\s{2}([A-Za-z_][A-Za-z0-9_]*):\s*"
            r"([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)",
            line,
        )
        if section and value_match:
            values[f"{section}.{value_match.group(1)}"] = float(value_match.group(2))
    return values


def patch_motion(text: str, parameters: dict[str, float]) -> str:
    remaining = dict(parameters)
    output: list[str] = []
    section = ""
    for line in text.splitlines(keepends=True):
        section_match = re.match(r"^([A-Za-z_][A-Za-z0-9_]*):\s*(?:#.*)?(?:\r?\n)?$", line)
        if section_match:
            section = section_match.group(1)
            output.append(line)
            continue
        key_match = re.match(
            r"^(\s{2})([A-Za-z_][A-Za-z0-9_]*):(\s*)"
            r"([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)"
            r"([^\r\n]*)(\r?\n)?$",
            line,
        )
        full_key = f"{section}.{key_match.group(2)}" if key_match else ""
        if key_match and full_key in remaining:
            value = remaining.pop(full_key)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"{full_key}: calibration value must be numeric")
            value = float(value)
            if not math.isfinite(value):
                raise ValueError(f"{full_key}: calibration value must be finite")
            output.append(
                f"{key_match.group(1)}{key_match.group(2)}:"
                f"{key_match.group(3)}{value:.16g}"
                f"{key_match.group(5)}{key_match.group(6) or ''}"
            )
        else:
            output.append(line)
    if remaining:
        raise ValueError(f"{MOTION}: missing keys {sorted(remaining)}")
    return "".join(output)


def profile_parameters(profile_id: str) -> dict[str, float]:
    profiles = load_registry()["profiles"]
    if profile_id not in profiles:
        raise ValueError(
            f"unknown profile {profile_id!r}; choose one of {sorted(profiles)}"
        )
    parameters = profiles[profile_id].get("parameters")
    if not isinstance(parameters, dict) or not parameters:
        raise ValueError(f"{profile_id}: missing parameters")
    return parameters


def verify(profile_id: str) -> list[str]:
    expected = profile_parameters(profile_id)
    actual = parse_motion_values(MOTION.read_text(encoding="utf-8"))
    mismatches = []
    for key, value in expected.items():
        if key not in actual or not math.isclose(
            actual[key], float(value), rel_tol=1e-12, abs_tol=1e-12
        ):
            mismatches.append(
                f"{key}: expected {value!r}, found {actual.get(key)!r}"
            )
    return mismatches


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "profile",
        choices=sorted(load_registry()["profiles"]),
        help="calibration profile to apply or verify",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="verify Motion.yaml without changing it",
    )
    args = parser.parse_args()

    if args.check:
        mismatches = verify(args.profile)
        if mismatches:
            print("\n".join(mismatches))
            return 1
        print(f"Motion.yaml matches {args.profile}")
        return 0

    original = MOTION.read_text(encoding="utf-8")
    patched = patch_motion(original, profile_parameters(args.profile))
    if patched != original:
        backup = MOTION.with_suffix(".yaml.before-calibration")
        backup.write_text(original, encoding="utf-8")
        MOTION.write_text(patched, encoding="utf-8")
        print(f"Applied {args.profile}; rollback snapshot: {backup}")
    else:
        print(f"Motion.yaml already matches {args.profile}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
