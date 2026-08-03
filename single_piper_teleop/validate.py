#!/usr/bin/env python3
"""Validate timing, command tracking, robot status, and RGB-D references."""

import argparse
import json
import math
from pathlib import Path
from typing import Any


def percentile(values: list[float], p: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = (len(ordered) - 1) * p
    low = math.floor(index)
    high = math.ceil(index)
    if low == high:
        return ordered[low]
    return ordered[low] * (high - index) + ordered[high] * (index - low)


def load_samples(path: Path) -> tuple[list[dict[str, Any]], int]:
    samples = []
    malformed = 0
    with path.open(encoding="utf-8") as stream:
        for line in stream:
            try:
                samples.append(json.loads(line))
            except json.JSONDecodeError:
                malformed += 1
    return samples, malformed


def validate(episode: Path) -> dict[str, Any]:
    samples_path = episode / "samples.jsonl"
    metadata_path = episode / "metadata.json"
    metadata = json.loads(metadata_path.read_text(encoding="utf-8")) if metadata_path.is_file() else None
    samples, malformed = load_samples(samples_path)
    if not samples:
        return {"valid": False, "reasons": ["no samples"], "sample_count": 0}

    mono = [int(s["timestamp_monotonic_ns"]) for s in samples]
    xr = [int(s.get("timestamp_xr_ns") or 0) for s in samples]
    dt_ms = [(b - a) / 1e6 for a, b in zip(mono, mono[1:]) if b > a]
    duration_s = (mono[-1] - mono[0]) / 1e9 if len(mono) > 1 else 0.0
    rates = len(samples) / duration_s if duration_s > 0 else 0.0
    non_monotonic_pc = sum(b <= a for a, b in zip(mono, mono[1:]))
    positive_xr = [v for v in xr if v > 0]
    non_monotonic_xr = sum(b <= a for a, b in zip(positive_xr, positive_xr[1:]))

    tracking_errors = []
    status_errors = 0
    non_normal = 0
    tracer_errors = 0
    tracer_estops = 0
    # Commands are written after the feedback in the same sample. Compare each
    # active command with feedback at least 100ms later instead of pre-command feedback.
    future_index = 0
    for index, sample in enumerate(samples):
        commanded = sample.get("commanded_joint_deg")
        if commanded and sample.get("clutch_active") and not sample.get("dry_run", False):
            target_ns = int(sample["timestamp_monotonic_ns"]) + 100_000_000
            future_index = max(future_index, index + 1)
            while future_index < len(samples) and int(samples[future_index]["timestamp_monotonic_ns"]) < target_ns:
                future_index += 1
            if future_index < len(samples):
                measured = samples[future_index].get("piper_joint_deg")
                if measured:
                    tracking_errors.append(max(abs(float(a) - float(b)) for a, b in zip(commanded, measured)))
    for sample in samples:
        status = sample.get("piper_status")
        if status:
            status_errors += int(status.get("error_code", 0) != 0)
            non_normal += int(status.get("arm_status", 0) != 0)
        tracer = sample.get("tracer_status")
        if tracer:
            tracer_errors += int(tracer.get("error", 0) != 0)
            tracer_estops += int(tracer.get("vehicle", 0) != 0)

    unique_frames: dict[int, dict[str, Any]] = {}
    missing_camera_refs = 0
    camera_age_ms = []
    missing_files = 0
    for sample in samples:
        ref = sample.get("camera_frame_reference")
        if not ref:
            missing_camera_refs += 1
            continue
        number = int(ref["frame_number"])
        unique_frames[number] = ref
        camera_age_ms.append((int(sample["timestamp_monotonic_ns"]) - int(ref["timestamp_monotonic_ns"])) / 1e6)
    for ref in unique_frames.values():
        for key in ("color_path", "depth_path"):
            if not (episode / ref[key]).is_file():
                missing_files += 1
    frame_numbers = sorted(unique_frames)
    dropped_frames = sum(max(0, b - a - 1) for a, b in zip(frame_numbers, frame_numbers[1:]))
    camera_rate = len(unique_frames) / duration_s if duration_s > 0 else 0.0

    reasons = []
    if metadata is None:
        reasons.append("episode metadata missing")
    elif metadata.get("status") != "complete":
        reasons.append(f"episode status is {metadata.get('status')}")
    elif not metadata.get("dry_run", False) and not metadata.get("unsafe_stop", False):
        if not metadata.get("return_confirmed") or not metadata.get("torque_released"):
            reasons.append("normal shutdown did not confirm return and torque release")
    if malformed:
        reasons.append(f"{malformed} malformed JSON lines")
    if non_monotonic_pc:
        reasons.append(f"{non_monotonic_pc} non-monotonic PC timestamps")
    if status_errors or non_normal:
        reasons.append(f"robot status failures: errors={status_errors}, non_normal={non_normal}")
    if tracer_errors or tracer_estops:
        reasons.append(f"tracer status failures: errors={tracer_errors}, estop={tracer_estops}")
    if missing_files:
        reasons.append(f"{missing_files} camera files missing")
    if missing_camera_refs / len(samples) > 0.10:
        reasons.append(f"camera references missing in {missing_camera_refs}/{len(samples)} samples")
    if not samples[0].get("dry_run", False) and tracking_errors and (percentile(tracking_errors, 0.95) or 0) > 5.0:
        reasons.append("joint tracking p95 exceeds 5 degrees")
    if rates < 35.0:
        reasons.append(f"control sample rate too low: {rates:.1f}Hz")
    if unique_frames and camera_rate < 20.0:
        reasons.append(f"camera frame rate too low: {camera_rate:.1f}Hz")

    return {
        "valid": not reasons,
        "reasons": reasons,
        "sample_count": len(samples),
        "episode_metadata": metadata,
        "duration_s": duration_s,
        "control_rate_hz": rates,
        "control_dt_ms": {"median": percentile(dt_ms, 0.5), "p95": percentile(dt_ms, 0.95), "max": max(dt_ms) if dt_ms else None},
        "timestamp": {"pc_non_monotonic": non_monotonic_pc, "xr_non_monotonic": non_monotonic_xr},
        "robot": {
            "status_error_samples": status_errors,
            "non_normal_samples": non_normal,
            "joint_tracking_max_error_deg": {
                "median": percentile(tracking_errors, 0.5),
                "p95": percentile(tracking_errors, 0.95),
                "max": max(tracking_errors) if tracking_errors else None,
            },
        },
        "tracer": {
            "error_samples": tracer_errors,
            "estop_samples": tracer_estops,
        },
        "camera": {
            "unique_frames": len(unique_frames),
            "rate_hz": camera_rate,
            "dropped_frame_numbers": dropped_frames,
            "missing_references": missing_camera_refs,
            "missing_files": missing_files,
            "sample_frame_age_ms": {
                "median": percentile(camera_age_ms, 0.5),
                "p95": percentile(camera_age_ms, 0.95),
                "max": max(camera_age_ms) if camera_age_ms else None,
            },
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("episode", type=Path)
    args = parser.parse_args()
    result = validate(args.episode)
    output = args.episode / "validation.json"
    output.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))
    print("Saved:", output)
    raise SystemExit(0 if result["valid"] else 2)


if __name__ == "__main__":
    main()
