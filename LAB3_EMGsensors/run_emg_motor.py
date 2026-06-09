from __future__ import annotations

from time import perf_counter, sleep
from pathlib import Path
import csv
import math
import re
import argparse
import io
from typing import Optional

import serial
from can import CAN_Bus
from motors.gyems import GyemsDRC


SERIAL_PORT = "/dev/ttyACM0"
SERIAL_BAUD = 115200
SERIAL_TIMEOUT_S = 0

_EMG_HEADER = ("time_ms", "phase", "raw", "ac", "rect", "envelope")
_NUMBER_RE = re.compile(r"[-+]?\d*\.?\d+")


class EmgSample(dict):
    """Typed dict-like holder for parsed EMG sample."""


def try_parse_emg_csv_row(row: dict) -> Optional[EmgSample]:
    try:
        return EmgSample(
            time_ms=float(row.get("time_ms", 0.0)),
            phase=str(row.get("phase", "CSV")),
            raw=int(float(row.get("raw", 0))),
            ac=int(float(row.get("ac", 0))),
            rect=int(float(row.get("rect", 0))),
            envelope=float(row.get("envelope", 0.0)),
        )
    except (ValueError, TypeError):
        return None


def read_next_csv_sample(reader: csv.DictReader) -> Optional[EmgSample]:
    for row in reader:
        sample = try_parse_emg_csv_row(row)
        if sample is not None:
            return sample
    return None


def load_emg_samples_from_csv(path: Path) -> list[EmgSample]:
    text = path.read_text(encoding="utf-8", errors="ignore")
    lines = text.splitlines()

    header_idx = None
    for idx, line in enumerate(lines):
        lower = line.strip().lower()
        if all(k in lower for k in _EMG_HEADER):
            header_idx = idx
            break

    if header_idx is None:
        raise RuntimeError(
            f"CSV header not found in {path}. Expected columns: time_ms,phase,raw,ac,rect,envelope"
        )

    csv_payload = "\n".join(lines[header_idx:])
    reader = csv.DictReader(io.StringIO(csv_payload))

    samples: list[EmgSample] = []
    for row in reader:
        sample = try_parse_emg_csv_row(row)
        if sample is not None:
            samples.append(sample)

    if not samples:
        raise RuntimeError(f"No valid EMG rows in CSV: {path}")

    return samples



def try_parse_emg_line(line: bytes) -> Optional[EmgSample]:
    if not line:
        return None

    try:
        s = line.decode("utf-8", errors="ignore").strip()
    except Exception:
        return None

    if not s:
        return None

    if s.startswith("===") or s.startswith("###"):
        return None

    lower = s.lower()
    if all(k in lower for k in _EMG_HEADER):
        return None

    parts = s.split(",")
    if len(parts) >= 6:
        try:
            return EmgSample(
                time_ms=float(parts[0]),
                phase=parts[1].strip(),
                raw=int(float(parts[2])),
                ac=int(float(parts[3])),
                rect=int(float(parts[4])),
                envelope=float(parts[5]),
            )
        except (ValueError, TypeError):
            pass

    nums = _NUMBER_RE.findall(s)
    if len(nums) >= 5:
        try:
            return EmgSample(
                time_ms=float(nums[0]),
                phase="UNKNOWN",
                raw=int(float(nums[1])),
                ac=int(float(nums[2])),
                rect=int(float(nums[3])),
                envelope=float(nums[4]),
            )
        except (ValueError, TypeError):
            return None

    return None


motor_param = {
    "interface": "can0",
    "id_motor": 0x141,
    "current_limit": 200,
}

CTRL_HZ = 250.0
DT_TARGET = 1.0 / CTRL_HZ

PRINT_HZ = 25.0
PRINT_DT = 1.0 / PRINT_HZ

LOG_HZ = 250.0
LOG_DT = 1.0 / LOG_HZ
OUT_CSV = Path("run_emg_log.csv")

XMAX_DEG = 1080.0
EPS_HOME_DEG = 0.3

EMG_ON = 120.0
EMG_OFF = 100.0
EMG_MAP_MAX = 250.0

EMG_LP_ALPHA = 0.85
EMG_HOLD_S = 0.5

V_EMG_MAX = 40.0
A_EMG_MAX = 120.0
V_MAX_RETURN = 15.0
A_MAX_RETURN = 120.0
MOVE_DIR = +1.0

Kp = 3.0
Kd = 2.0



def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x



def lowpass(prev: float, x: float, alpha: float) -> float:
    return alpha * prev + (1.0 - alpha) * x



def rate_limit(x: float, x_prev: float, max_rate: float, dt: float) -> float:
    dx = x - x_prev
    max_dx = max_rate * dt
    dx = clamp(dx, -max_dx, +max_dx)
    return x_prev + dx



def main() -> None:
    parser = argparse.ArgumentParser(description="Control RMD motor from EMG serial or saved CSV")
    parser.add_argument(
        "--emg-csv",
        type=str,
        default=None,
        help="Path to saved EMG CSV (columns: time_ms,phase,raw,ac,rect,envelope)",
    )
    parser.add_argument(
        "--serial-port",
        type=str,
        default=SERIAL_PORT,
        help="Serial port for live EMG mode",
    )
    args = parser.parse_args()

    emg_csv_path = Path(args.emg_csv) if args.emg_csv else None
    use_csv = emg_csv_path is not None

    ser: Optional[serial.Serial] = None
    csv_samples: list[EmgSample] = []
    csv_idx = 0
    csv_next_sample: Optional[EmgSample] = None
    csv_t0_ms = 0.0
    csv_playback_t0 = perf_counter()
    csv_finished = False

    if use_csv:
        if not emg_csv_path.exists():
            raise FileNotFoundError(f"EMG CSV file not found: {emg_csv_path}")
        csv_samples = load_emg_samples_from_csv(emg_csv_path)
        csv_next_sample = csv_samples[0]
        csv_t0_ms = float(csv_next_sample["time_ms"])
        csv_playback_t0 = perf_counter()
    else:
        ser = serial.Serial(args.serial_port, SERIAL_BAUD, timeout=SERIAL_TIMEOUT_S)

    bus = CAN_Bus(interface=motor_param["interface"])
    print("CAN BUS connected successfully")

    motor = GyemsDRC(can_bus=bus, device_id=motor_param["id_motor"])
    motor.set_degrees()
    motor.current_limit = motor_param["current_limit"]
    motor.enable()

    q0 = motor.state["angle"]
    q_des = q0

    active_mode = False
    prev_active_mode = False

    emg_raw = 0
    emg_ac = 0
    emg_rect = 0
    emg_env_raw = 0.0
    emg_env_f = 0.0
    emg_phase = "NA"
    t_emg_last = perf_counter()

    v_ref = 0.0

    t_next = perf_counter()
    t_last = perf_counter()
    t_print = perf_counter()
    t_log = perf_counter()

    if use_csv:
        print(f"Reading EMG from CSV: {emg_csv_path.resolve()}")
    else:
        print(f"Reading EMG from serial: {args.serial_port}")
    print(f"Logging to: {OUT_CSV.resolve()}")

    with OUT_CSV.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(
            [
                "t_s",
                "q_deg",
                "dq_deg_s",
                "omega_rad_s",
                "I_cmd_A",
                "I_meas_A",
                "emg_phase",
                "emg_raw",
                "emg_ac",
                "emg_rect",
                "emg_env",
                "emg_env_f",
                "mode",
            ]
        )

        print("Motor control starts (EMG_ACTIVE=forward, RELAX=return home)")

        try:
            while True:
                now = perf_counter()
                if now < t_next:
                    sleep(t_next - now)
                    now = perf_counter()
                t_next += DT_TARGET

                dt_real = now - t_last
                t_last = now
                if dt_real <= 1e-6:
                    dt_real = DT_TARGET

                q = motor.state["angle"]
                dq = motor.state["speed"]
                omega = dq * (math.pi / 180.0)
                I_meas = motor.state.get("current", None) if isinstance(motor.state, dict) else None

                sample: Optional[EmgSample] = None
                if use_csv:
                    if not csv_finished:
                        elapsed_replay_s = now - csv_playback_t0
                        while csv_next_sample is not None:
                            t_rel_s = (float(csv_next_sample["time_ms"]) - csv_t0_ms) / 1000.0
                            if t_rel_s <= elapsed_replay_s:
                                sample = csv_next_sample
                                csv_idx += 1
                                csv_next_sample = csv_samples[csv_idx] if csv_idx < len(csv_samples) else None
                            else:
                                break
                        if csv_next_sample is None:
                            csv_finished = True
                else:
                    sample = try_parse_emg_line(ser.readline())

                if sample is not None:
                    emg_phase = str(sample["phase"])
                    emg_raw = int(sample["raw"])
                    emg_ac = int(sample["ac"])
                    emg_rect = int(sample["rect"])
                    emg_env_raw = float(sample["envelope"])
                    t_emg_last = now
                else:
                    if (now - t_emg_last) > EMG_HOLD_S:
                        emg_env_raw = 0.0
                        emg_rect = 0
                        emg_ac = 0

                emg_env_f = lowpass(emg_env_f, emg_env_raw, EMG_LP_ALPHA)

                prev_active_mode = active_mode
                if not active_mode:
                    if emg_env_f > EMG_ON:
                        active_mode = True
                else:
                    if emg_env_f < EMG_OFF:
                        active_mode = False

                if (not prev_active_mode) and active_mode:
                    v_ref = max(0.0, v_ref)

                if active_mode:
                    emg01 = clamp((emg_env_f - EMG_ON) / max(1e-6, (EMG_MAP_MAX - EMG_ON)), 0.0, 1.0)
                    v_cmd = V_EMG_MAX * (emg01 ** 0.5)
                    v_des = MOVE_DIR * v_cmd

                    v_ref = rate_limit(v_des, v_ref, A_EMG_MAX, dt_real)
                    q_des = q_des + v_ref * dt_real
                    q_des = clamp(q_des, q0, q0 + XMAX_DEG)

                    if q_des >= (q0 + XMAX_DEG - 1e-6) and v_ref > 0.0:
                        v_ref = 0.0
                else:
                    err_home = q0 - q_des
                    VIRT_K_HOME = 2.0
                    v_des = clamp(VIRT_K_HOME * err_home, -V_MAX_RETURN, +V_MAX_RETURN)

                    v_ref = rate_limit(v_des, v_ref, A_MAX_RETURN, dt_real)
                    q_des = q_des + v_ref * dt_real

                    if abs(q_des - q0) < EPS_HOME_DEG and abs(dq) < 1.0:
                        q_des = q0
                        v_ref = 0.0

                dq_des = 0.0
                I_cmd = Kp * (q_des - q) + Kd * (dq_des - dq)
                I_cmd = clamp(I_cmd, -motor.current_limit, +motor.current_limit)
                motor.set_current(I_cmd)

                if now >= t_log:
                    t_log += LOG_DT
                    w.writerow(
                        [
                            now,
                            q,
                            dq,
                            omega,
                            I_cmd,
                            ("" if I_meas is None else I_meas),
                            emg_phase,
                            emg_raw,
                            emg_ac,
                            emg_rect,
                            emg_env_raw,
                            emg_env_f,
                            ("EMG_ACTIVE" if active_mode else "RELAX"),
                        ]
                    )

                if now >= t_print:
                    t_print += PRINT_DT
                    print(
                        f"phase={emg_phase:8s} env={emg_env_raw:7.2f} env_f={emg_env_f:7.2f} "
                        f"mode={'EMG_ACTIVE' if active_mode else 'RELAX':10s} "
                        f"q={q:8.2f} dq={dq:7.2f}deg/s Icmd={I_cmd:7.2f}"
                    )

                if csv_finished and (not active_mode) and abs(q_des - q0) < EPS_HOME_DEG and abs(dq) < 1.0:
                    print("CSV playback finished and motor returned home")
                    break

        except KeyboardInterrupt:
            motor.set_current(0)
            print("Stopped by user")
        finally:
            motor.disable()
            if ser is not None:
                ser.close()
            print("Motor disabled")
            print(f"Saved log: {OUT_CSV.resolve()}")


if __name__ == "__main__":
    main()
