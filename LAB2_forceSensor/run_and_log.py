from time import perf_counter, sleep
from can import CAN_Bus
from motors.gyems import GyemsDRC
import serial
import re
from typing import Optional
import math
import csv
from pathlib import Path


SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUD = 115200
ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0)

_INT_RE = re.compile(r"[-+]?\d+")

def read_force_line(ser, *, f_min: int = 0, f_max: int = 26000, jump_max: int = 30000) -> Optional[int]:
    line = ser.readline()
    if not line:
        return None
    try:
        s = line.decode("utf-8", errors="ignore").strip()
    except Exception:
        return None
    if not s:
        return None
    nums = _INT_RE.findall(s)
    if not nums:
        return None
    try:
        val = int(nums[-1])
    except ValueError:
        return None
    if abs(val) > jump_max:
        return None
    if val < f_min:
        val = f_min
    if val > f_max:
        val = f_max
    return val

#  CALIBRATION: ADC -> Newton

# 0.5 kg -> 4.903325 N
# 1.0 kg -> 9.80665 N
# 2.5 kg -> 24.516625 N
# 3.0 kg -> 29.41995 N
# 3.5 kg -> 34.323275 N
ADC_POINTS = [0, 2243.7820512820513, 5321.74, 18963.676470588234, 23457.68253968254, 25907.011904761905]
F_POINTS_N = [0, 4.903325, 9.80665, 24.516625, 29.41995, 34.323275]

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def lowpass(prev, x, alpha):
    return alpha * prev + (1 - alpha) * x

def rate_limit(x, x_prev, max_rate, dt):
    dx = x - x_prev
    max_dx = max_rate * dt
    dx = clamp(dx, -max_dx, +max_dx)
    return x_prev + dx

def adc_to_newton(adc: float) -> float:
    # linear interpolation using calibration points to account for sensor nonlinearity
    # Clamp values outside the calibration range to the extreme values
    if adc <= ADC_POINTS[0]:
        return float(F_POINTS_N[0])
    if adc >= ADC_POINTS[-1]:
        return float(F_POINTS_N[-1])
    # Manual linear interpolation between calibration points
    for i in range(len(ADC_POINTS) - 1):
        x0, x1 = ADC_POINTS[i], ADC_POINTS[i + 1]
        if x0 <= adc <= x1:
            y0, y1 = F_POINTS_N[i], F_POINTS_N[i + 1]
            t = (adc - x0) / (x1 - x0)
            return float(y0 + t * (y1 - y0))
    return float(F_POINTS_N[-1])

motor_param = {"interface": "can0", "id_motor": 0x141, "current_limit": 200}

F_SENSOR_MAX = 26000.0
F_DEAD = 300.0
F_ON   = 1000.0
F_OFF  = 500.0
FORCE_LP_ALPHA = 0.75
FORCE_HOLD_S = 1

XMAX_DEG = 1080.0

V_FORCE_MAX = 40.0
A_FORCE_MAX = 10.0
F_MAP_MAX   = 20000.0

V_MAX_RETURN = 15.0
A_MAX_RETURN = 100.0
EPS_HOME_DEG = 0.3

CTRL_HZ = 250.0
DT_TARGET = 1.0 / CTRL_HZ

Kp = 3.0
Kd = 2.0

FORCE_DIR = +1.0

# Lever length (12.5 cm)
LEVER_M = 0.125  # meters

# Data logging configuration
LOG_HZ = 250.0
LOG_DT = 1.0 / LOG_HZ
OUT_CSV = Path("run_log.csv")

PRINT_HZ = 25.0
PRINT_DT = 1.0 / PRINT_HZ

bus = CAN_Bus(interface=motor_param["interface"])
print("CAN BUS connected successfully")

motor = GyemsDRC(can_bus=bus, device_id=motor_param["id_motor"])
motor.set_degrees()
motor.current_limit = motor_param["current_limit"]
motor.enable()

q0 = motor.state["angle"]
q_des = q0

force_mode = False
prev_force_mode = False

F_raw = 0
Ff = 0.0
t_force_last = perf_counter()

v_ref = 0.0

t_next = perf_counter()
t_last = perf_counter()
t_print = perf_counter()
t_log = perf_counter()

print(f"Logging to: {OUT_CSV.resolve()}")

# Create CSV file and write header
with OUT_CSV.open("w", newline="") as f:
    w = csv.writer(f)
    w.writerow([
        "t_s", "q_deg", "dq_deg_s", "omega_rad_s",
        "I_cmd_A", "I_meas_A",
        "adc", "F_N", "tau_Nm",
        "mode"
    ])

    print("Motor control starts (FORCE=one direction, RETURN=home)")

    try:
        while True:
            # Maintain fixed control loop rate
            now = perf_counter()
            if now < t_next:
                sleep(t_next - now)
                now = perf_counter()
            t_next += DT_TARGET

            dt_real = now - t_last
            t_last = now
            if dt_real <= 1e-6:
                dt_real = DT_TARGET

            # Read motor state
            q = motor.state["angle"]
            dq = motor.state["speed"]   # deg/s
            omega = dq * (math.pi / 180.0)
            I_meas = motor.state.get("current", None) if isinstance(motor.state, dict) else None

            # Read force sensor with timeout (hold last value if sensor stalls)
            F_new = read_force_line(ser, f_min=0, f_max=int(F_SENSOR_MAX))
            if F_new is not None:
                F_raw = F_new
                t_force_last = now
            else:
                if (now - t_force_last) > FORCE_HOLD_S:
                    F_raw = 0

            # Apply low-pass filter and deadband to force measurement
            Ff = lowpass(Ff, float(F_raw), FORCE_LP_ALPHA)
            F_eff = 0.0 if Ff < F_DEAD else Ff

            # Mode switching with hysteresis
            prev_force_mode = force_mode
            if not force_mode:
                if F_eff > F_ON:
                    force_mode = True
            else:
                if F_eff < F_OFF:
                    force_mode = False

            if (not prev_force_mode) and force_mode:
                v_ref = max(0.0, v_ref)

            # Control
            if force_mode:
                f01 = clamp(F_eff / F_MAP_MAX, 0.0, 1.0)
                g = f01 ** 0.5
                v_cmd = V_FORCE_MAX * g
                v_des = FORCE_DIR * v_cmd

                v_ref = rate_limit(v_des, v_ref, A_FORCE_MAX, dt_real)
                q_des = q_des + v_ref * dt_real
                q_des = clamp(q_des, q0, q0 + XMAX_DEG)

                if q_des >= (q0 + XMAX_DEG - 1e-6) and v_ref > 0:
                    v_ref = 0.0
            else:
                err_home = (q0 - q_des)
                VIRT_K_HOME = 2.0
                v_des = clamp(VIRT_K_HOME * err_home, -V_MAX_RETURN, +V_MAX_RETURN)

                v_ref = rate_limit(v_des, v_ref, A_MAX_RETURN, dt_real)
                q_des = q_des + v_ref * dt_real

                if abs(q_des - q0) < EPS_HOME_DEG and abs(dq) < 1.0:
                    q_des = q0
                    v_ref = 0.0

            # Motor Control (PD -> Current Command)
            dq_des = 0.0
            I_cmd = Kp * (q_des - q) + Kd * (dq_des - dq)
            I_cmd = clamp(I_cmd, -motor.current_limit, +motor.current_limit)
            motor.set_current(I_cmd)

            # Data Logging (angular velocity, current, force)
            if now >= t_log:
                t_log += LOG_DT
                F_N = adc_to_newton(Ff) if Ff > 0 else 0.0
                tau = F_N * LEVER_M

                w.writerow([
                    now, q, dq, omega,
                    I_cmd, ("" if I_meas is None else I_meas),
                    int(F_raw), F_N, tau,
                    ("FORCE" if force_mode else "RETURN")
                ])

            # Print real-time telemetry to console for monitoring
            if now >= t_print:
                t_print += PRINT_DT
                print(
                    f"adc={int(F_raw):5d}  Ff={Ff:8.1f}  mode={'FORCE' if force_mode else 'RETURN':6s}  "
                    f"q={q:8.2f}  dq={dq:7.2f}deg/s  Icmd={I_cmd:7.2f}"
                )

    except KeyboardInterrupt:
        motor.set_current(0)
        print("Stopped by user")

    finally:
        motor.disable()
        print("Motor disabled")
        print(f"Saved log: {OUT_CSV.resolve()}")