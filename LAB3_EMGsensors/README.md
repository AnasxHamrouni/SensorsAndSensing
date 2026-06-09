# LAB3 — EMG Sensor + GYEMS Motor Control

This repository contains scripts and Arduino firmware used to run a lab experiment where EMG activity is processed and used to control a GYEMS motor.

- **Live mode:** EMG is read from Arduino serial in real time and mapped to motor motion.
- **Playback mode:** Previously recorded EMG CSV is replayed to reproduce the same motor behavior offline.
- **Post-processing:** Logged motor/EMG data is plotted for analysis.

## Repository contents

- `readEMGserial.ino` — Arduino sketch for EMG acquisition and envelope extraction.
- `run_emg_motor.py` — Main script: EMG (serial or CSV) → motor control + logging.
- `processData.py` — Plot logged variables from `run_emg_log.csv`.
- `emg_log.csv`, `emg_logFirst.csv`, `emg_logSecondAndrey.csv` — Example EMG recordings.
- `run_emg_log.csv` — Example output log from motor run.

## Experiment setup (details in the report)

### Hardware

- Arduino-compatible board connected to an EMG sensor output (analog input `A0` in the sketch).
- GYEMS motor connected through CAN interface (`can0` in Python scripts).

### Software

- Python 3.10+ recommended.
- Arduino IDE (or Arduino CLI) to upload `readEMGserial.ino`.
- Python packages listed in `requirements.txt`.
- Motor/CAN modules from:
  - `https://github.com/valeriaskvo/gyems_motor_control`
  - required Python packages/modules used by this project imports:
    - `motors.gyems`
    - `can` with `CAN_Bus`

> Note: This repository does not include `can/` and `motors/` folders. You must provide them from `gyems_motor_control`.

## Linux + CANable setup (required for motor control)

These steps are based on your motor-control repository (`gyems_motor_control`) and should be run on Linux.

If you see `OSError: [Errno 19] No such device`:

1. Build and install `can-utils` from that repository:

```bash
cd gyems_motor_control/can-utils
make
sudo make install
```

2. Configure CANable (adjust serial device if needed):

```bash
sudo slcand -o -c -s0 /dev/ttyACM0 can0
sudo ifconfig can0 up
sudo ifconfig can0 txqueuelen 1000
```

Reference: `https://canable.io/getting-started.html`

## 1) Prepare Python environment

From project root:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## 1.1) Get motor-control modules used by this project

Clone your motor control repository and make its modules available.

```bash
git clone https://github.com/valeriaskvo/gyems_motor_control.git
```

Then choose one of the following options.

Option A (recommended, run from this project root):

```bash
ln -s ../gyems_motor_control/can ./can
ln -s ../gyems_motor_control/motors ./motors
```

Option B (alternative): copy `can/` and `motors/` folders from `gyems_motor_control` into this repository root.

After this, imports in `run_emg_motor.py` and `run_and_log.py` will resolve correctly.

## 2) Upload Arduino EMG firmware

1. Open `readEMGserial.ino` in Arduino IDE.
2. Select your board and serial port.
3. Upload the sketch.
4. Confirm Serial Monitor shows phase messages and CSV-like rows:
   - `time_ms,phase,raw,ac,rect,envelope`

The firmware runs a fixed protocol:

1. `RELAX1` — baseline calibration
2. `PREPARE` — get ready
3. `SQUEEZE` — muscle contraction
4. `RELAX2` — release

## 3) Run experiment (live EMG → motor, on Linux CAN setup)

By default, `run_emg_motor.py` uses serial port `/dev/ttyACM0`.
Also ensure CAN interface in script config matches your setup (`motor_param["interface"]`, default `can0`).

```bash
python run_emg_motor.py --serial-port /dev/ttyACM0
```

What it does:

- Reads EMG envelope from Arduino.
- Filters and applies hysteresis (`EMG_ON` / `EMG_OFF`).
- Commands motor forward during activation and returns home during relax.
- Logs run data to `run_emg_log.csv`.

If your Arduino appears under another device (for example `/dev/ttyUSB0`), pass that port.

## 4) Reproduce run from saved EMG CSV (no live EMG needed)

```bash
python run_emg_motor.py --emg-csv emg_log.csv
```

You can replace `emg_log.csv` with any CSV that contains:

- `time_ms,phase,raw,ac,rect,envelope`

## 5) Plot and analyze results

```bash
python processData.py run_emg_log.csv
```

The plot includes (if columns exist):

- EMG envelope (raw and filtered)
- Motor angle (`q_deg`)
- Motor speed (`dq_deg_s`)
- Current command (`I_cmd_A`)

## Important parameters to tune

Inside `run_emg_motor.py`:

- `EMG_ON`, `EMG_OFF` — activation hysteresis thresholds.
- `EMG_MAP_MAX` — envelope scaling to velocity command.
- `V_EMG_MAX`, `A_EMG_MAX` — motion speed/acceleration in active mode.
- `V_MAX_RETURN`, `A_MAX_RETURN` — return-home profile.
- `Kp`, `Kd` — PD gains mapped to current command.

Tune these for your specific sensor placement, muscle group, and motor/load setup.

## Publish to GitHub

From project root:

```bash
git init
git add .
git commit -m "Initial commit: EMG motor lab experiment"
git branch -M main
git remote add origin https://github.com/<your-username>/<your-repo>.git
git push -u origin main
```

If the repository already exists locally, skip `git init` and only set/update remote and push.

## Safety notes

- Keep emergency stop / motor disable procedure ready before testing.
- Start with low gains and low current limits.
- Ensure mechanical limits and travel range are safe before enabling closed-loop control.
