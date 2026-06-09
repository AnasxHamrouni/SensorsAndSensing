# Force Sensor Lab - Motor Control and Data Analysis

This project implements real-time motor control with force feedback sensing and data logging. The system reads force measurements from a sensor, controls a GYEMS DRC motor via CAN bus, and logs angular velocity, current, and force data for post-analysis.

## Project Overview

The experiment consists of two main components:

1. **`run_and_log.py`** - Real-time motor control loop that:
   - Reads force sensor input via serial connection
   - Controls a brushless DC motor with PD feedback control
   - Implements force-based mode switching (FORCE mode and RETURN home mode)
   - Logs all sensor data to CSV format

2. **`plot_force_current.py`** - Data analysis and visualization that:
   - Reads logged motor data from the CSV file
   - Filters unreliable measurements
   - Bins data by angular velocity
   - Fits polynomial approximations
   - Generates plots of Force(ω) and Current(ω) relationships

## Hardware Requirements

- **Motor**: GYEMS DRC brushless motor with CAN interface
- **Motor Controller**: Connected via CAN bus (default: `can0`)
- **Force Sensor**: Connected via USB serial port (default: `/dev/ttyUSB0` at 115200 baud)
- **Computer**: Linux-based system with CAN interface support

## Software Requirements

- Python 3.7+
- Dependencies:
  ```
  numpy
  matplotlib
  ```
- GYEMS Motor Control Library: https://github.com/valeriaskvo/gyems_motor_control

## Installation

### 1. Clone GYEMS Motor Control Repository

```bash
git clone https://github.com/valeriaskvo/gyems_motor_control.git
cd gyems_motor_control
git submodule update --init --recursive
```

### 2. Compile can-utils

Navigate to the `can-utils` directory and compile:

```bash
cd can-utils
make
sudo make install
cd ..
```

### 3. Install Python Dependencies

```bash
pip install numpy matplotlib pyserial
```

### 4. Set Up Python Path

Add the GYEMS repository to your Python path. Either:

**Option A**: Copy the `can` and `motors` directories to your project:
```bash
cp -r can motors /path/to/LAB2_forceSensor/
```

**Option B**: Add the repository to Python path in your script:
```python
import sys
sys.path.insert(0, '/path/to/gyems_motor_control')
```

### 5. Configure CANable (if using CANable USB adapter)

If using a CANable USB-to-CAN adapter:

```bash
# Setup the CAN interface (replace /dev/ttyACM0 with your device)
sudo slcand -o -c -s0 /dev/ttyACM0 can0
sudo ifconfig can0 up
sudo ifconfig can0 txqueuelen 1000

# Verify the interface is active
ip link show can0
```

If using a native CAN interface, activate it directly:

```bash
# Native CAN interface
sudo ip link set can0 up type can bitrate 1000000
```

### 6. Configure Serial Port Permissions

Allow user access to the serial port without sudo:

```bash
# Add current user to dialout group
sudo usermod -a -G dialout $USER
# Log out and back in for changes to take effect
```

## Configuration

### Project Structure

If you copied the modules locally, your project structure should look like:

```
LAB2_forceSensor/
├── can/              (from GYEMS repository)
├── motors/           (from GYEMS repository)
├── run_and_log.py
├── plot_force_current.py
├── run_log.csv
├── Force/
│   ├── 0.5.txt
│   ├── 1.txt
│   └── ... (calibration data files)
└── README.md
```

### Motor Parameters

Edit `run_and_log.py` to adjust:

```python
motor_param = {"interface": "can0", "id_motor": 0x141, "current_limit": 200}
```

- `interface`: CAN bus interface name (`can0` for CANable or native CAN)
- `id_motor`: Motor device ID on the CAN bus in hexadecimal (default: `0x141`)
- `current_limit`: Maximum current limit in Amperes (default: 200A)

### Force Sensor Calibration

The force sensor uses piecewise linear calibration. Update the calibration points in `run_and_log.py`:

```python
ADC_POINTS = [0, 2243.78, 5321.74, 18963.68, 23457.68, 25907.01]
F_POINTS_N = [0, 4.903325, 9.80665, 24.516625, 29.41995, 34.323275]
```

These correspond to:
- 0 kg → 0 N
- 0.5 kg → 4.903325 N
- 1.0 kg → 9.80665 N
- 2.5 kg → 24.516625 N
- 3.0 kg → 29.41995 N
- 3.5 kg → 34.323275 N

### Control Parameters

Adjust these values in `run_and_log.py`:

```python
# Force thresholds (ADC units)
F_DEAD = 300.0      # Deadband below which force is ignored
F_ON   = 1000.0     # Force threshold to enter FORCE mode
F_OFF  = 500.0      # Force threshold to leave FORCE mode

# Movement limits
XMAX_DEG = 1080.0   # Maximum rotation range in degrees
EPS_HOME_DEG = 0.3  # Tolerance for home position

# Velocity and acceleration limits
V_FORCE_MAX = 40.0      # Max velocity in FORCE mode (deg/s)
A_FORCE_MAX = 10.0      # Max acceleration in FORCE mode (deg/s²)
V_MAX_RETURN = 15.0     # Max velocity in RETURN mode (deg/s)
A_MAX_RETURN = 100.0    # Max acceleration in RETURN mode (deg/s²)

# PD controller gains
Kp = 3.0    # Proportional gain
Kd = 2.0    # Derivative gain

# Mechanical parameters
LEVER_M = 0.125  # Lever arm length in meters
```

## Running the Experiment

### Step 1: Start Data Collection

```bash
python run_and_log.py
```

The script will:
- Connect to the motor and force sensor
- Print real-time telemetry to the console
- Log all data to `run_log.csv`
- Run until interrupted with `Ctrl+C`

Example output:
```
CAN BUS connected successfully
Logging to: /path/to/run_log.csv
Motor control starts (FORCE=one direction, RETURN=home)
adc= 1234  Ff=  1234.5  mode=FORCE   q=   45.67  dq=  12.34deg/s  Icmd=   45.67
```

### Step 2: Stop the Experiment

Press `Ctrl+C` to stop. The script will:
- Disable the motor
- Save all logged data
- Display the CSV file location

### Step 3: Analyze Results

```bash
python plot_force_current.py
```

This generates two plots:
- **F(ω)**: Force vs angular velocity with polynomial fit
- **I(ω)**: Current vs angular velocity with polynomial fit

#### Plot Components

Each plot shows:
- **Light blue dots**: Raw filtered data points
- **Blue curve**: Binned averages (mean values in velocity bins)
- **Orange curve**: 2nd-degree polynomial fit to binned data

## Output Files

### `run_log.csv`

CSV file with the following columns:

| Column | Unit | Description |
|--------|------|-------------|
| t_s | seconds | Elapsed time |
| q_deg | degrees | Motor angle |
| dq_deg_s | deg/s | Motor angular velocity |
| omega_rad_s | rad/s | Motor angular velocity (radians) |
| I_cmd_A | Amperes | Current command sent to motor |
| I_meas_A | Amperes | Measured motor current (if available) |
| adc | counts | Raw force sensor ADC value |
| F_N | Newtons | Calibrated force measurement |
| tau_Nm | N⋅m | Torque = Force × Lever arm |
| mode | — | Operating mode (FORCE or RETURN) |

## Troubleshooting

### CANable Setup Issues
```
OSError: [Errno 19] No such device
```
- Ensure can-utils is compiled and installed (see Installation step 2)
- Verify CANable device is connected: `ls /dev/ttyACM*`
- Run the CANable setup commands with correct device path:
  ```bash
  sudo slcand -o -c -s0 /dev/ttyACM0 can0
  sudo ifconfig can0 up
  sudo ifconfig can0 txqueuelen 1000
  ```

### CAN Bus Connection Error
```
RuntimeError: Failed to connect to CAN interface
```
- Verify CAN interface is active: `ip link show can0`
- Check motor device ID is correct in `motor_param`
- Ensure CAN interface is properly initialized (see CANable Setup)
- Verify can-utils is compiled and installed

### Serial Port Connection Error
```
SerialException: could not open port /dev/ttyUSB0
```
- Verify sensor is connected: `ls /dev/ttyUSB*`
- Check user permissions: `ls -l /dev/ttyUSB0`
- Verify baud rate matches sensor (default: 115200)
- Add user to dialout group if permission denied

### Motor Saturation in Plots
If many data points exceed current limit (195A out of 200A limit):
- Reduce `V_FORCE_MAX` or `A_FORCE_MAX`
- Verify force sensor calibration
- Check for mechanical friction

### No Force Readings
- Verify serial connection and baud rate
- Check force sensor output format (expects integer values)
- Inspect sensor calibration data

## Data Filtering

The plotting script applies several filters to improve data quality:

```python
mask &= (np.abs(omega) > 0.02)     # Skip extremely low velocities
mask &= (F_N > 0.2)                # Skip very small forces
mask &= (np.abs(Icmd) < 195)       # Exclude current saturation region
```

Adjust these thresholds in `plot_force_current.py` based on your experimental observations.

## Additional Notes

- The control loop runs at **250 Hz** (configurable via `CTRL_HZ`)
- Data logging occurs at **250 Hz** (configurable via `LOG_HZ`)
- Console telemetry prints at **25 Hz** for real-time monitoring
- Force measurement uses low-pass filtering with α = 0.75 for noise reduction
- Mode switching includes hysteresis to prevent chatter near thresholds

## References

- GYEMS Motor Control Repository: https://github.com/valeriaskvo/gyems_motor_control
- RMD L-Series Motor Manual: Available in the GYEMS repository (RMD_L_Series_Servo_Actuator_User_Manual_Rev_1_01_Release_1.pdf)
- CANable Setup Guide: https://canable.io/getting-started.html
