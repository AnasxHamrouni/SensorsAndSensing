import csv
import numpy as np
import matplotlib.pyplot as plt

CSV_FILE = "run_log.csv"

omega, Icmd, F_N, mode = [], [], [], []

with open(CSV_FILE, "r") as f:
    r = csv.DictReader(f)
    for row in r:
        try:
            omega.append(float(row["omega_rad_s"]))
            Icmd.append(float(row["I_cmd_A"]))
            F_N.append(float(row["F_N"]))
            mode.append(row.get("mode", ""))
        except Exception:
            continue

omega = np.array(omega)
Icmd  = np.array(Icmd)
F_N   = np.array(F_N)
mode  = np.array(mode)

# Filter out invalid or unreliable data points
mask = np.isfinite(omega) & np.isfinite(Icmd) & np.isfinite(F_N)
mask &= (np.abs(omega) > 0.02)     # Skip near-zero angular velocity
mask &= (F_N > 0.2)                # Skip near-zero force readings
mask &= (np.abs(Icmd) < 195)       # Exclude current saturation (assuming 200A limit)

# Uncomment the line below to analyze only force control mode
# mask &= (mode == "FORCE")

omega2, I2, F2 = omega[mask], Icmd[mask], F_N[mask]

# Bin data by angular velocity and compute mean values within each bin
bins = np.linspace(np.min(omega2), np.max(omega2), 40)
idx = np.digitize(omega2, bins)

w_mean, I_mean, F_mean = [], [], []
for k in range(1, len(bins)):
    m = (idx == k)
    if np.sum(m) < 10:
        continue
    w_mean.append(np.mean(omega2[m]))
    I_mean.append(np.mean(I2[m]))
    F_mean.append(np.mean(F2[m]))

w_mean = np.array(w_mean)
I_mean = np.array(I_mean)
F_mean = np.array(F_mean)

# Fit 2nd degree polynomial approximations to the binned data
pI = np.poly1d(np.polyfit(w_mean, I_mean, deg=2))
pF = np.poly1d(np.polyfit(w_mean, F_mean, deg=2))

w_line = np.linspace(np.min(w_mean), np.max(w_mean), 300)

# Plot current vs angular velocity
plt.figure()
plt.scatter(omega2, I2, s=6, alpha=0.3)
plt.plot(w_mean, I_mean, linewidth=2)
plt.plot(w_line, pI(w_line), linewidth=2)
plt.xlabel("ω, rad/s")
plt.ylabel("I_cmd, A")
plt.title("I(ω)")
plt.grid(True)

# Plot force vs angular velocity
plt.figure()
plt.scatter(omega2, F2, s=6, alpha=0.3)
plt.ylim(0, 6)   # Adjust y-axis limits based on your sensor range (e.g., 0-6 N)
plt.xlim(-0.4, 0.6)  # Adjust x-axis limits as needed
plt.plot(w_mean, F_mean, linewidth=2)
plt.plot(w_line, pF(w_line), linewidth=2)
plt.xlabel("ω, rad/s")
plt.ylabel("F, N")
plt.title("F(ω)")
plt.grid(True)

plt.show()

print("I(ω) poly2:", pI)
print("F(ω) poly2:", pF)