import matplotlib.pyplot as plt
import numpy as np

# Generate time axis
time = np.linspace(0, 30, 301)

# Simulate encoder count behavior
count = np.zeros_like(time)

# Clockwise rotation (0–10s)
count[(time >= 0) & (time < 10)] = np.floor(time[(time >= 0) & (time < 10)])

# Stop (10–15s)
count[(time >= 10) & (time < 15)] = count[time < 10][-1]

# Counter-clockwise rotation (15–25s)
count[(time >= 15) & (time < 25)] = count[time < 10][-1] - np.floor(time[(time >= 15) & (time < 25)] - 15)

# Stop again (25–30s)
count[time >= 25] = count[(time >= 15) & (time < 25)][-1]

# Plot
plt.figure()
plt.plot(time, count)
plt.xlabel("Time")
plt.ylabel("Encoder Count")
plt.title("Quadrature Encoder Algorithm Check")
plt.show()
