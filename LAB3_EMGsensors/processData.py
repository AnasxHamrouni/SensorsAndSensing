import argparse

import matplotlib.pyplot as plt
import pandas as pd


def main() -> None:
	parser = argparse.ArgumentParser(description="Plot EMG/motor log data")
	parser.add_argument("csv", nargs="?", default="run_emg_log.csv", help="Path to CSV log")
	args = parser.parse_args()

	df = pd.read_csv(args.csv)

	if "t_s" not in df.columns:
		raise ValueError("Expected column t_s in the log file.")

	t = pd.to_numeric(df["t_s"], errors="coerce")
	df = df[t.notna()].copy()
	t = df["t_s"] - df["t_s"].iloc[0]

	fig, axs = plt.subplots(5, 1, figsize=(12, 12), sharex=True)

	if "emg_env" in df.columns:
		axs[0].plot(t, df["emg_env"])
		axs[0].set_title("EMG envelope (raw)")

	if "emg_env_f" in df.columns:
		axs[1].plot(t, df["emg_env_f"])
		axs[1].set_title("EMG envelope (filtered)")

	if "q_deg" in df.columns:
		axs[2].plot(t, df["q_deg"])
		axs[2].set_title("Motor angle (deg)")

	if "dq_deg_s" in df.columns:
		axs[3].plot(t, df["dq_deg_s"])
		axs[3].set_title("Motor speed (deg/s)")

	if "I_cmd_A" in df.columns:
		axs[4].plot(t, df["I_cmd_A"])
		axs[4].set_title("Current command (A)")

	axs[4].set_xlabel("time [s]")
	plt.tight_layout()
	plt.show()


if __name__ == "__main__":
	main()
