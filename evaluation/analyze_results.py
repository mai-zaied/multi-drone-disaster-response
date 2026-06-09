import pandas as pd
import matplotlib.pyplot as plt
import glob
import os

csv_files = glob.glob("evaluation/results_real/*.csv")

if not csv_files:
    print("No CSV files found.")
    exit()

all_data = []

for file in csv_files:
    df = pd.read_csv(file)
    all_data.append(df)

data = pd.concat(all_data, ignore_index=True)

print("\n=== SUMMARY ===\n")

summary = data.groupby("mode")[[
    "latency_sec",
    "completion_time_sec"
]].mean()

print(summary)

os.makedirs("evaluation/plots", exist_ok=True)

# Average latency
summary["latency_sec"].plot(kind="bar")
plt.ylabel("Seconds")
plt.title("Average Latency per Mode")
plt.tight_layout()
plt.savefig("evaluation/plots/avg_latency.png")
plt.close()

# Average completion time
summary["completion_time_sec"].plot(kind="bar")
plt.ylabel("Seconds")
plt.title("Average Completion Time per Mode")
plt.tight_layout()
plt.savefig("evaluation/plots/avg_completion.png")
plt.close()

print("\nPlots saved in evaluation/plots/")
