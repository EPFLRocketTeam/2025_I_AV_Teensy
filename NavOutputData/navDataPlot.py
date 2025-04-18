import pandas as pd
import matplotlib.pyplot as plt


# === Load your CSV file ===
filename = "NavOutputData/navFlight.csv"  # Change to your filename
df = pd.read_csv(filename)

# Affichage rapide pour test
print(df.head())

# === Optional: Rename columns if necessary ===
df.columns = [
    "Time(ms)", "X", "Y", "Z",
    "VX", "VY", "VZ",
    "AX", "AY", "AZ",
    "OX", "OY", "OZ",
    "WX", "WY", "WZ",
    "execution_time"
]

# Convert time from ms to seconds
df["Time(s)"] = df["Time(ms)"] / 1000.0

# === Plotting functions ===

def plot_group(df, cols, title, ylabel):
    plt.figure(figsize=(10, 4))
    for col in cols:
        plt.plot(df["Time(s)"], df[col], label=col)
    plt.title(title)
    plt.xlabel("Time (s)")
    plt.ylabel(ylabel)
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

# === Position ===
plot_group(df, ["X", "Y", "Z"], "Position vs Time", "Position (m)")

# === Velocity ===
plot_group(df, ["VX", "VY", "VZ"], "Velocity vs Time", "Velocity (m/s)")

# === Acceleration ===
plot_group(df, ["AX", "AY", "AZ"], "Acceleration vs Time", "Acceleration (m/s²)")

# === Orientation ===
plot_group(df, ["OX", "OY", "OZ"], "Orientation vs Time", "Degrees")

# === Angular Velocity ===
plot_group(df, ["WX", "WY", "WZ"], "Angular Velocity vs Time", "Degrees/s")

# === Execution Time Distribution ===
plt.figure(figsize=(8, 4))
plt.hist(df["execution_time"], bins=50, color='skyblue', edgecolor='black')
plt.title("Distribution of Execution Time")
plt.xlabel("Execution Time (ms)")
plt.ylabel("Frequency")
plt.grid(True)
plt.tight_layout()

plt.show()