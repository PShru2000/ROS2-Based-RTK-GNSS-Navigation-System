import pandas as pd
import utm
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns

# File paths
data_file = "/home/shreeya/Desktop/EECE5554/LAB2/Data/open_space_moving/open space moving.csv"
output_csv = "/home/shreeya/Desktop/EECE5554/LAB2/Data/open_space_moving/open_space_moving_utm.csv"

# Load the CSV file
df = pd.read_csv(data_file)

# Convert Latitude/Longitude to UTM
def latlon_to_utm(lat, lon):
    utm_x, utm_y, _, _ = utm.from_latlon(lat, lon)
    return utm_x, utm_y

df["UTM_X"], df["UTM_Y"] = zip(*df.apply(lambda row: latlon_to_utm(row["latitude"], row["longitude"]), axis=1))

# Save updated CSV file
df.to_csv(output_csv, index=False)
print(f"UTM conversion complete! Data saved in {output_csv}")

# Convert columns to numeric
df["UTM_X"] = pd.to_numeric(df["UTM_X"], errors="coerce")
df["UTM_Y"] = pd.to_numeric(df["UTM_Y"], errors="coerce")

# Drop NaN values
df = df.dropna(subset=["UTM_X", "UTM_Y"])

# Normalize UTM values for better visualization
df["UTM_X_Scaled"] = (df["UTM_X"] - df["UTM_X"].min()) / (df["UTM_X"].max() - df["UTM_X"].min()) * 25
df["UTM_Y_Scaled"] = (df["UTM_Y"] - df["UTM_Y"].min()) / (df["UTM_Y"].max() - df["UTM_Y"].min()) * 25

# Convert to NumPy arrays to avoid multi-dimensional indexing issues
utm_x = df["UTM_X_Scaled"].to_numpy()
utm_y = df["UTM_Y_Scaled"].to_numpy()

# 1. **GPS Trajectory - Line Plot**
plt.figure(figsize=(7, 6))
plt.plot(utm_x, utm_y, marker='o', linestyle='-', color='b', markersize=2, label="GPS Path (UTM)")
plt.xlabel("UTM Easting (Scaled)")
plt.ylabel("UTM Northing (Scaled)")
plt.title("GPS Trajectory (Open Space Moving)")
plt.legend()
plt.grid()
plt.savefig("gps_trajectory_mov.png")
plt.show()

# 2. **Scatter Plot**
plt.figure(figsize=(7, 6))
plt.scatter(utm_x, utm_y, c='blue', s=5, alpha=0.5)
plt.xlabel("UTM Easting (Scaled)")
plt.ylabel("UTM Northing (Scaled)")
plt.title("GPS Trajectory - Scatter Plot (Open Space Moving)") 
plt.grid(True)
plt.savefig("gps_scatter_mov.png")
plt.show()

# 3. **2D Histogram (Heatmap)**
plt.figure(figsize=(7, 6))
plt.hist2d(utm_x, utm_y, bins=50, cmap="viridis", density=False)
plt.colorbar(label="Frequency")
plt.xlabel("UTM Easting (Scaled)")
plt.ylabel("UTM Northing (Scaled)")
plt.title("2D Histogram of UTM Northing vs. UTM Easting (Open Space Moving")
plt.savefig("histogram_utm_mov.png")
plt.show()

