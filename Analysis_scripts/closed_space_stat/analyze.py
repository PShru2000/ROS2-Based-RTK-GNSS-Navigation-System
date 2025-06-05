import pandas as pd
import utm
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns

# File paths
data_file = "/home/shreeya/Desktop/EECE5554/LAB2/Data/closed_space_stat/occuluded_stationary 1.csv"
output_csv = "/home/shreeya/Desktop/EECE5554/LAB2/Data/closed_space_stat/occuluded_stationary_utm 1.csv"

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

# Normalize UTM values for better visualization
df["UTM_X_Scaled"] = (df["UTM_X"] - df["UTM_X"].min()) / (df["UTM_X"].max() - df["UTM_X"].min())
df["UTM_Y_Scaled"] = (df["UTM_Y"] - df["UTM_Y"].min()) / (df["UTM_Y"].max() - df["UTM_Y"].min())

# Drop NaN values
df = df.dropna(subset=["UTM_X_Scaled", "UTM_Y_Scaled"])

# **Scatter Plot of GPS Trajectory**
plt.figure(figsize=(10, 6))
plt.scatter(df["UTM_X"], df["UTM_Y"], c='blue', s=5, alpha=0.5)  # Scatter plot with transparency
plt.xlabel("UTM Easting (m)")
plt.ylabel("UTM Northing (m)")
plt.title("Closed Space Stationary - Scatter Plot")
plt.grid(True)
plt.savefig("scatter_plot_closed_space_stationary.png")
plt.show()

# **2D Histogram (Heatmap)**
plt.figure(figsize=(10, 6))
plt.hist2d(df["UTM_X_Scaled"], df["UTM_Y_Scaled"], bins=50, cmap="viridis", density=False)
plt.colorbar(label="Point Density")
plt.xlabel("UTM Easting (Scaled)")
plt.ylabel("UTM Northing (Scaled)")
plt.title("Closed Space Stationary - 2D Histogram of UTM Northing vs. UTM Easting")
plt.savefig("utm_2D_histogram_closed_space_stationary.png")
plt.show()

