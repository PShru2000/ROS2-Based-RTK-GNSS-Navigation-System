# ROS2-Based-RTK-GNSS-Navigation-System

## Goal

This project implements a high-precision GPS data collection and analysis pipeline using Real-Time Kinematic (RTK) GNSS in a ROS2 environment. It compares RTK GNSS performance against standard GPS (Lab 1) to evaluate accuracy, reliability, and environmental sensitivity.

## System Overview

- **Hardware Used**:
  - Ardusimple simpleRTK2B-F9P boards (base and rover)
  - Telemetry radios (915 MHz) for correction transmission
  - GNSS antennas

- **Software Stack**:
  - ROS2 (Humble)
  - Python + pyserial
  - Custom ROS2 messages for enriched GPS data
  - Bag recording for post-processing
  - Python scripts for visualization and error analysis

## RTK GNSS Features

- Parses NMEA `GNGGA` sentences
- Adds fix quality info (RTK Float, RTK Fix)
- Converts Latitude/Longitude to UTM
- Publishes enriched position data over ROS2 `/gps` topic
- Stores data in ROS2 bag files for analysis

## Datasets Collected

| Environment | Type       | Description                                   |
|-------------|------------|-----------------------------------------------|
| Open        | Stationary | Rooftop data with RTK Fix                     |
| Occluded    | Stationary | Near buildings/trees                          |
| Open        | Moving     | Structured walk (300m) in clear space         |
| Occluded    | Moving     | Walk with partial signal blockage             |

## Analysis Results

### Open Stationary (Best-Case)

- Deviation: 0.15–0.20 m  
- Observation: Very stable GNSS fix, tightly clustered points

### Occluded Stationary

- Deviation: 0.7–1.0 m  
- Observation: Signal reflections and blockage cause mild scatter

### Open Moving

- Deviation: 0.3–0.4 m  
- Observation: Clean trajectory, reliable RTK fix during motion

### Occluded Moving (Worst-Case)

- Deviation: 0.9–1.1 m  
- Observation: Noisy data, trajectory affected by signal interference

## RTK GNSS vs. Standard GPS (Lab 1)

| Feature                     | Standard GPS (USB Puck) | RTK GNSS System       |
|----------------------------|-------------------------|-----------------------|
| Accuracy (Open Space)      | ~5–10 m                 | 0.15–0.4 m            |
| Accuracy (Occluded)        | 10–30 m                 | 0.7–1.1 m             |
| Fix Quality Info           | Not Available           | Available             |
| Data Stability             | Poor in motion          | Robust, consistent    |
| Requires Correction Link   | No                      | Yes (base-rover)      |
| Setup Complexity           | Plug-and-play           | Advanced setup        |

RTK GNSS provides sub-meter accuracy even in partially occluded environments and is far more reliable than standard GPS pucks for applications where precision is critical.

## Key Learnings

- Integrated serial communication with RTK hardware
- Published real-time high-precision GPS in ROS2
- Learned to extract and interpret fix quality from NMEA
- Performed accurate UTM conversion and recording
- Analyzed spatial error distributions using Python

## Conclusion

This project demonstrates the value of using RTK GNSS over standard GPS pucks . RTK GNSS enables centimeter-level positioning accuracy, robust fix quality monitoring, and reliable performance even in partially obstructed conditions, Making it ideal for autonomous systems and outdoor navigation tasks.
