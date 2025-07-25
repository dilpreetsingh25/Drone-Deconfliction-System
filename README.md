# Drone Flight Conflict Detection and Visualization System

<img width="800" height="600" alt="image" src="https://github.com/user-attachments/assets/765c7d3e-8c38-4cb6-9d7f-1793f18051ba" />

A comprehensive system for detecting and visualizing potential conflicts between drone flight paths in shared airspace. This solution performs spatio-temporal analysis to ensure safe drone operations by identifying conflicts **before** they occur.

## ✈️ Key Features

- **4D Conflict Detection**  
  Detects conflicts in **3D space + time** to ensure real-time safety awareness.

- **Safety Buffer**  
  Configurable minimum separation distance between flight paths.

- **Detailed Reporting**  
  Outputs conflict details including:
  - Location (X, Y, Z)
  - Time of conflict
  - Involved drone IDs

- **Visualization**
  - Generates **3D static plots** of flight paths
  - Creates **4D animations** (space + time) using Matplotlib and FFmpeg

- **User-Friendly Interface**  
  Enables interactive mission configuration and simulation.

---

## 🛠️ Installation

### ✅ Prerequisites

- Python **3.7+**
- **FFmpeg** (required for animation export)

Install FFmpeg:

**Ubuntu/Debian:**
```bash
sudo apt install ffmpeg matplotlib
```
**Window:**
```bash
conda install ffmpeg matplotlib
```
**Mac:**
```bash
sudo apt install ffmpeg matplotlib
```
**Clone the repository:**
```bash
git clone https://github.com/dilpreetsingh25/Drone-Deconfliction-System.git
```
---

## 🛠️ Usage

 Run the system with:
```bash
python drone_deconfliction.py

```
### Follow the interactive prompts:
#### 1. Enter primary mission details:
- Number of waypoints
- X, Y, Z coordinates for each waypoint
- Mission duration in minutes
#### 2. View conflict analysis results:
- Clearance status
- Detailed conflict report (if any conflicts detected)

#### 3. Generate visualizations:

- 3D static plot of all flight paths
- 4D animation showing temporal evolution (with configurable FPS)



## ✅ Example Scenerios


**Conflict-Free Mission**
```bash
Waypoint 1: (0, 0, 0)
Waypoint 2: (100, 100, 50)
Waypoint 3: (400, 400, 100)
Duration: 10 minutes
```

**Conflict-Prone Mission**
```bash
Waypoint 1: (0, 0, 0)
Waypoint 2: (150, 150, 40)
Waypoint 3: (300, 0, 20)
Duration: 9 minutes
```

**Sample output for conflict scenario:**
```bash
Conflict Report:
==================================================
Conflict 1:
  Time: [Current Time] + 0:04:00
  Location: (100.2, 125.5, 40.1)
  With Drone: drone1
  Minimum Distance: 2.3m
--------------------------------------------------
Conflict 2:
  Time: [Current Time] + 0:05:00
  Location: (150.0, 150.0, 50.0)
  With Drone: drone2
  Minimum Distance: 0.0m
--------------------------------------------------
Conflict 3:
  Time: [Current Time] + 0:06:00
  Location: (225.4, 49.8, 30.2)
  With Drone: drone3
  Minimum Distance: 5.7m
--------------------------------------------------
```
---
## System Architecture
<img width="300" height="500" alt="deepseek_mermaid_20250725_231b34" src="https://github.com/user-attachments/assets/59f0486b-2463-4929-b567-3247802c4e40" />
