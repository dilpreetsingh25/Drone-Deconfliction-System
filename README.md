# Drone Flight Conflict Detection and Visualization System

<img width="800" height="600" alt="image" src="https://github.com/user-attachments/assets/765c7d3e-8c38-4cb6-9d7f-1793f18051ba" />
<p align="center"></p>
The Drone Flight Conflict Detection and Visualization System is an advanced safety solution designed to prevent mid-air collisions in shared airspace. As drone operations rapidly expand across industries like delivery, surveying, and emergency response, this system provides critical infrastructure for safely coordinating multiple autonomous flights. By analyzing planned trajectories in both space and time dimensions, the system identifies potential conflicts before takeoff, enabling operators to modify flight plans proactively rather than reacting to emergencies.

[Drone Confliction Simulation.webm](https://github.com/user-attachments/assets/b1e978a1-0239-43f7-bca8-a627a6574c85)
<p align="center"></p>

## ✈️ Key Features

- **4D Conflict Detection**  
- **Configurable Safety Buffers**  
- **Detailed Conflict Reporting**
- **Interactive 4D Visualization**  

---

## 🛠️ Installation

### ✅ Prerequisites

- Python **3.7+**
- FFmpeg (required for animation export)
- NumPY  (Vector math and numerical computations)
- Matplotlib  (2D/3D visualization and animation)

### 🐧 Ubuntu (Debian-based)

**1. Install Python**
```bash
sudo apt update
sudo apt install python3 python3-pip
```
**2. Check Python Version**
```bash
python --version   # Should show Python 3.7 or higher
sudo apt install ffmpeg python3-matplotlib python3-numpy
```
**3. Install FFmpeg, Matplotlib, NumPy**
```bash
sudo apt install ffmpeg python3-matplotlib python3-numpy
```

### 🪟 Windows

**1. Install Python**
- Download the installer from: https://www.python.org/downloads/windows/
- During installation, make sure to check the box that says: Add Python to PATH.

**2. Check Python Version (in Command Prompt)**
```cmd
python --version   # Should show Python 3.7 or higher
```
**3. Install FFmpeg**
- Download from: https://ffmpeg.org/download.html
- Extract it and add the bin folder to your system's PATH.

**4. Install Matplotlib & NumPy (in Command Prompt)**
```cmd
pip install matplotlib numpy
```

### 🍎 MacOS

**1. Install Python (if not already installed)**
- Use Homebrew:
```bash
brew install python
```

**2. Check Python Version**
```bash
python --version   # Should show Python 3.7 or higher
```
**3. Install FFmpeg**
```bash
brew install ffmpeg
```

**4. Install Matplotlib & NumPy**
```bash
pip3 install matplotlib numpy
```

### 📂 Clone the repository

```bash
git clone https://github.com/dilpreetsingh25/Drone-Deconfliction-System.git
```
---

## ⚙️ Usage

 Run the system with:
```bash
python drone_deconfliction.py

```
### Follow the interactive prompts:
#### 1. Enter primary mission details:
```terminal
==================================================
Drone Deconfliction System
==================================================
Created 3 simulated flights in memory

==================================================
Primary Mission Configuration
==================================================
Enter the number of mission waypoints: 3

Waypoint 1:
  X coordinate (integer): 0
  Y coordinate (integer): 0
  Z coordinate (integer): 0

Waypoint 2:
  X coordinate (integer): 100
  Y coordinate (integer): 100
  Z coordinate (integer): 50

Waypoint 3:
  X coordinate (integer): 300
  Y coordinate (integer): 250
  Z coordinate (integer): 80

Enter mission time window
  Start time will be current time: 2025-07-26 12:43:33.895631
Enter mission duration in minutes: 10
  End time: 2025-07-26 12:53:33.895631

```
#### 2. View conflict analysis results:
- Clearance status
- Detailed conflict report (if any conflicts detected)

```terminal
  
==================================================
Deconfliction Results
==================================================
Conflict Detected: 1 conflict(s) found

Conflict Report:
==================================================
Conflict 1:
  Time: 2025-07-26 12:48:07.968931
  Location: (126.64, 119.98, 54.00)
  With Drone: drone3
  Minimum Distance: 8.34m

```

#### 3. Generate visualizations:

- 3D static plot of all flight paths
```bash
==================================================
Generating Visualizations
==================================================
Saved 3D plot to drone_paths.png
```
- 4D animation showing temporal evolution (with configurable FPS)
```bash
Enter animation frame rate (FPS, 5-30): 10
*c* argument looks
--
--
--
-- RGB or RGBA value for all points.
Saved 4D animation to drone_animation.mp4

Simulation complete!
- 3D plot saved to: drone_paths.png
- 4D animation saved to: drone_animation.mp4
```


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
<img width="500" height="1200" alt="deepseek_mermaid_20250726_1e839a" src="https://github.com/user-attachments/assets/31e6e90c-0800-4f3e-bcb0-a50065cdb202" />

