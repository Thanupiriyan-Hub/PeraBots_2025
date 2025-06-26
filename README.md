# 🧭 Perabots 2025 - Obstacle Avoidance & Path Optimization Robot (Round 1 Simulation)

This repository contains the **Webots simulation code** for our **obstacle-avoiding robot** designed for the **Perabots 2025 Robotics Competition**. The simulation is focused on **line-following, red marker detection (lap counting), and trajectory logging** for future path optimization.

## 🚀 Project Summary

In the first round, we simulated a differential drive robot with the ability to:
- Navigate using **camera-based line detection**
- Detect red zones to **count laps**
- Log wheel velocities and positions to **record and replay paths**
- Apply **smoothing** to sharp turns for optimized motion
- Integrate an **IMU sensor** for potential pose estimation and stability checks

This is implemented entirely using the **Webots C API** with built-in sensors including:
- Position sensors (encoders)
- Motors
- Camera
- **IMU (Inertial Measurement Unit)**

> 🛠️ Final Round: We are currently working on the **hardware prototype** using **STM32 MCU, TOF sensor, IMU, encoders, and gyroscope**. The full PCB and firmware will be published in this repository soon.

---

## 🧑‍🤝‍🧑 Team Members

- **Team Leader**: Thanupiriyan  
- Manuka  
- Piraveen  
- Tharmeegan  
- Nirusan  

---

## 📁 Repository Structure

```bash
.
├── controller/
│   └── robot_controller.c   # Main C file for Webots simulation logic
├── logs/
│   ├── lap1_log.txt         # Recorded lap data
│   └── lap1_log_smoothed.txt # Smoothed trajectory data
├── worlds/
│   └── perabots_arena.wbt   # Webots simulation world
├── README.md                # This file
