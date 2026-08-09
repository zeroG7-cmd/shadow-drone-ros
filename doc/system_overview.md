# Shadow Robotics Platform — System Overview

## 1. Introduction

The Shadow Robotics Platform is a modular robotics development system designed for:

- UAV (drone) simulation and development
- Sensor integration (LiDAR, camera, IMU)
- Data logging and analysis
- Simulation-to-hardware transition
- Scalable robotics experimentation

The system integrates ROS, Gazebo, Python-based data pipelines, and SQLite for structured telemetry storage.

---

## 2. High-Level Architecture

The system is built in layered form:

1. Simulation Layer (Gazebo)
2. Robotics Middleware (ROS)
3. Visualization Layer (RViz)
4. Data Processing Layer (Python)
5. Storage Layer (SQLite)
6. Version Control Layer (GitHub)

---

## 3. System Data Flow

### Step 1 — Simulation / Hardware Execution
- Shadow drone operates in Gazebo or real-world hardware
- Sensors generate real-time data:
  - LiDAR (/scan)
  - Camera (/image_raw)
  - TF frames
  - Joint states (future expansion)

---

### Step 2 — ROS Communication Layer
- ROS publishes all sensor and state data as topics
- Topics act as the central communication bus

Examples:
- /scan
- /camera/image_raw
- /tf
- /joint_states

---

### Step 3 — Visualization Layer (RViz)
- RViz provides real-time visualization of ROS data
- Used for:
  - Debugging robot model
  - Inspecting sensor outputs
  - Verifying TF frame correctness

Note: RViz does NOT control the robot.

---

### Step 4 — Data Ingestion Layer (Python)
- Custom Python scripts extract:
  - Simulation logs
  - Hardware logs
  - ROS-derived test results

- Data is parsed into structured format:
  - Test name
  - Component
  - Result
  - Notes
  - Timestamp
  - Source (simulation/hardware)

---

### Step 5 — Database Layer (SQLite)
- All structured data is stored in a central SQLite database:

`shadow.db`

This acts as the system's "memory layer".

Capabilities:
- Query test history
- Compare simulation vs hardware results
- Track system reliability
- Support future analytics

---

### Step 6 — Version Control Layer (GitHub)
- GitHub stores:
  - Source code
  - Documentation
  - Log files (raw data snapshots)
  - Database backups

GitHub acts as a historical archive, not a live database.

---

## 4. Data Types

The system handles two primary data sources:

### Simulation Data
- Generated in Gazebo
- Used for testing and development
- Safe environment for experimentation

### Hardware Data
- Generated from real drone systems
- Used for validation and calibration
- Represents real-world performance

Both data types are stored in the same database with a `source` tag.

---

## 5. Key Design Principles

- Single source of truth: SQLite database
- Modular architecture: each layer is independent
- Simulation and hardware parity: both treated equally in data structure
- Scalable ingestion pipeline: new sensors can be added without redesign
- Separation of concerns:
  - ROS → communication
  - Python → processing
  - SQLite → storage
  - GitHub → history

---

## 6. Current System Status

### Implemented:
- ROS workspace structure
- Shadow drone URDF model
- Gazebo simulation environment
- LiDAR and camera sensors
- Python log ingestion pipeline
- SQLite database logging system
- GitHub-based version control

### In Progress:
- Full ROS control integration
- Simulation movement control (Gazebo physics)
- RViz debugging workflow expansion

### Future Work:
- PX4/ArduPilot integration
- Real drone hardware deployment
- Live telemetry streaming
- AI-based analytics layer

---

## 7. Summary

The Shadow Robotics Platform is evolving into a full robotics data ecosystem combining:

- Simulation
- Real hardware
- Structured data logging
- Scalable analysis
- Modular robotics control systems

The long-term goal is to create a unified system where simulation and real-world robotics operate under the same data architecture.
