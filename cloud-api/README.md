# Cloud Integration README — Task 5 Extension

## Overview

This module extends the multi-drone disaster response system with a lightweight cloud layer that archives mission events from the fog decision system.

The cloud system receives important drone events such as:

- survivor detections
- mission updates
- drone alerts
- fog decisions

and stores them through a Flask-based REST API.

This creates a complete pipeline:

text Drone → Fog Node → Decision System → Cloud Client → Cloud API → Cloud Storage

The cloud layer enables:
- centralized event logging
- remote monitoring
- mission history archival
- future dashboard integration
- cloud analytics support

---

## Components

| Component | Role |
|---|---|
| cloud-api/app.py | Flask REST API server |
| cloud_client.py | ROS2 node that uploads events to cloud |
| cloud_logs.json | Archived mission logs |
| /decision/status | ROS2 topic carrying mission decisions |

---

## Cloud API Features

### Endpoints

| Endpoint | Method | Description |
|---|---|---|
| / | GET | API status |
| /health | GET | Health check |
| /upload | POST | Upload drone/fog events |
| /drones | GET | Retrieve archived logs |

---

## Event Flow

### Step 1 — Decision System publishes event

Example:

bash ros2 topic pub /decision/status std_msgs/msg/String \ "{data: 'survivor_detected'}" --once

---

### Step 2 — Cloud Client receives the event

cloud_client.py subscribes to:

text /decision/status

and sends the event to:

text http://127.0.0.1:5000/upload

using HTTP POST requests.

---

### Step 3 — Cloud API archives the event

The Flask API stores all received events inside:

text cloud_logs.json

with timestamps and metadata.

---

## Example Stored Log

json {   "received_at": "2026-06-05T18:49:16",   "drone_id": "drone1",   "event": "survivor_detected",   "status": "uploaded",   "source": "fog_decision_system" }

---

## Technologies Used

- ROS2 Jazzy
- Python 3
- Flask
- REST API
- JSON
- PX4
- Gazebo
- Ubuntu Linux

---

## Build Instructions

### Build ROS2 package

bash cd ~/multi-drone-disaster-response colcon build --packages-select drone_node source install/setup.bash

---

## Running the Cloud API

bash cd ~/multi-drone-disaster-response/cloud-api  source venv/bin/activate  python3 app.py

Expected output:

text Running on http://127.0.0.1:5000

---

## Running the Cloud Client

bash cd ~/multi-drone-disaster-response  source install/setup.bash  ros2 run drone_node cloud_client

---

## Testing the System

### Publish a test event

bash ros2 topic pub /decision/status std_msgs/msg/String \ "{data: 'survivor_detected'}" --once

---

### Check archived logs

bash curl http://127.0.0.1:5000/drones

---

## Example Successful Output

text [CLOUD CLIENT] Sending data to cloud... [CLOUD UPLOAD] success | response=200

API response:

json {   "status": "success",   "message": "Data archived successfully" }

---

## Achievements

✅ Real-time cloud upload  
✅ REST communication between ROS2 and cloud  
✅ Event archival system  
✅ Timestamped logging  
✅ Cloud-ready architecture  
✅ Delay measurement support  
✅ JSON-based storage  
✅ Modular integration with fog layer  

---

## Future Improvements

- Deploy API to real cloud server
- Add MongoDB/PostgreSQL database
- Build web dashboard
- Add authentication/security
- Add live drone visualization
- Integrate analytics and AI monitoring

---

## Architecture Summary

text PX4 Drone     ↓ Fog Detection     ↓ Decision Node     ↓ ROS2 Topic (/decision/status)     ↓ Cloud Client     ↓ HTTP POST Flask Cloud API     ↓ JSON Cloud Storage

Based on the existing Task 5 structure and pipeline.
