# Fog-Assisted UAV Swarm for Low-Latency Disaster Response

## Overview

This repository contains the implementation of a **fog-assisted UAV swarm architecture** for autonomous disaster-response missions. The system combines **PX4 SITL**, **Gazebo Harmonic**, **ROS 2 Humble**, fog computing, and cloud services to enable real-time victim detection, autonomous mission coordination, dynamic area partitioning, and post-mission data archival.

The project was developed as a Computer Engineering graduation project at **Birzeit University**.

---

## System Architecture

The proposed architecture consists of three cooperative layers:

### UAV Layer
- Autonomous flight control using PX4 SITL
- Camera image acquisition
- Mission execution
- Telemetry publishing
- Communication with the fog server

### Fog Layer
- Victim detection (YOLOv8)
- Mission coordination
- Dynamic area partitioning
- Coverage monitoring
- Task assignment and reassignment
- Battery monitoring
- Failure detection and recovery

### Cloud Layer
- Mission-event archival
- REST API for data storage
- Post-mission analysis support

---

## Main Features

- Multi-UAV autonomous search missions
- Fog-assisted victim detection
- Dynamic search-area partitioning
- Coverage monitoring
- Battery simulation
- UAV failure recovery
- Cloud-based mission archival
- Automatic evaluation and result analysis

---

## Technologies Used

- ROS 2 Humble
- PX4 SITL
- Gazebo Harmonic
- Python
- Flask
- OpenCV
- YOLOv8
- MAVLink
- Micro XRCE-DDS
- Pandas
- Matplotlib

---

## Repository Structure

```text
multi-drone-disaster-response/
│
├── src/                      # ROS 2 packages
├── cloud-api/                # Flask cloud service
├── evaluation/
│   ├── analyze_results.py
│   ├── battery_logger.py
│   ├── results_real/
│   ├── Figures/
│   └── Tables/
│
├── Docs/
├── README.md
└── ...
```

---

## Experimental Evaluation

The proposed architecture was evaluated through four experiments:

1. **Processing Mode Comparison**
   - Fog-assisted processing
   - Cloud processing
   - Local onboard processing

2. **Search Area Scalability**
   - Different mission sizes
   - Coverage analysis
   - Battery consumption
   - Mission duration

3. **Drone Failure Recovery**
   - Dynamic area reassignment
   - Autonomous recovery
   - Coverage preservation

4. **Resource Utilization**
   - Edge workload
   - Fog workload
   - Cloud workload

---

## Running the Evaluation

The evaluation script automatically generates all figures and tables from the recorded experiment data.

```bash
python3 evaluation/analyze_results.py
```

Generated outputs include:

- Battery comparison
- Latency comparison
- Response time
- Coverage analysis
- Search-area scaling
- Resource utilization
- Drone failure recovery
- Summary tables

---

## Results Summary

The experimental evaluation demonstrated that the proposed fog-assisted architecture:

- Reduced victim-detection latency compared with cloud processing.
- Outperformed local onboard processing for real-time inference.
- Reduced UAV battery consumption.
- Maintained high search coverage across different mission sizes.
- Successfully recovered from UAV failures using dynamic area reassignment.
- Distributed computational tasks according to the intended three-tier architecture.

---

## Limitations

The current implementation was evaluated in a **Software-in-the-Loop (SITL)** environment.

The project does not yet include:

- Real UAV deployment
- Thermal camera support
- Multiple fog servers
- Real cloud infrastructure
- Severe weather simulation

These aspects are proposed as future work.

---

## Future Work

Potential extensions include:

- Real UAV implementation
- Thermal imaging integration
- Multi-fog architectures
- Reinforcement-learning-based task allocation
- Larger UAV swarms
- LiDAR and multispectral sensors
- Real cloud deployment
- Adverse weather simulation

---

## Citation

If you use this repository in academic work, please cite the corresponding graduation thesis.

---

## Authors

**Doaa Hatu**
**Mai Beitnoba**
**Lina Abureesh**

Computer Engineering Department

Birzeit University

---

## License

This project is intended for academic and research purposes.
