# Task 6 - Real System Evaluation

This evaluation framework measures the performance of the UAV disaster response system using real ROS2 topics and node communication.

## Features

- Real latency measurement
- Mission completion timing
- Fog vs Cloud comparison
- Battery-aware evaluation
- CSV logging
- Automatic plot generation

## Files

- `metrics_collector.py`
  Collects ROS2 messages and stores metrics in CSV files.

- `analyze_results.py`
  Analyzes CSV logs and generates plots.

- `results_real/`
  Stores experiment CSV outputs.

- `plots/`
  Stores generated graphs.

## Example Metrics

- Latency
- Completion Time
- Detection Success
- Task Completion
- Battery Status

## Run Evaluation

```bash
python3 evaluation/metrics_collector.py
