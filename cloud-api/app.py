from flask import Flask, request, jsonify
from datetime import datetime
import json
import os
import time

app = Flask(__name__)

LOG_FILE = "cloud_logs.json"


def load_logs():
    if not os.path.exists(LOG_FILE):
        return []
    with open(LOG_FILE, "r") as file:
        return json.load(file)


def save_logs(logs):
    with open(LOG_FILE, "w") as file:
        json.dump(logs, file, indent=4)


@app.route("/", methods=["GET"])
def home():
    return jsonify({
        "status": "running",
        "service": "UAV Cloud API",
        "description": "Cloud archiving layer for UAV swarm disaster response events",
        "endpoints": {
            "POST /upload": "Archive drone/fog events",
            "GET /drones": "View archived logs",
            "GET /health": "Check service health"
        }
    })


@app.route("/health", methods=["GET"])
def health():
    return jsonify({
        "status": "healthy",
        "service": "cloud_api"
    })


@app.route("/upload", methods=["POST"])
def upload_data():
    received_time = time.time()
    data = request.get_json(force=True)

    sent_time = data.get("timestamp", None)
    cloud_delay = None

    if sent_time is not None:
        try:
            cloud_delay = round(received_time - float(sent_time), 4)
        except (ValueError, TypeError):
            cloud_delay = None

    log_entry = {
        "received_at": datetime.utcnow().isoformat(),
        "source": data.get("source", "unknown"),
        "drone_id": data.get("drone_id", "unknown"),
        "event": data.get("event", data.get("message", "unknown")),
        "status": data.get("status", "archived"),
        "cloud_delay_seconds": cloud_delay,
        "raw_data": data
    }

    logs = load_logs()
    logs.append(log_entry)
    save_logs(logs)

    return jsonify({
        "status": "success",
        "message": "Data archived successfully",
        "received_at": log_entry["received_at"],
        "cloud_delay_seconds": cloud_delay
    })


@app.route("/drones", methods=["GET"])
def get_drones():
    return jsonify({
        "total_logs": len(load_logs()),
        "logs": load_logs()
    })


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
