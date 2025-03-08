import json
import time
import paho.mqtt.client as mqtt
from typing import Dict
from fastapi import FastAPI
from pydantic import BaseModel

app = FastAPI()

# In-memory storage for missions
missions: Dict[str, Dict] = {}

# MQTT Client Setup
'''
MQTT_BROKER = "localhost"
MQTT_TOPIC = "missions"
mqtt_client = mqtt.Client()
mqtt_client.connect(MQTT_BROKER, 1883, 60)
'''

MQTT_BROKER = "mqtt"  # Ensure this matches docker-compose service name
MQTT_TOPIC = "missions"
MQTT_PORT = 1883

mqtt_client = mqtt.Client()

# MQTT Retry Logic
MAX_RETRIES = 5
for attempt in range(MAX_RETRIES):
    try:
        mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
        mqtt_client.loop_start()  # Enables asynchronous message handling
        print("Connected to MQTT Broker")
        break
    except Exception as e:
        print(f"MQTT connection failed: {e}, retrying ({attempt+1}/{MAX_RETRIES})...")
        time.sleep(5)
else:
    raise RuntimeError("Failed to connect to MQTT broker after multiple attempts")

class Mission(BaseModel):
    id: str
    command: str

@app.post("/mission/")
def create_mission(mission: Mission):
    mission_data = mission.dict()
    mqtt_client.publish(MQTT_TOPIC, json.dumps(mission_data))
    return {"status": "Mission published to MQTT", "mission": mission_data}

@app.get("/mission/{mission_id}")
def get_mission(mission_id: str):
    if mission_id not in missions:
        raise HTTPException(status_code=404, detail="Mission not found")
    
    return missions[mission_id]