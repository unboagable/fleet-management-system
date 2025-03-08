import json
import time
import paho.mqtt.client as mqtt
from fastapi import FastAPI
from pydantic import BaseModel

app = FastAPI()

# MQTT Client Setup
'''
MQTT_BROKER = "localhost"
MQTT_TOPIC = "missions"
mqtt_client = mqtt.Client()
mqtt_client.connect(MQTT_BROKER, 1883, 60)
'''

MQTT_BROKER = "mqtt"  # Ensure this matches docker-compose service name
MQTT_PORT = 1883

mqtt_client = mqtt.Client()

# Retry loop
for _ in range(5):
    try:
        mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
        print("Connected to MQTT Broker")
        break
    except Exception as e:
        print(f"MQTT connection failed: {e}, retrying...")
        time.sleep(5)  # Wait 5 seconds before retrying

class Mission(BaseModel):
    id: str
    command: str

@app.post("/mission/")
def create_mission(mission: Mission):
    mission_data = mission.dict()
    mqtt_client.publish(MQTT_TOPIC, json.dumps(mission_data))
    return {"status": "Mission published to MQTT", "mission": mission_data}
