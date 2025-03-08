import json
import os
import rclpy
from rclpy.node import Node
import paho.mqtt.client as mqtt
from example_interfaces.action import Fibonacci
from rclpy.action import ActionClient


MQTT_BROKER = os.getenv("MQTT_BROKER", "mqtt")


#MQTT_BROKER = "project_root-mqtt-1"
MQTT_TOPIC = "missions"

import time
import paho.mqtt.client as mqtt

MAX_RETRIES = 5



class RN1Client(Node):
    def __init__(self):
        super().__init__('rn1_client')
        self.client = ActionClient(self, Fibonacci, 'mission_action')

        # MQTT Setup
        for attempt in range(MAX_RETRIES):
            try:
                self.mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
                self.mqtt_client.on_message = self.on_mqtt_message
                self.mqtt_client.connect(MQTT_BROKER, 1883, keepalive=300)
                print("MQTT Connection Successful!")
                break
            except ConnectionRefusedError:
                print(f"MQTT Connection failed, retrying ({attempt+1}/{MAX_RETRIES})...")
                time.sleep(2)
        
        self.mqtt_client.subscribe(MQTT_TOPIC)
        self.mqtt_client.loop_start()

    def on_mqtt_message(self, client, userdata, message):
        mission_data = json.loads(message.payload.decode())
        goal_msg = Fibonacci.Goal()
        goal_msg.order = int(mission_data["command"])
        self.send_goal(goal_msg)

    def send_goal(self, goal_msg):
        self.client.wait_for_server()
        self.client.send_goal_async(goal_msg)
        self.get_logger().info(f"Sent mission: {goal_msg.order}")

def main(args=None):
    rclpy.init(args=args)
    node = RN1Client()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
