import rclpy
from rclpy.node import Node
import json
import paho.mqtt.client as mqtt
from example_interfaces.action import Fibonacci
from rclpy.action import ActionClient

MQTT_BROKER = "localhost"
MQTT_TOPIC = "missions"

class RN1Client(Node):
    def __init__(self):
        super().__init__('rn1_client')
        self.client = ActionClient(self, Fibonacci, 'mission_action')

        # MQTT Setup
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_message = self.on_mqtt_message
        self.mqtt_client.connect(MQTT_BROKER, 1883, 60)
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
