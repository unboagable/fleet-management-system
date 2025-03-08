import rclpy
from rclpy.node import Node
from example_interfaces.action import Fibonacci
from rclpy.action import ActionServer

class RN2Server(Node):
    def __init__(self):
        super().__init__('rn2_server')
        self.server = ActionServer(self, Fibonacci, 'mission_action', self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info(f"Received mission: {goal_handle.request.order}")
        goal_handle.succeed()
        return Fibonacci.Result()

def main(args=None):
    rclpy.init(args=args)
    node = RN2Server()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
