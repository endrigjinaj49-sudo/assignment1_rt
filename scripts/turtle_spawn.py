#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn

class SimpleSpawner(Node):
    def __init__(self):
        super().__init__('simple_spawner')
        self.spawn_client = self.create_client(Spawn, '/spawn')

        # Attende che il servizio spawn sia disponibile
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("waiting for /spawn...")

        # Richiesta per spawn
        req = Spawn.Request()
        req.x = 7.0
        req.y = 7.0
        req.theta = 0.0
        req.name = "turtle2"

        future = self.spawn_client.call_async(req)
        future.add_done_callback(self.spawn_callback)

    def spawn_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Spawned turtle: {response.name}")
        except Exception as e:
            self.get_logger().error(f"Error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = SimpleSpawner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

