import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16, Bool
from bsactuator import bsactuator
import time
import logging

class BsActuatorNode(Node):

    def __init__(self):
        super().__init__('bsactuator_ros')
        self.model = "50mm02"

        # Set logging level
        self.get_logger().set_level(logging.DEBUG)

        # Initialize the bsactuator
        # self.ba = bsactuator.BsActuator("/dev/bambooshoot_actuator", 115200, self.model)

        # Setting subscribers
        self.get_logger().info("Setting subscribers...")
        self.create_subscription(Int16, 'bsactuator/set_length', self.set_length_callback, 10)
        self.create_subscription(Bool, 'bsactuator/hold', self.hold_callback, 10)
        self.create_subscription(Bool, 'bsactuator/release', self.release_callback, 10)
        self.create_subscription(Bool, 'bsactuator/reset', self.reset_callback, 10)
        self.create_subscription(Bool, 'bsactuator/stop', self.stop_callback, 10)

        # Setting publishers
        self.get_logger().info("Setting publishers...")
        self.publisher_get_length = self.create_publisher(Int16, 'bsactuator/length', 10)

        self.moving = False
        self.current_length = 0
        self.dist_length = 0
        self.timeout = time.time()
        self.waitsec = 0.5

        # Timer for periodic publish
        self.timer = self.create_timer(0.1, self.publish_get_length)
        self.count = 0

        self.get_logger().info("Bambooshoot actuator started.")

    def set_length_callback(self, msg):
        self.get_logger().info(f"Dist length: {msg.data}mm")
        self.dist_length = msg.data

        if self.model == "50mm02":
            self.moving = True
            # self.ba.set_length(msg.data)
            self.get_logger().info(f"setting actuator length: {msg.data} mm")

    def hold_callback(self, msg):
        if msg.data:
            # self.ba.hold()
            self.get_logger().info(f"actuator hold")

    def release_callback(self, msg):
        if msg.data:
            # self.ba.release()
            self.get_logger().info(f"actuator release")

    def reset_callback(self, msg):
        if msg.data:
            # self.ba.reset()
            self.get_logger().info(f"actuator reset")

    def stop_callback(self, msg):
        if msg.data:
            # self.ba.stop()
            self.get_logger().info(f"actuator stop")

    def publish_get_length(self):
        # self.current_length = int(self.ba.get_length())
        self.current_length = int(333)
        self.publisher_get_length.publish(Int16(data=self.current_length))
        if self.moving and abs(self.current_length - self.dist_length) < 10:
            self.moving = False
            self.get_logger().info("Goal reached.")


def main(args=None):
    rclpy.init(args=args)
    node = BsActuatorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
