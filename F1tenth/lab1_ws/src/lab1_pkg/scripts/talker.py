#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

class Talker(Node):
    def __init__(self):
        super().__init__("talker")

        # Declare and get parameters for speec(v) and steering angle (d)
        self.declare_parameter("v", 0.0)
        self.declare_parameter("d", 0.0)

        # Create publisher for AckermannDriveStamped messages
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, "drive", 10)

        # Set a timer to publish as fast as possible
        self.timer = self.create_timer(0.01, self.publish_drive_command)

    def publish_drive_command(self):
        # Get parameters
        v = self.get_parameter("v").value
        d = self.get_parameter('d').value
        
        # Create and populate the ackermannDriveStamped message
        msg = AckermannDriveStamped()
        msg.drive.speed = v
        msg.drive.steering_angle = d

        # Publish the message
        self.drive_publisher.publish(msg)
        self.get_logger().info(f'Publishing: speed={v}, steering_angle={d}')


def main(args = None):
    rclpy.init(args=args)
    talker = Talker()

    try:
        rclpy.spin(talker)
    except KeyboardInterrupt:
        pass

    # Shutdown
    talker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()