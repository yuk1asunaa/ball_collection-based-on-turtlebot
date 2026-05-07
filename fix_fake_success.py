import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

class FixHacker(Node):
    def __init__(self):
        super().__init__('fix_hacker')
        self.get_logger().info('Draining maps')
        self.sub = self.create_subscription(OccupancyGrid, '/density_map', self.cb, 10)

    def cb(self, msg):
        pass

def main():
    rclpy.init()
    node = FixHacker()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
