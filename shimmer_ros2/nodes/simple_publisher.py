import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class FastPUB(Node):
    def __init__(self):
        super().__init__('fast_pub')
        self.pub = self.create_publisher(Float32, 'emg_raw', 10)
        self.msg = Float32()
        self.create_timer(0.001, self.cb)  # 1 ms = 1 kHz

    def cb(self):
        self.msg.data += 1.0  # fake data
        self.pub.publish(self.msg)


def main():
    rclpy.init()
    rclpy.spin(FastPUB())
    rclpy.shutdown()
