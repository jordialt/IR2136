import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix
from pymavlink import mavutil
import time

class BatteryGPSNode(Node):
    def __init__(self):
        super().__init__('battery_gps_node')
        self.battery_pub = self.create_publisher(Float32, 'battery_status', 10)
        self.gps_sub = self.create_subscription(NavSatFix, 'target_gps', self.send_gps_to_drone, 10)
        self.connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')
        self.connection.wait_heartbeat()
        self.get_logger().info("Connected to SITL.")
        self.create_timer(5, self.publish_battery_status)
    
    def publish_battery_status(self):
        msg = self.connection.recv_match(type='BATTERY_STATUS', blocking=True, timeout=5)
        if msg:
            battery_remaining = msg.battery_remaining
            self.get_logger().info(f"Battery: {battery_remaining}%")
            battery_msg = Float32()
            battery_msg.data = float(battery_remaining)
            self.battery_pub.publish(battery_msg)
        else:
            self.get_logger().warn("Failed to get battery status.")
    
    def send_gps_to_drone(self, msg):
        lat, lon = msg.latitude, msg.longitude
        self.connection.mav.set_position_target_global_int_send(
            0, self.connection.target_system, self.connection.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            0b110111111000, int(lat * 1e7), int(lon * 1e7), 10,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        self.get_logger().info(f"Sent target GPS: {lat}, {lon}")

def main(args=None):
    rclpy.init(args=args)
    node = BatteryGPSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
