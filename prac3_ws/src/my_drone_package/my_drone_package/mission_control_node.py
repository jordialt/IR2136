import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix
from pymavlink import mavutil
import time
import sys

class MissionControlNode(Node):
    def __init__(self, target_lat, target_lon):
        super().__init__('mission_control_node')
        self.gps_pub = self.create_publisher(NavSatFix, 'target_gps', 10)
        self.battery_sub = self.create_subscription(Float32, 'battery_status', self.check_battery, 10)
        self.connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')
        self.connection.wait_heartbeat()
        self.get_logger().info("Connected to SITL.")
        self.publish_target_gps(target_lat, target_lon)
        self.set_mode("GUIDED")
        time.sleep(2)
        self.arm_vehicle()  # Arm the drone at the beginning
        time.sleep(2)
        self.takeoff(10)
        time.sleep(2)
        self.publish_target_gps(target_lat, target_lon)
        
          # Set to GUIDED mode
    
    def publish_target_gps(self, lat, lon):
        gps_msg = NavSatFix()
        gps_msg.latitude = lat
        gps_msg.longitude = lon
        self.gps_pub.publish(gps_msg)
        self.get_logger().info(f"Published target GPS: {lat}, {lon}")
    
    def check_battery(self, msg):
        if msg.data <= 20.0:
            self.get_logger().warn("Low battery! Initiating landing.")
            self.connection.mav.command_long_send(
                self.connection.target_system, self.connection.target_component,
                mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0
            )

    def arm_vehicle(self):
        """Arms the drone."""
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # Confirmation
            1,  # Arm
            0, 0, 0, 0, 0, 0
        )
        self.get_logger().info("Drone armed.")

    def takeoff(self, altitude):
        """Initiates takeoff."""
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,  # Confirmation
            0, 0, 0, 0, 0, 0, altitude  # Desired altitude
        )
        self.get_logger().info(f"Taking off to {altitude} meters.")

    def set_mode(self, mode):
        """Sets the flight mode for the drone."""
        # Request the mode change to GUIDED
        self.connection.set_mode(mode)
        self.get_logger().info(f"Mode changed to {mode}.")

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) < 3:
        print("Usage: mission_control_node.py <latitude> <longitude>")
        return
    lat, lon = float(sys.argv[1]), float(sys.argv[2])
    node = MissionControlNode(lat, lon)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

