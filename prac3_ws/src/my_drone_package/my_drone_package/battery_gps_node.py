import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix
from pymavlink import mavutil
import time

class BatteryGPSNode(Node):
    def __init__(self):
        super().__init__('battery_gps_node')
        
        # Initialize MAVLink connection
        self.connection = None
        self.connect_to_drone()

        # Create ROS 2 publishers and subscribers
        self.battery_pub = self.create_publisher(Float32, 'battery_status', 10)
        self.gps_sub = self.create_subscription(NavSatFix, 'target_gps', self.send_gps_to_drone, 10)
        
        # Create a timer to periodically publish battery status
        self.create_timer(1.0, self.publish_battery_status)  # publish every second
    
    def connect_to_drone(self):
        """Attempts to connect to the drone using MAVLink."""
        try:
            self.connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')
            self.connection.wait_heartbeat()  # Wait for the first heartbeat to confirm connection
            self.get_logger().info("Connected to SITL.")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to the drone: {e}")
            time.sleep(5)  # Wait for 5 seconds before retrying
            self.connect_to_drone()  # Retry connection
    
    def publish_battery_status(self):
        """Reads the battery status and publishes it."""
        if self.connection is None:
            self.get_logger().warn("No connection to drone, attempting to reconnect...")
            self.connect_to_drone()  # Attempt reconnection if connection is lost

        try:
            # Use non-blocking call to get battery status
            msg = self.connection.recv_match(type='BATTERY_STATUS', blocking=False)
            if msg:
                remaining_battery = msg.battery_remaining
                battery_msg = Float32()
                battery_msg.data = float(remaining_battery)
                self.battery_pub.publish(battery_msg)
                self.get_logger().info(f"Published battery status: {remaining_battery}%")
            else:
                self.get_logger().warn("Failed to get battery status, retrying...")
                # Retry after a small delay
                time.sleep(1)
        except Exception as e:
            self.get_logger().error(f"Error retrieving battery status: {e}")
            self.connection = None  # Mark connection as lost
            time.sleep(5)  # Wait before attempting to reconnect
            self.connect_to_drone()  # Reconnect if there was an error
    
    def send_gps_to_drone(self, msg):
        """Receives a GPS position and sends it to the drone."""
        if self.connection is None:
            self.get_logger().warn("No connection to drone, attempting to reconnect...")
            self.connect_to_drone()  # Attempt reconnection if connection is lost

        lat, lon = msg.latitude, msg.longitude
        altitude = 10  # Set altitude to 10 meters as an example
        try:
            # Send the GPS position to the drone
            self.connection.mav.set_position_target_global_int_send(
                0,  # Timestamp
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # Relative altitude frame
                0b110111111000,  # Ignore velocity and acceleration
                int(lat * 1e7),  # Latitude as integer
                int(lon * 1e7),  # Longitude as integer
                altitude,  # Desired altitude
                0, 0, 0,  # Velocities
                0, 0, 0,  # Accelerations
                0, 0  # Yaw and yaw rate
            )
            self.get_logger().info(f"Sent GPS position to drone: Lat={lat}, Lon={lon}, Alt={altitude}m")
            # Add a small delay to avoid conflicting commands
            time.sleep(1)  # Wait a little before trying to read battery status again
        except Exception as e:
            self.get_logger().error(f"Error sending GPS data to the drone: {e}")
            self.connection = None  # Mark connection as lost
            time.sleep(5)  # Wait before attempting to reconnect
            self.connect_to_drone()  # Reconnect if there was an error

def main(args=None):
    rclpy.init(args=args)
    node = BatteryGPSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

