from pymavlink import mavutil
import time

# Connect to SITL
connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')
connection.wait_heartbeat()  # Wait for the first heartbeat to confirm connection
print("Connected to SITL.")

def read_sensors(connection):
    """Reads the drone's sensor data (GPS position)."""
    msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    if msg:
        latitude = msg.lat / 1e7
        longitude = msg.lon / 1e7
        altitude = msg.relative_alt / 1000.0
        print(f"Position: Lat={latitude}, Lon={longitude}, Alt={altitude} m")
    else:
        print("Failed to read sensor data.")

def arm_vehicle(connection):
    """Arms the drone."""
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,  # Confirmation
        1,  # Arm
        0, 0, 0, 0, 0, 0
    )
    print("Drone armed.")

def takeoff(connection, altitude):
    """Initiates takeoff."""
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,  # Confirmation
        0, 0, 0, 0, 0, 0, altitude  # Desired altitude
    )
    print(f"Taking off to {altitude} meters.")

def set_mode(connection, mode):
    """Sets the flight mode for the drone."""
    connection.set_mode(mode)
    print(f"Mode changed to {mode}.")

def go_to_position(connection, lat, lon, alt):
    """Moves the drone to a specific latitude, longitude, and altitude."""
    connection.mav.set_position_target_global_int_send(
        0,  # Timestamp
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # Relative altitude frame
        0b110111111000,  # Ignore velocity and acceleration
        int(lat * 1e7),  # Latitude as integer
        int(lon * 1e7),  # Longitude as integer
        alt,  # Desired altitude in meters
        0, 0, 0,  # Velocities
        0, 0, 0,  # Accelerations
        0, 0  # Yaw and yaw rate
    )
    print(f"Moving drone to Lat: {lat}, Lon: {lon}, Alt: {alt} m.")

def read_battery_status(connection):
    """Reads the battery status of the drone."""
    msg = connection.recv_match(type='BATTERY_STATUS', blocking=True, timeout=5)
    if msg:
        voltage = msg.voltages[0] / 1000.0
        current = msg.current_battery / 100.0
        remaining = msg.battery_remaining
        print(f"Battery: {voltage:.2f} V, Current: {current:.2f} A, Remaining: {remaining}%")
        return remaining
    else:
        print("Failed to get battery status.")
        return None

def land_vehicle(connection):
    """Sends the landing command to the drone."""
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,  # Confirmation
        0, 0, 0, 0, 0, 0, 0
    )
    print("Landing command sent.")

# Main sequence
arm_vehicle(connection)
takeoff(connection, 10)
time.sleep(3)
set_mode(connection, 'GUIDED')
go_to_position(connection, 37.7749, -122.4194, 10)  # Example coordinates

while True:
    battery_remaining = read_battery_status(connection)
    if battery_remaining is not None and battery_remaining <= 20:
        print("Low battery! Initiating emergency landing.")
        land_vehicle(connection)
        break
    
    # Perform sensor reading
    read_sensors(connection)
    
    # Check every 5 seconds
    time.sleep(5)
