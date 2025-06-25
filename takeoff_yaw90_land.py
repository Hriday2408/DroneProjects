from pymavlink import mavutil
import time
import threading

# Connect to the flight controller
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)
master.wait_heartbeat()
print("Heartbeat received")

# Set mode to GUIDED
mode = 'GUIDED'
mode_id = master.mode_mapping()[mode]
master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id
)
print("Mode set to GUIDED")
time.sleep(2)

# Function to monitor attitude and disarm if pitch or roll exceed 20 degrees
def monitor_attitude():
    print("Starting attitude monitoring...")
    time.sleep(3)  # Let motors spin up
    while True:
        msg = master.recv_match(type='ATTITUDE', blocking=True, timeout=1)
        if msg:
            pitch = msg.pitch * 57.2958  # Convert from radians to degrees
            roll = msg.roll * 57.2958
            if abs(pitch) > 20 or abs(roll) > 20:
                print("Detected dangerous angle! Pitch: {:.2f} deg, Roll: {:.2f} deg".format(pitch, roll))
                print("Disarming motors for safety...")
                master.mav.command_long_send(
                    master.target_system,
                    master.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0,
                    0, 0, 0, 0, 0, 0, 0
                )
                break

# Start the attitude monitoring thread
monitor_thread = threading.Thread(target=monitor_attitude, daemon=True)
monitor_thread.start()

# Arm the drone
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0
)
print("Arming sent")
time.sleep(3)

# Takeoff to 2 meters
altitude = 2
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0,
    0, 0, 0, 0, 0, 0,
    altitude
)
print("Takeoff command sent to 2 meters")
time.sleep(8)

# Yaw control to 90 degrees
yaw_deg = 90
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_CONDITION_YAW,
    0,
    yaw_deg,   # Target yaw angle
    20,        # Yaw speed (deg/s)
    1,         # Direction (1 = CW, -1 = CCW)
    1,         # Relative (1 = relative, 0 = absolute)
    0, 0, 0
)
print("Yaw set to 90 degrees")
time.sleep(5)

# Land
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_LAND,
    0,
    0, 0, 0, 0, 0, 0, 0
)
print("Landing the drone")
time.sleep(10)

# Final disarming (in case it wasn’t already disarmed)
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    0, 0, 0, 0, 0, 0, 0
)
print("Drone disarmed")
