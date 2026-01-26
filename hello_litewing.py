import time
import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

# URI for your LiteWing drone
DRONE_URI = "udp://192.168.43.42"

# Initialize CRTP drivers
cflib.crtp.init_drivers()

# Create Crazyflie instance
with SyncCrazyflie(DRONE_URI) as scf:
    connection = scf.cf

    # First send zero setpoint to unlock safety and arm drone
    print("Sending zero setpoint to unlock safety...")
    connection.commander.send_setpoint(0, 0, 0, 0)
    time.sleep(0.1)

    # Flight parameters
    roll = 0.0
    pitch = 0.0
    yaw = 0
    thrust = 20_000  # Thrust value is 10_000 minimum and 60_000 maximum
    # Start motors
    print("Starting motors at minimum speed...")
    connection.commander.send_setpoint(roll, pitch, yaw, thrust)
    #print(connection.log.log_blocks)
    time.sleep(1)

    # Stop the motors
    print("Stopping motors...")
    connection.commander.send_setpoint(0, 0, 0, 0)
    time.sleep(0.1)

