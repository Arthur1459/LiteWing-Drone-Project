import time
import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

# URI for your LiteWing drone
DRONE_URI = "udp://192.168.43.42"

def test_connection():
    print(f"Testing UDP connection to drone at '{DRONE_URI}'")

    # Initialize CRTP drivers
    print("Driver Init...")
    cflib.crtp.init_drivers()

    try:
        # Use SyncCrazyflie for a more robust connection
        print("Connection...")
        with SyncCrazyflie(DRONE_URI) as scf:
            cf = scf.cf
            print("Connected !")
            time.sleep(1)
    except Exception as e:
        print(f"Connexion Failed : {e}")

if __name__ == "__main__":
    test_connection()
