import DroneMonitor as dm
from time import time, sleep

# ---- Setup ---- #
dm.ACTIVATE_CONTROL = True
dm.MY_PILOT = True
dm.CONTROLLER = False
# --------------- #

# ---- Info ---- #
# dm.joystick_a0 -> Right - Left/Right (yaw) (-1 -> 1)
# dm.joystick_a1 -> Right - Up/Down (thrust) (-1 -> 1)
# dm.joystick_a2 -> Left - Left/Right (roll) (-1 -> 1)
# dm.joystick_a3 -> Left - Up/Down (pitch) (-1 -> 1)
# dm.pitch, dm.roll, dm.yaw -> Mesure d'angle du drone
# dm.key_inputs -> Entrée clavier
# -------------- #

# --- Programmes de pilotage :
def pilote_base(cf):
    dm.StartProcedure(cf)

    print("Pilote : pilote_base")
    while dm.running:
        print()
        print(f"Roll : {dm.roll}, Pitch : {dm.pitch}, Yaw : {dm.yaw}")
        print(f"Joystick Thrust : {dm.joystick_a1}, Joystick Roll : {dm.joystick_a2}, Joystick Pitch : {dm.joystick_a3}")

        dm.roll_cmd, dm.pitch_cmd, dm.yaw_cmd, dm.thrust_cmd = 0, 0, 0, 0
        cf.commander.send_setpoint(dm.roll_cmd, dm.pitch_cmd, dm.yaw_cmd, dm.thrust_cmd)
        sleep(0.01)

def slow_turning_motors(cf):
    dm.StartProcedure(cf)

    print("Pilote : slow_turning_motors")
    while dm.running:
        dm.roll_cmd, dm.pitch_cmd, dm.yaw_cmd, dm.thrust_cmd = 0, 0, 0, 0

        dm.thrust_cmd = 20_000
        cf.commander.send_setpoint(0, 0, 0, dm.thrust_cmd)
        sleep(0.01)

def yaw_rotation(cf):
    dm.StartProcedure(cf)

    t_init = time()
    yaw_init = dm.yaw # Mesure du yaw initial

    print("Pilote : yaw_rotation")
    while dm.running:
        dm.roll_cmd, dm.pitch_cmd = 0, 0

        dt = time() - t_init
        dm.yaw_cmd = (yaw_init + 5 * dt) % 360 # x ° / s

        dm.thrust_cmd = 20_000

        cf.commander.send_setpoint(0, 0, dm.yaw_cmd, dm.thrust_cmd)
        sleep(0.01)

def vertical_thrust(cf):
    dm.StartProcedure(cf)

    print("Pilote : vertical_thrust")
    while dm.running:
        dm.roll_cmd, dm.pitch_cmd, dm.yaw_cmd, dm.thrust_cmd = 0, 0, 0, 0

        cmd_thrust_input = max(0., dm.joystick_a1)
        dm.thrust_cmd = 40_000 * cmd_thrust_input

        cf.commander.send_setpoint(0, 0, 0, dm.thrust_cmd)
        sleep(0.01)

def takeoff_landing(cf):
    dm.StartProcedure(cf)

    print("Pilote : vertical_thrust")

    takeoff_duration = 1 # s
    landing_duration = 1 # s
    t_init = time()

    while dm.running:
        dm.roll_cmd, dm.pitch_cmd, dm.yaw_cmd, dm.thrust_cmd = 0, 0, 0, 0

        duration = time() - t_init

        if duration < takeoff_duration:
            dm.thrust_cmd = 30_000
        elif duration < takeoff_duration + landing_duration:
            dm.thrust_cmd = 15_000
        else:
            dm.thrust_cmd = 0

        cf.commander.send_setpoint(0, 0, 0, dm.thrust_cmd)
        sleep(0.01)

def pilote_full(cf):
    dm.StartProcedure(cf)

    while dm.running:
        dm.roll_cmd, dm.pitch_cmd, dm.yaw_cmd, dm.thrust_cmd = 0, 0, 0, 0

        if not dm.CONTROLLER:
            cmd_thrust_input = 0.65 if dm.key_inputs["Z"] else (-1 if dm.key_inputs["S"] else 0)
            cmd_roll_input = 1 if dm.key_inputs["RIGHT"] else (-1 if dm.key_inputs["LEFT"] else 0)
            cmd_pitch_input = 1 if dm.key_inputs["UP"] else (-1 if dm.key_inputs["DOWN"] else 0)
        else:
            cmd_thrust_input = (1 + dm.joystick_a1) / 2  # 0 -> 1
            cmd_roll_input = dm.joystick_a2
            cmd_pitch_input = -dm.joystick_a3

        min_thrust = 15_000 - max(0., -cmd_thrust_input) * 15_000
        max_thrust = 50_000
        thrust = max(0., cmd_thrust_input) * (max_thrust - min_thrust) + min_thrust
        roll = 15 * cmd_roll_input
        pitch = 15 * cmd_pitch_input

        pitch_cmd, roll_cmd, yaw_cmd, thrust_cmd = pitch, roll, 0, thrust
        cf.commander.send_setpoint(int(roll), int(pitch), 0, int(thrust))
        sleep(0.01)

# --- Lance le programme sur le drone
dm.code_pilote = pilote_base # Choisi le programme de pilotage
dm.main()

