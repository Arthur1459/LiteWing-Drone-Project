import DroneMonitor as dm
from time import time, sleep

# ---- Setup ---- #
dm.ACTIVATE_CONTROL = True
dm.MY_PILOT = True
dm.CONTROLLER = True
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
        print(f"Roll : {dm.roll_measured}, Pitch : {dm.pitch_measured}, Yaw : {dm.yaw_measured}")
        print(f"Joystick Thrust : {dm.joystick_a1}, Joystick Roll : {dm.joystick_a2}, Joystick Pitch : {dm.joystick_a3}")

        cf.commander.send_setpoint(0, 0, 0, 0) # Order required to keep connexion on
        sleep(0.2) # Maximum time between 2 orders

def slow_turning_motors(cf):
    dm.StartProcedure(cf)

    init_roll, init_pitch, init_yaw = dm.roll_measured, dm.pitch_measured, dm.yaw_measured
    print("Pilote : slow_turning_motors")
    while dm.running:
        dm.roll_cmd, dm.pitch_cmd, dm.yaw_cmd, dm.thrust_cmd = 0, 0, 0, 0 # Reset all commands

        dm.thrust_cmd = 15_000

        print(init_yaw, dm.yaw_measured)
        cf.commander.send_setpoint(init_roll, init_pitch, 0, dm.thrust_cmd)
        sleep(0.01)

def yaw_rotation(cf):
    dm.StartProcedure(cf)

    init_roll, init_pitch, init_yaw = dm.roll_measured, dm.pitch_measured, dm.yaw_measured
    print("Pilote : yaw_rotation")
    while dm.running:
        dm.roll_cmd, dm.pitch_cmd = 0, 0

        dm.yaw_cmd = 90  # command ° / s

        dm.thrust_cmd = 15_000

        cf.commander.send_setpoint(dm.roll_measured, dm.pitch_measured, dm.yaw_cmd, dm.thrust_cmd)
        sleep(0.01)

def vertical_thrust(cf):
    dm.StartProcedure(cf)

    init_roll, init_pitch, init_yaw = dm.roll_measured, dm.pitch_measured, dm.yaw_measured
    print("Pilote : vertical_thrust")
    while dm.running:
        dm.roll_cmd, dm.pitch_cmd, dm.yaw_cmd, dm.thrust_cmd = 0, 0, 0, 0

        cmd_thrust_input = max(0., dm.joystick_a1)
        dm.thrust_cmd = 40_000 * cmd_thrust_input
        print(cmd_thrust_input, dm.thrust_cmd)

        cf.commander.send_setpoint(init_roll, init_pitch, 0, max(0, int(dm.thrust_cmd)))
        sleep(0.01)

def pilote_full(cf):
    dm.StartProcedure(cf)

    init_roll, init_pitch, init_yaw = dm.roll_measured, dm.pitch_measured, dm.yaw_measured
    while dm.running:
        dm.roll_cmd, dm.pitch_cmd, dm.thrust_cmd = 0, 0, 0

        if not dm.CONTROLLER:
            cmd_thrust_input = 0.65 if dm.key_inputs["Z"] else (-1 if dm.key_inputs["S"] else 0)
            cmd_roll_input = 1 if dm.key_inputs["RIGHT"] else (-1 if dm.key_inputs["LEFT"] else 0)
            cmd_pitch_input = 1 if dm.key_inputs["UP"] else (-1 if dm.key_inputs["DOWN"] else 0)
        else:
            cmd_thrust_input = max(0., dm.joystick_a1)  # 0 -> 1
            cmd_roll_input = dm.joystick_a2
            cmd_pitch_input = dm.joystick_a3

        dm.thrust_cmd = 40_000 * cmd_thrust_input
        dm.roll_cmd = 15 * cmd_roll_input
        dm.pitch_cmd = 15 * cmd_pitch_input

        cf.commander.send_setpoint(init_roll + int(dm.roll_cmd), init_pitch + int(dm.pitch_cmd), 0, max(0, int(dm.thrust_cmd)))
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

def takeoff_landing_stabilization(cf):
    dm.StartProcedure(cf)

    print("Pilote : vertical_thrust")

    takeoff_duration = 2.5 # s
    landing_duration = 0.5 # s
    phase = 1 # s
    t_init = time()

    init_roll, init_pitch, init_yaw = dm.roll_measured, dm.pitch_measured, dm.yaw_measured
    while dm.running:
        dm.roll_cmd, dm.pitch_cmd, dm.yaw_cmd, dm.thrust_cmd = 0, 0, 0, 0

        duration = time() - t_init

        # Stabilization
        if abs(dm.roll_measured - init_roll) > 0.5:
            if (dm.roll_measured < init_roll): dm.roll_cmd = 5
            if (dm.roll_measured > init_roll): dm.roll_cmd = -5
        if abs(dm.pitch_measured - init_pitch) > 0.5:
            if (dm.pitch_measured < init_pitch) : dm.pitch_cmd = 5
            if (dm.pitch_measured > init_pitch) : dm.pitch_cmd = -5

        if duration < takeoff_duration:
            dm.thrust_cmd = 27_000
            if duration < phase :
                dm.pitch_cmd += 10
            else:
                dm.pitch_cmd += -10
        elif duration < takeoff_duration + landing_duration:
            dm.thrust_cmd = 15_000
        else:
            dm.thrust_cmd = 0

        # Static Correction
        dm.pitch_cmd += 0
        dm.roll_cmd += 3

        cf.commander.send_setpoint(init_roll + int(dm.roll_cmd), init_pitch + int(dm.pitch_cmd), 0, max(0, int(dm.thrust_cmd)))
        sleep(0.01)

# --- Lance le programme sur le drone
dm.code_pilote = takeoff_landing_stabilization # Choisi le programme de pilotage
dm.main()

