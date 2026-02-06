import pygame
import sys
import time
import threading
import hid
import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from math import sin

# --- Configuration ---
KEYBOARD = True # On utilise le keyboard comme entré PAR DEFAUT (Ne pas changer)
CONTROLLER = False # On utilise un controller comme entré OVERRIDE KEYBOARD
ACTIVATE_CONTROL = True # Pour utiliser le mode manuel ou automatique
MANUAL_CONTROL = True # Pour choisir comment on commande les moteurs
MY_PILOT = False # Pour utiliser son propre pilote

# NETWORK LiteWing_1020ABE203.. / PSWD : 12345678
DRONE_URI = "udp://192.168.43.42"
CONTROLLER_VID = 0x3285
CONTROLLER_PID = 0xc03

# --- Variables globales ---
running = True
battery_voltage = 0.0
pitch, roll, yaw = 0, 0, 0
pitch_cmd, roll_cmd, yaw_cmd, thrust_cmd = 0, 0, 0, 0
position_z = 0.0
joystick_a1 = 0.0  # Right Up/Down (thrust)
joystick_a2 = 0.0  # Left Left/Right (roll)
joystick_a3 = 0.0  # Left Up/Down (pitch)

# --- Initialisation Pygame ---
key_inputs = {"UP": 0., "DOWN": 0., "RIGHT": 0., "LEFT": 0., "SPACE": 0., "Z": 0., "S": 0.}
pygame.init()
screen = pygame.display.set_mode((800, 600))
pygame.display.set_caption("Drone Monitor")
font = pygame.font.SysFont("Arial", 18)
clock = pygame.time.Clock()

# --- Fonctions pour le drone ---
def log_data_callback(timestamp, data, logconf):
    """Callback pour les données du drone (copié de battery_voltage_read.py et position-tracking-graph.py)."""
    global battery_voltage, pitch, roll, yaw, position_z
    if 'pm.vbat' in data: battery_voltage = data['pm.vbat']
    if 'stateEstimate.pitch' in data: pitch = data['stateEstimate.pitch']
    if 'stateEstimate.roll' in data: roll = data['stateEstimate.roll']
    if 'stateEstimate.yaw' in data: yaw = data['stateEstimate.yaw']
    if 'stateEstimate.z' in data: position_z = data['stateEstimate.z']

def setup_drone_logging(cf):
    """Configure le logging pour le drone (copié de battery_voltage_read.py)."""
    log_conf = LogConfig(name='DroneData', period_in_ms=100)
    log_conf.add_variable('pm.vbat', 'float')
    log_conf.add_variable('stateEstimate.pitch', 'float')
    log_conf.add_variable('stateEstimate.roll', 'float')
    log_conf.add_variable('stateEstimate.yaw', 'float')
    log_conf.add_variable('stateEstimate.z', 'float')
    cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(log_data_callback)
    log_conf.start()

# --- Fonctions pour le contrôleur (copié exactement de height-hold-joystick.py) ---
def read_controller():
    """Lit les données du contrôleur (copié exactement de height-hold-joystick.py)."""
    global joystick_a1, joystick_a2, joystick_a3, running

    try:
        device = hid.device()
        device.open(CONTROLLER_VID, CONTROLLER_PID)
        print(f"Contrôleur connecté: {device.get_product_string()}")

        axis_right_updown = 6
        axis_left_leftright = 3
        axis_left_updown = 4

        while running:
            data = device.read(64)
            if data:
                # Conversion EXACTE depuis height-hold-joystick.py
                joystick_a1 = round(((255 - data[axis_right_updown]) - 128) / 128, 2)  # Right Up/Down
                joystick_a2 = round((data[axis_left_leftright] - 128) / 128, 2)         # Left Left/Right
                joystick_a3 = round((data[axis_left_updown] - 128) / 128, 2)           # Left Up/Down
            else:
                print("Error: no data received from controller")
            time.sleep(0.01)

    except Exception as e:
        print(f"Erreur avec le contrôleur: {e}")
    finally:
        if 'device' in locals():
            device.close()

def send_manual_drone_commands(cf):
    """Envoie les commandes au drone (copié exactement de height-hold-joystick.py)."""
    global joystick_a1, joystick_a2, joystick_a3, running, key_inputs, CONTROLLER, KEYBOARD
    global pitch_cmd, roll_cmd, yaw_cmd, thrust_cmd

    cmd_thrust_input, cmd_roll_input, cmd_pitch_input = 0, 0, 0
    while running:
        if KEYBOARD:
            cmd_thrust_input = 0.65 if key_inputs["Z"] else (-1 if key_inputs["S"] else 0)
            cmd_roll_input = 1 if key_inputs["RIGHT"] else (-1 if key_inputs["LEFT"] else 0)
            cmd_pitch_input = 1 if key_inputs["UP"] else (-1 if key_inputs["DOWN"] else 0)
        if CONTROLLER:
            cmd_thrust_input = joystick_a1
            cmd_roll_input = joystick_a2
            cmd_pitch_input = -joystick_a3

        min_thrust = 15_000 - max(0.01, -cmd_thrust_input) * 15_000
        max_thrust = 50_000
        thrust = max(0., cmd_thrust_input) * (max_thrust - min_thrust) + min_thrust
        roll = 15 * cmd_roll_input
        pitch = 15 * cmd_pitch_input

        pitch_cmd, roll_cmd, yaw_cmd, thrust_cmd = pitch, roll, 0, thrust
        cf.commander.send_setpoint(int(roll), int(pitch), 0, int(thrust))
        time.sleep(0.01)

def send_auto_drone_commands(cf):
    """Envoie les commandes au drone (copié exactement de height-hold-joystick.py)."""
    global pitch_cmd, roll_cmd, yaw_cmd, thrust_cmd, running

    def update_cmd():
        global thrust_cmd, roll_cmd
        thrust_cmd = 20_000
        roll_cmd =  30 * sin(2 * 3.14 * 0.25 * time.time())

    cf.commander.send_setpoint(0, 0, 0, 0)
    time.sleep(0.1)

    while running:
        try:
            update_cmd()
            cf.commander.send_setpoint(int(roll_cmd), int(pitch_cmd), int(yaw_cmd), int(thrust_cmd))
        except:
            print(f"ERROR : command invalid ({roll_cmd}, {pitch_cmd}, {yaw_cmd}, {thrust_cmd}).")

        time.sleep(0.01)

def do_nothing(cf):
    while running:
        cf.commander.send_setpoint(0, 0, 0, 0)
        time.sleep(0.01)

def my_pilote(cf):
    global roll, pitch
    cf.commander.send_setpoint(0, 0, 0, 0)
    thrust = 0
    while running:

        print(roll, pitch)
        if abs(roll) > 20 or abs(pitch) > 20:
            thrust = 0
        else:
            thrust = 30000

        cf.commander.send_setpoint(0, 0, 0, thrust)
        time.sleep(0.01)

# --- Keyboard input --- #
def getkeyInputs():
    global key_inputs
    keys = pygame.key.get_pressed()
    key_inputs["SPACE"] = True if keys[pygame.K_SPACE] else False
    key_inputs["UP"] = True if keys[pygame.K_UP] else False
    key_inputs["DOWN"] = True if keys[pygame.K_DOWN] else False
    key_inputs["RIGHT"] = True if keys[pygame.K_RIGHT] else False
    key_inputs["LEFT"] = True if keys[pygame.K_LEFT] else False
    key_inputs["Z"] = True if keys[pygame.K_z] else False
    key_inputs["S"] = True if keys[pygame.K_s] else False

# --- Fonction pour dessiner l'interface ---
def draw_interface():
    global battery_voltage, pitch, roll, yaw, position_z, screen, joystick_a1, joystick_a2, joystick_a3
    global pitch_cmd, roll_cmd, yaw_cmd, thrust_cmd, key_inputs

    # --- Variables de dimensionnement (modifiables) ---
    SECTION_WIDTH = 380
    SECTION_HEIGHT = 290  # Réduite pour laisser de la place
    CONTROL_SECTION_HEIGHT = 120  # Hauteur pour les commandes
    MARGIN = 10
    LINE_SPACING = 30
    LEFT_COLUMN = MARGIN
    RIGHT_COLUMN = MARGIN + SECTION_WIDTH + MARGIN
    BOTTOM_SECTION_TOP = 350  # Position verticale des commandes

    screen.fill((30, 30, 30))  # Fond gris foncé

    # --- Titre principal ---
    title = font.render("DRONE MONITOR", True, (100, 200, 255))
    screen.blit(title, (280, 10))

    # --- Section État du Drone (gauche) ---
    pygame.draw.rect(screen, (40, 40, 40), (LEFT_COLUMN, 40, SECTION_WIDTH, SECTION_HEIGHT))
    drone_title = font.render("ÉTAT DU DRONE", True, (200, 200, 200))
    screen.blit(drone_title, (LEFT_COLUMN + 10, 50))

    # Batterie avec couleur selon le voltage
    battery_color = (0, 255, 0)  # Vert par défaut
    if battery_voltage < 3.5:
        battery_color = (255, 0, 0)  # Rouge
    elif battery_voltage < 3.6:
        battery_color = (255, 165, 0)  # Orange

    battery_text = font.render(f"Batterie: {battery_voltage:.2f} V", True, battery_color)
    screen.blit(battery_text, (LEFT_COLUMN + 10, 80))

    # Orientation
    orientation_title = font.render("Orientation (°):", True, (200, 200, 200))
    screen.blit(orientation_title, (LEFT_COLUMN + 10, 110))
    pitch_text = font.render(f"  Pitch: {pitch:.1f}°", True, (255, 255, 255))
    screen.blit(pitch_text, (LEFT_COLUMN + 30, 140))
    roll_text = font.render(f"  Roll: {roll:.1f}°", True, (255, 255, 255))
    screen.blit(roll_text, (LEFT_COLUMN + 30, 170))
    yaw_text = font.render(f"  Yaw: {yaw:.1f}°", True, (255, 255, 255))
    screen.blit(yaw_text, (LEFT_COLUMN + 30, 200))

    # Altitude
    altitude_text = font.render(f"Altitude (Z): {position_z:.2f} m", True, (255, 255, 255))
    screen.blit(altitude_text, (LEFT_COLUMN + 10, 230))

    # --- Section Contrôles (droite) ---
    pygame.draw.rect(screen, (40, 40, 40), (RIGHT_COLUMN, 40, SECTION_WIDTH, SECTION_HEIGHT))
    controller_title = font.render("CONTRÔLES", True, (200, 200, 200))
    screen.blit(controller_title, (RIGHT_COLUMN + 10, 50))

    # Sous-titre Joystick
    joystick_title = font.render(f"Joystick: {'ON' if CONTROLLER else 'OFF'}", True, (200, 200, 200))
    screen.blit(joystick_title, (RIGHT_COLUMN + 20, 80))
    thrust_text = font.render(f"  Thrust (Right Y): {joystick_a1:.2f}", True, (255, 255, 255))
    screen.blit(thrust_text, (RIGHT_COLUMN + 40, 110))
    roll_text = font.render(f"  Roll (Left X): {joystick_a2:.2f}", True, (255, 255, 255))
    screen.blit(roll_text, (RIGHT_COLUMN + 40, 140))
    pitch_text = font.render(f"  Pitch (Left Y): {joystick_a3:.2f}", True, (255, 255, 255))
    screen.blit(pitch_text, (RIGHT_COLUMN + 40, 170))

    # Sous-titre Clavier (positionné plus bas)
    keyboard_title = font.render("Clavier:", True, (200, 200, 200))
    screen.blit(keyboard_title, (RIGHT_COLUMN + 20, 200))

    # Affichage compact des touches
    up_text = font.render(f"↑ : {key_inputs['UP']:.1f}", True, (255, 255, 255))
    screen.blit(up_text, (RIGHT_COLUMN + 40, 230))
    down_text = font.render(f"↓ : {key_inputs['DOWN']:.1f}", True, (255, 255, 255))
    screen.blit(down_text, (RIGHT_COLUMN + 120, 230))
    left_text = font.render(f"← : {key_inputs['LEFT']:.1f}", True, (255, 255, 255))
    screen.blit(left_text, (RIGHT_COLUMN + 200, 230))
    right_text = font.render(f"→ : {key_inputs['RIGHT']:.1f}", True, (255, 255, 255))
    screen.blit(right_text, (RIGHT_COLUMN + 280, 230))

    space_text = font.render(f"SPACE: {key_inputs['SPACE']:.1f}", True, (255, 255, 255))
    screen.blit(space_text, (RIGHT_COLUMN + 40, 260))
    z_text = font.render(f"Z: {key_inputs['Z']:.1f}", True, (255, 255, 255))
    screen.blit(z_text, (RIGHT_COLUMN + 160, 260))
    s_text = font.render(f"S: {key_inputs['S']:.1f}", True, (255, 255, 255))
    screen.blit(s_text, (RIGHT_COLUMN + 220, 260))

    # --- Section Commandes Envoyées (bas) ---
    pygame.draw.rect(screen, (40, 40, 40), (MARGIN, BOTTOM_SECTION_TOP, 780, CONTROL_SECTION_HEIGHT))
    commands_title = font.render("COMMANDES ENVOYÉES AU DRONE", True, (200, 200, 200))
    screen.blit(commands_title, (MARGIN + 10, BOTTOM_SECTION_TOP + 10))

    cmd_pitch_text = font.render(f"Pitch: {pitch_cmd:.1f}°", True, (255, 255, 0))
    screen.blit(cmd_pitch_text, (MARGIN + 50, BOTTOM_SECTION_TOP + 40))
    cmd_roll_text = font.render(f"Roll: {roll_cmd:.1f}°", True, (255, 255, 0))
    screen.blit(cmd_roll_text, (MARGIN + 200, BOTTOM_SECTION_TOP + 40))
    cmd_yaw_text = font.render(f"Yaw: {yaw_cmd:.1f}°", True, (255, 255, 0))
    screen.blit(cmd_yaw_text, (MARGIN + 350, BOTTOM_SECTION_TOP + 40))
    cmd_thrust_text = font.render(f"Thrust: {thrust_cmd:.0f}", True, (255, 255, 0))
    screen.blit(cmd_thrust_text, (MARGIN + 500, BOTTOM_SECTION_TOP + 40))

    # --- Instructions ---
    instructions = font.render("Appuyez sur Q pour quitter", True, (255, 100, 100))
    screen.blit(instructions, (MARGIN, 550))

    # --- Séparateurs visuels ---
    pygame.draw.line(screen, (100, 100, 100), (MARGIN, BOTTOM_SECTION_TOP - MARGIN), (790, BOTTOM_SECTION_TOP - MARGIN), 2)  # Ligne horizontale
    pygame.draw.line(screen, (100, 100, 100), (390 + 0.4 * MARGIN, 40), (390 + 0.4 * MARGIN, BOTTOM_SECTION_TOP - MARGIN), 2)  # Ligne verticale


# --- Fonction principale ---
def main():
    global running
    print("Drone Monitor")

    # Initialisation du drone
    cflib.crtp.init_drivers()

    with SyncCrazyflie(DRONE_URI) as scf:
        cf = scf.cf
        setup_drone_logging(cf)

        # Démarrage des threads
        if CONTROLLER:
            controller_thread = threading.Thread(target=read_controller)
            controller_thread.daemon = True
            controller_thread.start()

        if ACTIVATE_CONTROL:
            if MY_PILOT:
                command_thread = threading.Thread(target=my_pilote, args=(cf,))
                command_thread.daemon = True
                command_thread.start()
            elif MANUAL_CONTROL:
                command_thread = threading.Thread(target=send_manual_drone_commands, args=(cf,))
                command_thread.daemon = True
                command_thread.start()
            else:
                command_thread = threading.Thread(target=send_auto_drone_commands, args=(cf,))
                command_thread.daemon = True
                command_thread.start()
        else:
            command_thread = threading.Thread(target=do_nothing, args=(cf,))
            command_thread.daemon = True
            command_thread.start()

        # Boucle principale
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_q):
                    running = False

            getkeyInputs()
            draw_interface()
            pygame.display.flip()
            clock.tick(60)

    cf.commander.send_setpoint(0, 0, 0, 0)
    pygame.quit()
    sys.exit()

main()
