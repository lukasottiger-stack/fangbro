import pygame
import math
import os
import random
import tkinter as tk
from tkinter import ttk
import socket
import threading
import time

# ================================================
# PYGAME SETUP
# ================================================
pygame.init()
window_w, window_h = 1440, 1000
display = pygame.display.set_mode((window_w, window_h))
pygame.display.set_caption("Flight Simulator - Realistic MiG-21 - 1ms timestep")
clock = pygame.time.Clock()

# Load original image
script_dir = os.path.dirname(os.path.abspath(__file__))
base_plane_scale = 0.7
mig21_original = pygame.image.load(os.path.join(script_dir, "slaves", "mig21.png"))




# Zoom system
zoom_level = 1.0
base_pixels_per_meter_z = 100.0
base_horizontal_pixels_per_meter = 100.0   # balanced with vertical scale — fixes slow horizontal drift
plane_scale = base_plane_scale * zoom_level
zoom_min = 0.02
zoom_max = 2.2
zoom_factor = 1.2


wing_angle_lift_coefficient = 3.5   # real MiG-21 delta subsonic ~3.2-3.8 per rad
elevator_angle_lift_coefficient = 4.0   # real stabilator effective
reference_q_for_max_deflection = 35000.0   # tuned for realistic high-speed limit
inverted_flight = 0.0  # 0=no, 1=yes (tuner compatible)

# New stability derivatives (physical, slider tunable)

# Initial dots

# =========================
# MULTIPLAYER TCP BACKGROUND SYNC (chat-style, no UI)
# =========================
is_multiplayer = False
connected = False
remote_ip = None
server_socket = None
client_socket = None
conn = None

opponent_position_x = 0.0
opponent_position_z = 500.0
opponent_pitch_angle_rad = 0.03
opponent_inverted = 0.0


def start_server():
    global server_socket
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind(('', 5000))
    server_socket.listen(1)
    threading.Thread(target=accept_connection, daemon=True).start()

def accept_connection():
    global conn, connected
    try:
        new_conn, addr = server_socket.accept()
        conn = new_conn
        connected = True
        threading.Thread(target=receive_data, args=(conn,), daemon=True).start()
    except:
        pass

def connect_to_host(ip):
    global client_socket, conn, connected
    try:
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect((ip, 5000))
        conn = client_socket
        connected = True
        threading.Thread(target=receive_data, args=(conn,), daemon=True).start()
    except:
        connected = False

def receive_data(connection):
    global connected, opponent_position_x, opponent_position_z, opponent_pitch_angle_rad, opponent_inverted
    buffer = ""
    while True:
        try:
            data = connection.recv(1024).decode('utf-8')
            if not data:
                connected = False
                break
            buffer += data
            while '\n' in buffer:
                line, buffer = buffer.split('\n', 1)
                parts = line.split(',')
                if len(parts) == 4:
                    opponent_position_x = float(parts[0])
                    opponent_position_z = float(parts[1])
                    opponent_pitch_angle_rad = float(parts[2])
                    opponent_inverted = float(parts[3])
        except:
            connected = False
            break

def send_state():
    global connected
    if not connected or conn is None:
        return
    try:
        data = f"{plane_position_x:.1f},{plane_position_z:.1f},{pitch_angle_rad:.4f},{int(inverted_flight)}\n"
        conn.send(data.encode('utf-8'))
    except:

        connected = False

# ================================================
# MiG-21SMT PARAMETERS (accurate as possible based on real data)
# ================================================
pi = 3.14159
gravity = 9.80665
oswald_e = 0.80
wing_surface_area = 23.0  # same delta wing
wingspan = 7.154  # precise
specific_heat_ratio = 1.4
wing_angle_lift_coefficient = 4.2  # subsonic delta ~4.0-4.5 /rad
elevator_surface_area = 3.94
distance_from_cg_to_elevator = 7.5  # approximate, same tail
temperature_lapse_rate = 0.0065
sea_level_temperature = 288.15
gas_constant = 287.05
pressure_at_sea_level = 101325
timestep = 0.001
Cd0 = 0.030  # slightly higher due to dorsal hump drag (real ~0.018-0.032 clean)
zero_aoa_inlet_efficiency = 0.88
aoa_sensitivity_constant = 5.7
max_velocity_exit = 1220.0  # tuned lower for R-13-300 ~64.7 kN AB (vs bis 1320 m/s for ~69 kN)
max_inlet_area = 0.35  # fixed cone, approximate
plane_mass = 9500  # normal combat weight (heavier than bis ~8800 kg due to fuel spine)
aoa_stall_deg = 18  # slight increase possible with vortex lift
max_wing_lift_coefficient = 1.9
aoa_stall_rad = aoa_stall_deg * pi / 180
elevator_angle_lift_coefficient = 2 * pi
moment_of_inertia = 190000  # estimated +~13% vs bis due to spine mass distribution (no exact public data)
tail_efficiency = 0.7  # same tail
thrust_line_offset = 2.2  # approximate, engine low

# Elevator actuator
target_elevator_angle = 0.0
current_elevator_angle = 0.0
elevator_rate_deg_per_sec = 60.0
max_elevator_deflection_deg = 25.0
reference_q_for_max_deflection = 20000.0

# State (initial throttle higher for heavier SMT)
total_thrust = 0.0
plane_velocity = 0.0
ambient_density = 0.0
speed_of_sound = 0.0
plane_velocity_x = 250.0
plane_velocity_z = 0.0
plane_position_x = 0.0
plane_position_z = 500.0
nose_direction = 0.03
throttle_level = 40.0  # SMT needs more power at same speed
pitch_rate_rad = 0.0
pitch_angle_rad = 0.03
lift = 0.0
total_drag = 0.0
forward_acceleration = 0.0
vertical_acceleration = 0.0
flight_path_angle = 0.0
main_wing_aoa = 0.0
stall_aware_lift_coefficient = 0.0
main_wing_lift = 0.0
tail_aoa = 0.0
stall_aware_elevator_lift_coefficient = 0.0
lift_elevator = 0.0
pitch_torque = 0.0
inlet_efficiency = 0.0
mass_flow_rate = 0.0
velocity_exit = 0.0
velocity_inlet = 0.0
normal_drag = 0.0
induced_drag = 0.0
temperature_at_height = 0.0
pressure_at_height = 0.0
wake_reduction = 1.0
Cm_alpha = -0.8   # similar stability
Cm_q = -25.0      # similar damping
mean_aerodynamic_chord = wing_surface_area / wingspan

second_position_x = 0.0
second_position_z = 500.0
second_pitch_angle_rad = 0.03

# ================================================
# SAFE REAL NUMBER FUNCTIONSF
# ================================================
def safe_cos(x):
    return math.cos(float(x))

def safe_exp(x):
    return math.exp(float(x))

#reciever for LAN data
def receive_loop():
    global connected, remote_ip, opponent_position_x, opponent_position_z, opponent_pitch_angle_rad, opponent_inverted
    while True:
        try:
            data, addr = receive_socket.recvfrom(1024)
            if not connected:
                connected = True
                remote_ip = addr[0]
            parts = data.decode('utf-8').split(',')
            if len(parts) == 4:
                opponent_position_x = float(parts[0])
                opponent_position_z = float(parts[1])
                opponent_pitch_angle_rad = float(parts[2])
                opponent_inverted = float(parts[3])
        except BlockingIOError:
            pass
        except Exception:
            pass
        time.sleep(0.001)

def send_data():
    if not is_multiplayer or remote_ip is None or send_socket is None:
        return
    data = f"{plane_position_x},{plane_position_z},{pitch_angle_rad},{inverted_flight}"
    send_socket.sendto(data.encode('utf-8'), (remote_ip, local_port))

# ================================================
# TKINTER TUNER WINDOW (fixed closure bug with factory function)
# ================================================
tuner_root = None

def create_tuner_window():
    global tuner_root
    tuner_root = tk.Tk()
    tuner_root.title("MiG-21 Parameter Tuner - Real-Time Adjustment")
    tuner_root.geometry("600x900")
    tuner_root.resizable(False, False)

    tk.Label(tuner_root, text="TUNER READY - Sliders below", fg="green", font=("Consolas", 12, "bold")).grid(row=0, column=0, columnspan=3, pady=10)

    params = {
        "inverted_flight (0=no 1=yes)": (0.0, 1.0, 1.0, 0.0),
        "Cd0 (drag coeff)": (0.01, 0.05, 0.001, Cd0),
        "max_velocity_exit (m/s)": (1000.0, 1800.0, 10.0, max_velocity_exit),
        "thrust_line_offset (m below CG)": (0.0, 1.0, 0.05, thrust_line_offset),
        "tail_efficiency": (0.3, 1.0, 0.05, tail_efficiency),
        "wing_angle_lift_coefficient (/rad)": (3.0, 6.0, 0.1, wing_angle_lift_coefficient),
        "moment_of_inertia (kg·m²)": (100000, 400000, 10000, moment_of_inertia),
        "elevator_rate_deg_per_sec": (30.0, 100.0, 5.0, elevator_rate_deg_per_sec),
        "max_elevator_deflection_deg": (15.0, 35.0, 1.0, max_elevator_deflection_deg),
        "reference_q_for_max_deflection (Pa)": (10000.0, 40000.0, 1000.0, reference_q_for_max_deflection),
        "throttle_level (%)": (0.0, 100.0, 5.0, throttle_level),
        "initial_nose_direction (rad)": (-0.1, 0.1, 0.005, nose_direction),
    }

    row = 1
    for name, (min_val, max_val, res, initial) in params.items():
        tk.Label(tuner_root, text=name, font=("Consolas", 10)).grid(row=row, column=0, sticky="w", padx=10, pady=5)
        
        var = tk.DoubleVar(value=initial)
        slider = tk.Scale(tuner_root, from_=min_val, to=max_val, resolution=res, orient="horizontal", variable=var, length=300)
        slider.grid(row=row, column=1, padx=10, pady=5)
        
        label = tk.Label(tuner_root, text=f"{initial:.4f}", font=("Consolas", 10))
        label.grid(row=row, column=2, padx=10, pady=5)
        
        global_map = {
            "inverted_flight (0=no 1=yes)": "inverted_flight",
            "Cd0 (drag coeff)": "Cd0",
            "max_velocity_exit (m/s)": "max_velocity_exit",
            "thrust_line_offset (m below CG)": "thrust_line_offset",
            "tail_efficiency": "tail_efficiency",
            "wing_angle_lift_coefficient (/rad)": "wing_angle_lift_coefficient",
            "moment_of_inertia (kg·m²)": "moment_of_inertia",
            "elevator_rate_deg_per_sec": "elevator_rate_deg_per_sec",
            "max_elevator_deflection_deg": "max_elevator_deflection_deg",
            "reference_q_for_max_deflection (Pa)": "reference_q_for_max_deflection",
            "throttle_level (%)": "throttle_level",
            "initial_nose_direction (rad)": "nose_direction",
        }
        
        def make_update(var=var, label=label, gname=global_map[name]):
            def update(*args):
                value = var.get()
                globals()[gname] = value
                label.config(text=f"{value:.4f}")
            return update
        
        var.trace("w", make_update())

        row += 1

    tk.Label(tuner_root, text="Changes apply instantly (restart for initial nose_direction). Close window = tuner off.", font=("Consolas", 10, "italic")).grid(row=row, column=0, columnspan=3, pady=20)

    tuner_root.update_idletasks()
    tuner_root.update()




# ================================================
# ATMOSPHERE
# ================================================
def calculate_temperature_at_height():
    global temperature_at_height
    temperature_at_height = sea_level_temperature - temperature_lapse_rate * plane_position_z
    if temperature_at_height < 1.0:
        temperature_at_height = 1.0

def calculate_pressure_at_height():
    global pressure_at_height
    pressure_at_height = pressure_at_sea_level * (1 - temperature_lapse_rate * plane_position_z / sea_level_temperature) ** (gravity / (temperature_lapse_rate * gas_constant))

def calculate_ambient_density():
    global ambient_density
    ambient_density = pressure_at_height / (gas_constant * temperature_at_height)

def calculate_speed_of_sound():
    global speed_of_sound
    speed_of_sound = math.sqrt(specific_heat_ratio * gas_constant * temperature_at_height)

# ================================================
# FLIGHT DYNAMICS
# ================================================
def calculate_airspeed_magnitude():
    global plane_velocity
    plane_velocity = math.sqrt(plane_velocity_x ** 2 + plane_velocity_z ** 2)
    if plane_velocity < 10.0:
        plane_velocity = 10.0

def calculate_flight_path_angle():
    global flight_path_angle
    flight_path_angle = math.atan2(plane_velocity_z, plane_velocity_x)

def calculate_main_wing_aoa():
    global main_wing_aoa
    delta = nose_direction - flight_path_angle
    # Shortest angle (continuous, no 360° jump on wrap)
    main_wing_aoa = delta - 2 * pi * round(delta / (2 * pi))

def calculate_stall_aware_wing_lift_coefficient():
    global stall_aware_lift_coefficient
    if abs(main_wing_aoa) <= aoa_stall_rad:
        stall_aware_lift_coefficient = wing_angle_lift_coefficient * main_wing_aoa
    else:
        # Smooth parabolic drop after stall (realistic for delta vortex lift peak then drop)
        excess = abs(main_wing_aoa) - aoa_stall_rad
        stall_drop_factor = 1.0 - (excess / (pi/2 - aoa_stall_rad))**2   # tuned to drop smoothly to 0 at 90°
        stall_drop_factor = max(stall_drop_factor, 0.0)
        stall_aware_lift_coefficient = max_wing_lift_coefficient * math.copysign(stall_drop_factor, main_wing_aoa)

def get_local_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except:
        return "127.0.0.1"

local_ip = get_local_ip()   # DEFINED EARLY — available for menu

def calculate_main_wing_lift():
    global main_wing_lift
    q = 0.5 * ambient_density * plane_velocity ** 2
    main_wing_lift = q * wing_surface_area * stall_aware_lift_coefficient   # force magnitude

def calculate_elevator_aoa():
    global tail_aoa, wake_reduction
    wake_reduction = 1.0
    tail_aoa = main_wing_aoa * tail_efficiency - (current_elevator_angle * pi / 180)

    if abs(main_wing_aoa) > aoa_stall_rad + 0.087:  # ~5° buffer
        excess = abs(main_wing_aoa) - (aoa_stall_rad + 0.087)
        wake_reduction = max(0.2, 1.0 - excess / 0.3)  # drops faster to 0.2 (strong tail loss in deep stall)
        tail_aoa *= wake_reduction

    if plane_velocity > 10.0:
        damping_term = pitch_rate_rad * distance_from_cg_to_elevator / plane_velocity
        tail_aoa += damping_term * tail_efficiency * wake_reduction

def calculate_stall_aware_elevator_lift_coefficient():
    global stall_aware_elevator_lift_coefficient
    if abs(tail_aoa) <= aoa_stall_rad:
        stall_aware_elevator_lift_coefficient = elevator_angle_lift_coefficient * tail_aoa
    else:
        excess = abs(tail_aoa) - aoa_stall_rad
        stall_drop_factor = 1.0 - (excess / (pi/2 - aoa_stall_rad))**2
        stall_drop_factor = max(stall_drop_factor, 0.0)
        stall_aware_elevator_lift_coefficient = elevator_angle_lift_coefficient * math.copysign(stall_drop_factor, tail_aoa)

def calculate_elevator_lift():
    global lift_elevator
    q = 0.5 * ambient_density * plane_velocity ** 2
    lift_elevator = q * elevator_surface_area * stall_aware_elevator_lift_coefficient

def calculate_pitch_torque_from_elevator():
    global pitch_torque
    q = 0.5 * ambient_density * plane_velocity ** 2

    # Elevator + thrust line
    pitch_torque = -lift_elevator * distance_from_cg_to_elevator + total_thrust * thrust_line_offset

    # Static stability Cm_alpha (nose-down with AoA — real wing/fuselage contribution)
    stability_moment = q * wing_surface_area * mean_aerodynamic_chord * Cm_alpha * main_wing_aoa
    pitch_torque += stability_moment

    # Pitch damping Cm_q (strong opposing moment from rate — real main source)
    if plane_velocity > 10.0:
        damping_moment = q * wing_surface_area * mean_aerodynamic_chord**2 * Cm_q * (pitch_rate_rad / (2 * plane_velocity))
        pitch_torque += damping_moment

def calculate_drag():
    global total_drag, normal_drag, induced_drag
    AR = wingspan ** 2 / wing_surface_area
    Cl = stall_aware_lift_coefficient
    induced_drag_coeff = (Cl ** 2) / (math.pi * AR * oswald_e)
    q = 0.5 * ambient_density * plane_velocity ** 2
    normal_drag = q * wing_surface_area * Cd0
    induced_drag = q * wing_surface_area * induced_drag_coeff
    total_drag = normal_drag + induced_drag

def calculate_thrust():
    global total_thrust, mass_flow_rate, velocity_exit, velocity_inlet
    velocity_exit = throttle_level * max_velocity_exit / 100
    velocity_inlet = plane_velocity
    inlet_efficiency = zero_aoa_inlet_efficiency * safe_cos(main_wing_aoa) * safe_exp(-aoa_sensitivity_constant * abs(main_wing_aoa))
    mass_flow_rate = ambient_density * max_inlet_area * velocity_inlet * inlet_efficiency
    total_thrust = mass_flow_rate * (velocity_exit - velocity_inlet)

def calculate_accelerations_vector():
    global forward_acceleration, vertical_acceleration
    if plane_velocity < 10.0:
        forward_acceleration = 0.0
        vertical_acceleration = -gravity
        return

    # Unit vector in velocity direction
    Vhat_x = plane_velocity_x / plane_velocity
    Vhat_z = plane_velocity_z / plane_velocity

    # Lift perpendicular to velocity (positive lift upward/right depending on direction)
    # Wing lift component
    L_wing_x = -main_wing_lift * Vhat_z   # perpendicular component
    L_wing_z = main_wing_lift * Vhat_x

    # Tail lift component (same velocity direction)
    L_tail_x = -lift_elevator * Vhat_z
    L_tail_z = lift_elevator * Vhat_x

    # Total forces
    F_x = total_thrust + L_wing_x + L_tail_x - total_drag * Vhat_x   # drag opposite velocity
    F_z = L_wing_z + L_tail_z - plane_mass * gravity

    forward_acceleration = F_x / plane_mass
    vertical_acceleration = F_z / plane_mass

def calculate_forward_acceleration():
    global forward_acceleration
    net_force = total_thrust - total_drag - (lift * math.sin(main_wing_aoa))
    forward_acceleration = net_force / plane_mass

def calculate_vertical_acceleration():
    global vertical_acceleration
    net_vertical_force = lift * math.cos(main_wing_aoa) - (plane_mass * gravity)
    vertical_acceleration = net_vertical_force / plane_mass

def update_pitch_rate():
    global pitch_rate_rad
    pitch_rate_rad += (pitch_torque / moment_of_inertia) * timestep
    pitch_rate_rad = max(min(pitch_rate_rad, 2.0), -2.0)   # temporary safety ~115 deg/s max

def update_pitch_angle():
    global pitch_angle_rad
    pitch_angle_rad += pitch_rate_rad * timestep
    # Wrap pitch to -pi to pi for correct AoA and display after loops/inverted flight
    pitch_angle_rad = ((pitch_angle_rad + pi) % (2 * pi)) - pi

def update_plane_velocity():
    global plane_velocity_x, plane_velocity_z
    plane_velocity_x += forward_acceleration * timestep
    plane_velocity_z += vertical_acceleration * timestep

def update_plane_position():
    global plane_position_x, plane_position_z
    plane_position_x += plane_velocity_x * timestep
    if plane_position_x < 0:
        plane_position_x = 0.0
    plane_position_z += plane_velocity_z * timestep

def handle_input():
    global target_elevator_angle, throttle_level, inverted_flight
    keys = pygame.key.get_pressed()
    desired_elev = 0.0
    if keys[pygame.K_w]:
        desired_elev = -10.0
    elif keys[pygame.K_s]:
        desired_elev = 20.0
    target_elevator_angle = desired_elev if inverted_flight <= 0.5 else -desired_elev
    
    if keys[pygame.K_UP]:
        throttle_level = min(100.0, throttle_level + 0.5)
    if keys[pygame.K_DOWN]:
        throttle_level = max(0.0, throttle_level - 0.5)  # FIXED: was +0.5 bug
    

def update_elevator_angle():
    global current_elevator_angle
    q = 0.5 * ambient_density * plane_velocity ** 2
    current_max_deflection = max_elevator_deflection_deg * (reference_q_for_max_deflection / max(q, 1000.0))   # avoid divide zero
    current_max_deflection = min(current_max_deflection, max_elevator_deflection_deg)
    
    clamped_target = max(min(target_elevator_angle, current_max_deflection), -current_max_deflection)
    
    max_change = elevator_rate_deg_per_sec * timestep
    diff = clamped_target - current_elevator_angle
    if abs(diff) > max_change:
        current_elevator_angle += math.copysign(max_change, diff)
    else:
        current_elevator_angle = clamped_target

# ================================================
# STARTUP MENU
# ================================================
menu_root = tk.Tk()
menu_root.title("MiG-21 Flight Simulator")
menu_root.geometry("450x350")
menu_root.resizable(False, False)

tk.Label(menu_root, text="MiG-21 Realistic Flight Simulator", font=("Consolas", 16, "bold")).pack(pady=20)
tk.Label(menu_root, text="Choose game mode", font=("Consolas", 12)).pack(pady=10)

def start_singleplayer():
    global is_multiplayer
    is_multiplayer = False
    menu_root.destroy()

tk.Button(menu_root, text="Singleplayer", width=25, height=2, command=start_singleplayer).pack(pady=10)

def open_multiplayer_setup():
    menu_root.withdraw()
    multi_root = tk.Toplevel()
    multi_root.title("Multiplayer Setup")
    multi_root.geometry("400x300")

    tk.Label(multi_root, text="Multiplayer LAN", font=("Consolas", 14)).pack(pady=20)

    def host_game():
        global is_multiplayer
        is_multiplayer = True
        start_server()  # start TCP server
        multi_root.destroy()
        menu_root.destroy()

    def join_game():
        global is_multiplayer, remote_ip
        ip = ip_entry.get().strip()
        if ip:
            remote_ip = ip
            is_multiplayer = True
            connect_to_host(ip)  # connect via TCP
            multi_root.destroy()
            menu_root.destroy()
        else:
            tk.Label(multi_root, text="Enter valid IP!", fg="red").pack()

    tk.Button(multi_root, text="Host Game", width=20, height=2, command=host_game).pack(pady=10)
    tk.Label(multi_root, text=f"Your LAN IP: {local_ip}", font=("Consolas", 10)).pack()
    tk.Label(multi_root, text="Give this IP to your friend").pack(pady=5)

    tk.Label(multi_root, text="Or join a game:", font=("Consolas", 12)).pack(pady=20)
    ip_entry = tk.Entry(multi_root, width=20)
    ip_entry.pack()
    ip_entry.insert(0, "192.168.")
    tk.Button(multi_root, text="Join Game", width=20, height=2, command=join_game).pack(pady=10)

tk.Button(menu_root, text="Multiplayer (LAN)", width=25, height=2, command=open_multiplayer_setup).pack(pady=10)

menu_root.mainloop()

create_tuner_window()  # tuner after menu

# After menu closes — start networking if multiplaye

# Now continue with pygame.init() etc (your existing cod

def simulate_one_step():
    global lift, nose_direction, plane_position_z, plane_velocity_z, plane_velocity_x, tuner_root
    global plane_position_z, plane_velocity_z, plane_velocity_x, nose_direction, pitch_angle_rad, pitch_rate_rad, plane_position_x

    calculate_temperature_at_height()
    calculate_pressure_at_height()
    calculate_ambient_density()
    calculate_speed_of_sound()

    calculate_airspeed_magnitude()
    calculate_flight_path_angle()
    calculate_main_wing_aoa()

    calculate_thrust()
    update_elevator_angle()
    calculate_stall_aware_wing_lift_coefficient()
    calculate_main_wing_lift()
    calculate_elevator_aoa()
    calculate_stall_aware_elevator_lift_coefficient()
    calculate_elevator_lift()
    lift = main_wing_lift + lift_elevator

    calculate_drag()
    calculate_pitch_torque_from_elevator()
    update_pitch_rate()
    update_pitch_angle()
    nose_direction = pitch_angle_rad

    calculate_accelerations_vector()
    update_plane_velocity()
    update_plane_position()

    if plane_position_z < 0:
        

        if plane_velocity_z < -15.0:  # high downward speed = crash (tuned for fatal impact)
            # CRASH — respawn to initial trimmed state
            plane_position_z = 500.0
            plane_velocity_x = 250.0
            plane_velocity_z = 0.0
            plane_position_x += 100.0  # slight forward offset to avoid repeat crash if looping
            nose_direction = 0.03
            pitch_angle_rad = 0.03
            pitch_rate_rad = 0.0
            current_elevator_angle = 0.0
            target_elevator_angle = 0.0
            throttle_level = 36.0
            total_thrust = 0.0  # will recalculate next step
            print("CRASH DETECTED — RESPAWNED AT 500 m")  # console feedback for testing
        else:
            # Soft landing — stop at ground
            plane_position_z = 0.0
            plane_velocity_z = max(plane_velocity_z, 0.0)  # no bounce up
            plane_velocity_x *= 0.98  # rolling friction

    # In main loop — replace your tuner update with this safe version
    try:
        if tuner_root:
            tuner_root.update()
    except tk.TclError:
        tuner_root = None
    

    
# ================================================
# RENDER - fixed horizon sign, visible sky/ground at start
# ================================================
font = pygame.font.SysFont("consolas", 18)

def render():
    global plane_scale, pixels_per_meter_z, horizon_y, zoom_level

    plane_scale = base_plane_scale * zoom_level
    pixels_per_meter_z = base_pixels_per_meter_z * zoom_level
    horizontal_pixels_per_meter = base_horizontal_pixels_per_meter * zoom_level

    display.fill((135, 206, 235))

    screen_x = window_w // 2
    screen_y = window_h // 2 + 50

    horizon_y = int(screen_y + plane_position_z * pixels_per_meter_z)
    pygame.draw.line(display, (100, 100, 100), (0, horizon_y), (window_w, horizon_y), 5)

    ground_top = max(horizon_y, 0)
    if ground_top < window_h:
        pygame.draw.rect(display, (34, 139, 34), (0, ground_top, window_w, window_h - ground_top))

    # Determine which data to use for the "second" plane
    if is_multiplayer and connected:
        draw_x = opponent_position_x
        draw_z = opponent_position_z
        draw_pitch = opponent_pitch_angle_rad
        draw_inverted = opponent_inverted
        opponent_display = f"Opponent: X: {opponent_position_x:.0f} m   Z: {opponent_position_z:.0f} m"
    else:
        draw_x = second_position_x
        draw_z = second_position_z
        draw_pitch = second_pitch_angle_rad
        draw_inverted = 0.0
        opponent_display = "No data"

    # Second/opponent plane
    rel_x = draw_x - plane_position_x
    rel_z = plane_position_z - draw_z
    screen_second_x = screen_x - rel_x * horizontal_pixels_per_meter
    screen_second_y = screen_y + rel_z * pixels_per_meter_z

    if -500 < screen_second_x < window_w + 500 and -500 < screen_second_y < window_h + 500:
        rotation_angle = -math.degrees(draw_pitch)
        rotated_second = pygame.transform.rotozoom(mig21_original, rotation_angle, plane_scale)
        if draw_inverted > 0.5:
            rotated_second = pygame.transform.rotozoom(mig21_original, -rotation_angle, plane_scale)
            rotated_second = pygame.transform.flip(rotated_second, False, True)
        rect_second = rotated_second.get_rect(center=(screen_second_x, screen_second_y))
        display.blit(rotated_second, rect_second)

    # Arrow to opponent (only in multiplayer when connected)
    if is_multiplayer and connected:
        dx = opponent_position_x - plane_position_x
        dz = opponent_position_z - plane_position_z
        screen_dx = dx * horizontal_pixels_per_meter
        screen_dz = dz * pixels_per_meter_z
        dist = math.hypot(screen_dx, screen_dz)
        if dist > 50:
            dir_x = screen_dx / dist
            dir_z = screen_dz / dist
            arrow_len = min(dist * 0.8, 400)
            end_x = screen_x + dir_x * arrow_len
            end_y = screen_y + dir_z * arrow_len
            pygame.draw.line(display, (255, 0, 0), (screen_x, screen_y), (end_x, end_y), 6)
            head = 25
            angle = math.atan2(dir_z, dir_x)
            p1 = (end_x - head * math.cos(angle - 0.5), end_y - head * math.sin(angle - 0.5))
            p2 = (end_x - head * math.cos(angle + 0.5), end_y - head * math.sin(angle + 0.5))
            pygame.draw.line(display, (255, 0, 0), (end_x, end_y), p1, 6)
            pygame.draw.line(display, (255, 0, 0), (end_x, end_y), p2, 6)

    # Player plane
    rotation_angle = -math.degrees(pitch_angle_rad)
    rotated = pygame.transform.rotozoom(mig21_original, rotation_angle, plane_scale)
    if inverted_flight > 0.5:
        rotated = pygame.transform.rotozoom(mig21_original, -rotation_angle, plane_scale)
        rotated = pygame.transform.flip(rotated, False, True)
    rect = rotated.get_rect(center=(screen_x, screen_y))
    display.blit(rotated, rect)

    # Left telemetry (unchanged)
    txt = [
        f"Alt: {plane_position_z:6.0f} m",
        f"TAS: {plane_velocity:5.1f} m/s ({plane_velocity * 3.6:5.1f} km/h)",
        f"AoA: {math.degrees(main_wing_aoa):5.1f}°",
        f"Pitch: {math.degrees(pitch_angle_rad):5.1f}°",
        f"Flight Path γ: {math.degrees(flight_path_angle):5.1f}°",
        f"Lift: {lift:7.0f} N",
        f"Elev: {current_elevator_angle:5.1f}° (target {target_elevator_angle:5.1f})",
        f"Thrust: {total_thrust:6.0f} N",
        f"Drag:   {total_drag:6.0f} N",
        f"Throttle: {throttle_level:3.0f}%",
        f"Zoom Level: {zoom_level:.2f}",
        f"Inverted: {'YES' if inverted_flight > 0.5 else 'NO'}",
    ]
    if is_multiplayer:
        status = "Connected" if connected else "Waiting for opponent..."
        txt.append(f"Multiplayer: {status}")
        txt.append(f"Opponent IP: {remote_ip or 'N/A'}")
    for i, line in enumerate(txt):
        surf = font.render(line, True, (255, 255, 255))
        display.blit(surf, (20, 20 + i * 25))

    # NEW: Top-right coordinates display
    coord_your = f"Your Plane: X: {plane_position_x:.0f} m   Z: {plane_position_z:.0f} m"
    coord_opponent = opponent_display

    surf_your = font.render(coord_your, True, (255, 255, 255))
    surf_opp = font.render(coord_opponent, True, (255, 255, 255) if is_multiplayer and connected else (180, 180, 180))

    right_x = window_w - 380  # adjust if needed for your resolution
    display.blit(surf_your, (right_x, 20))
    display.blit(surf_opp, (right_x, 50))

    pygame.display.flip()

# ================================================
# INITIAL ATMOSPHERE
# ================================================
calculate_temperature_at_height()
calculate_pressure_at_height()
calculate_ambient_density()
calculate_speed_of_sound()

# ================================================
# MAIN LOOP
# ================================================
running = True
last_time = pygame.time.get_ticks()
accumulated_time = 0.0

while running:
    current_time = pygame.time.get_ticks()
    frame_dt = (current_time - last_time) / 1000.0
    last_time = current_time
    accumulated_time += frame_dt

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.MOUSEWHEEL:
            if event.y < 0:
                zoom_level /= zoom_factor
            elif event.y > 0:
                zoom_level *= zoom_factor
            zoom_level = max(zoom_min, min(zoom_level, zoom_max))
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_a or event.key == pygame.K_d:
                inverted_flight = 1.0 - inverted_flight

    handle_input()

    while accumulated_time >= timestep:
        simulate_one_step()
        accumulated_time -= timestep

    if is_multiplayer:
        send_state()  # TCP send every frame

    try:
        if tuner_root and tuner_root.winfo_exists():
            tuner_root.update()
    except:
        pass

    render()  # only once

    clock.tick(60)

pygame.quit()

# ===================================================================
# POTENTIAL FLAWS / INACCURACIES / LIKELY FUTURE BUG CAUSES
# ===================================================================
# 1. Temporary safety clamps on pitch rate/AoA — remove when stability confirmed perfect.
# 2. tail_efficiency = 0.8 estimated from typical tailed delta downwash — exact dε/dα for MiG-21 not public.
# 3. Lift slope 4.2 per rad estimated for subsonic delta — real has vortex lift non-linearity above ~15°.
# 4. No thrust line moment (engine low → nose-up from thrust).
# 5. No pressure thrust term.
# 6. Fixed inlet area — real cone variable with Mach.
# 7. No ground effect.
# 8. Horizon is flat line — real horizon curves at high alt, but fine for simulator.
# ===================================================================