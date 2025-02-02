import pygame
import sys
import numpy as np
import random
from math import pi
import matplotlib.pyplot as plt
from itertools import permutations
import time

def mypause(interval):
    manager = plt._pylab_helpers.Gcf.get_active()
    if manager is not None:
        canvas = manager.canvas
        if canvas.figure.stale:
            canvas.draw_idle()        
        #plt.show(block=False)
        canvas.start_event_loop(interval)
    else:
        time.sleep(interval)

# Data storage for plotting
time_data = []
pos_data = [[], [], []]
vel_data = [[], [], []]
acc_data = [[], [], []]

# Create Matplotlib figure and axes
fig, axs = plt.subplots(3, 1, figsize=(8, 12))
lines = {
    "position": [axs[0].plot([], [], label=f"Position {axis}")[0] for axis in "xyz"],
    "velocity": [axs[1].plot([], [], label=f"Velocity {axis}")[0] for axis in "xyz"],
    "acceleration": [axs[2].plot([], [], label=f"Acceleration {axis}")[0] for axis in "xyz"]
}
for ax, title in zip(axs, ["Position (m)", "Velocity (m/s)", "Acceleration (m/s²)"]):
    ax.set_xlim(0, 10)  # Initial time range
    ax.set_ylim(-300, 300)  # Y range
    ax.set_xlabel("Time(s)")
    ax.set_ylabel(title)
    ax.legend()
    ax.grid()

def norm(v):
    return v / np.linalg.norm(v)

def update_plot(frame):
    if not time_data:
        return
    for i in range(3):
        lines["position"][i].set_data(time_data, pos_data[i])
        lines["velocity"][i].set_data(time_data, vel_data[i])
        lines["acceleration"][i].set_data(time_data, acc_data[i])
    # Adjust axes limits dynamically based on the data
    for ax, data in zip(axs, [pos_data, vel_data, acc_data]):
        if data[0]:
            ax.set_xlim(max(0, time_data[-1] - 10), time_data[-1] + 1)
            ax.set_ylim(min(min(d) for d in data) - 5, max(max(d) for d in data) + 5)
    # Return all lines (not strictly necessary here)
    return [line for sublist in lines.values() for line in sublist]

# Initialize Pygame
pygame.init()

# Screen dimensions
WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("3D Drone Simulation (Rectangular Prism)")

# Clock to control frame rate
clock = pygame.time.Clock()
FPS = 60  # Frames per second

# Drone physical parameters
mass = 3.0           # mass in kg
thrust_power = 500.0  # thrust magnitude (N)
rotation_power = 0.75  # rotation speed (radians per second)
gravity = -100       # gravitational acceleration (m/s²)

# Drone state: 3D position and velocity
drone_pos = np.array([0, 0, 300], dtype=float)
# Initialize a transformation matrix for the drone orientation:
drone_transform = np.c_[norm(np.array([1., 0., 1.])), [0., 1., 0.], norm(np.array([-1., 0., 1.]))]
drone_vel = np.array([0.0, 0.0, 0.0], dtype=float)

# Time step per frame
dt = 1.0 / FPS

# # Orientation angles (in radians)
# phi = 0.0    # roll (rotation about the X-axis)
# theta = 0.0  # pitch (rotation about the Y-axis)
# psi = 0.0    # yaw (rotation about the Z-axis)

# Colors
WHITE = (255, 255, 255)
BLUE  = (50, 50, 255)
RED   = (255, 50, 50)
YELLOW= (255, 200, 50)
BLACK = (0, 0, 0)

# Define dimensions of the rectangular prism (drone shape)
w = 60  # width
h = 30  # height
d = 60  # depth

# Define the 8 vertices of the rectangular prism (centered at the origin)
prism_vertices = np.array([
    [-w/2, -h/2, -d/2],  # vertex 0
    [ w/2, -h/2, -d/2],  # vertex 1
    [ w/2,  h/2, -d/2],  # vertex 2
    [-w/2,  h/2, -d/2],  # vertex 3
    [-w/2, -h/2,  d/2],  # vertex 4
    [ w/2, -h/2,  d/2],  # vertex 5
    [ w/2,  h/2,  d/2],  # vertex 6
    [-w/2,  h/2,  d/2],  # vertex 7
    # Verticies for propellers
    [ w/2, h,  d/2], # 8
    [-w/2, h,  d/2], # 9
    [ w/2, h, -d/2], # 10
    [-w/2, h, -d/2]  # 11
])

# Define the edges that connect vertices (each pair represents an edge)
prism_edges = [
    (0, 1), (1, 2), (2, 3), (3, 0),  # Back face
    (4, 5), (5, 6), (6, 7), (7, 4),  # Front face
    (0, 4), (1, 5), (2, 6), (3, 7),  # Connecting edges
    (6, 8), (7, 9), (2, 10), (3, 11)    # Propellers
]

# Define vertices and edges for a bounding box (for visualization)
bb_verticies = np.array([
    [-300, -300, 0],
    [-300, -300, 600],
    [300, -300, 600],
    [300, -300, 0],
    [-300, 900, 0],
    [-300, 900, 600],
    [300, 900, 600],
    [300, 900, 0]
])

bb_edges = np.array([
    (0, 1), (1, 2), (2, 3), (3, 0), # Bottom face
    (0, 4), (1, 5), (2, 6), (3, 7), # Wall corners
])

# Coin stuff
# Function to generate a coin
def mk_coin():
    x = random.uniform(-300, 300)
    y = random.uniform(-250, 300)
    z = random.uniform(50, 600)
    return np.array([x, y, z])

# Function to draw a coin
def draw_coin(screen, pos):
    coin_size = 40
    screenspace_ctr = to_screenspace(pos)
    screenspace_edge = to_screenspace(pos + np.array([0, coin_size, 0]))
    screenspace_radius = np.linalg.norm(np.array(screenspace_ctr) - np.array(screenspace_edge))
    pygame.draw.circle(screen, YELLOW, screenspace_ctr, screenspace_radius, 2)
    # draw points on the three walls
    pos1 = np.copy(pos); pos1[0] = -300
    pos2 = np.copy(pos); pos2[0] = 300
    pos3 = np.copy(pos); pos3[2] = 600
    print(pos, pos1, pos2, pos3)
    pygame.draw.circle(screen, YELLOW, to_screenspace(pos1), 3)
    pygame.draw.circle(screen, YELLOW, to_screenspace(pos2), 3)
    pygame.draw.circle(screen, YELLOW, to_screenspace(pos3), 3)

# Function to test_collision with coion
def coin_collide(pos1, pos2):
    coin_size = 40
    return np.linalg.norm(pos1 - pos2) <= coin_size

# Rotation matrix functions
def roll_matrix(phi):
    """Rotation matrix for roll (rotation about the X-axis)."""
    return np.array([
        [1, 0, 0],
        [0, np.cos(phi), -np.sin(phi)],
        [0, np.sin(phi), np.cos(phi)]
    ])

def pitch_matrix(theta):
    """Rotation matrix for pitch (rotation about the Y-axis)."""
    return np.array([
        [np.cos(theta), 0, np.sin(theta)],
        [0, 1, 0],
        [-np.sin(theta), 0, np.cos(theta)]
    ])

def yaw_matrix(psi):
    """Rotation matrix for yaw (rotation about the Z-axis)."""
    return np.array([
        [np.cos(psi), -np.sin(psi), 0],
        [np.sin(psi), np.cos(psi), 0],
        [0, 0, 1]
    ])

def project(point):
    """
    Projects a 3D point to 2D using a simple perspective projection.
    The camera is assumed to be at the origin looking along the positive z-axis.
    """
    camera_distance = 500.0  # Focal length
    factor = camera_distance / (camera_distance + point[2])
    # [factor 0 0]
    # [0 factor 0]
    # where factor = C_d / (C_d + z)
    # x2d = point[0] * factor
    # y2d = point[1] * factor
    return factor * point[[0, 1]]

def to_screenspace(point):
    p = project(point)
    x_screen = int(WIDTH / 2 + p[0])
    y_screen = int(HEIGHT / 2 - p[1])  # Invert y for display
    return (x_screen, y_screen)

# Control map text to display on screen
control_text = [
    "Controls:",
    "E, B: Roll left/right",
    "T, C: Yaw left/right"
]

# Enable interactive mode for Matplotlib
plt.ion()
plt.show()

# Main simulation loop
running = True
frame_count = 0
graph_update_interval = 10
coin = mk_coin()
score = 0
while running:
    # --- Event Processing ---
    for event in pygame.event.get():
        if event.type == pygame.QUIT or event.type == pygame.KEYUP and event.key == pygame.K_q:
            running = False

    # --- Process Input ---
    keys = pygame.key.get_pressed()
    thrust = 0
    if keys[pygame.K_e]:
        thrust += thrust_power / 4
        drone_transform = drone_transform @ roll_matrix(-rotation_power * dt)
    if keys[pygame.K_b]:
        thrust += thrust_power / 4
        drone_transform = drone_transform @ roll_matrix(rotation_power * dt)
    if keys[pygame.K_t]:
        thrust += thrust_power / 4
        drone_transform = drone_transform @ yaw_matrix(rotation_power * dt)
    if keys[pygame.K_c]:
        thrust += thrust_power / 4
        drone_transform = drone_transform @ yaw_matrix(-rotation_power * dt)

    # --- Physics Update ---
    # Calculate acceleration from thrust and add gravity to Y-axis
    acceleration = (drone_transform @ np.array([0, thrust / mass, 0], dtype=float)) 
    acceleration[1] += gravity
    drone_vel += acceleration * dt
    drone_pos += drone_vel * dt

    # Record data for plotting
    time_data.append(time_data[-1] + dt if time_data else 0)
    for i in range(3):
        pos_data[i].append(drone_pos[i])
        vel_data[i].append(drone_vel[i])
        acc_data[i].append(acceleration[i])

    # Boundary conditions
    if drone_pos[0] < -300:
        drone_pos[0] = -300
        drone_vel[0] *= -0.75
    if drone_pos[0] > 300:
        drone_pos[0] = 300
        drone_vel[0] *= -0.75
    if drone_pos[2] < 0:
        drone_pos[2] = 0
        drone_vel[2] *= -0.75
    if drone_pos[2] > 600:
        drone_pos[2] = 600
        drone_vel[2] *= -0.75
    if drone_pos[1] < -300:
        drone_pos[1] = -300
        drone_vel = np.array([0., 0., 0.])

    # --- Orientation Update ---
    # Combine the current transform with an additional pitch rotation to correct for diagonal axes
    R = drone_transform @ pitch_matrix(-pi / 4)

    # --- Drawing with Pygame ---
    screen.fill(WHITE)

    # Draw bounding box edges
    for edge in bb_edges:
        pt1 = to_screenspace(bb_verticies[edge[0]])
        pt2 = to_screenspace(bb_verticies[edge[1]])
        pygame.draw.line(screen, RED, pt1, pt2, 2)

    draw_coin(screen, coin)

    # Transform and project each vertex of the drone
    projected_vertices = []
    for vertex in prism_vertices:
        rotated_vertex = R @ vertex
        global_vertex = rotated_vertex + drone_pos
        point_2d = project(global_vertex)
        x_screen = int(WIDTH / 2 + point_2d[0])
        y_screen = int(HEIGHT / 2 - point_2d[1])  # Invert y for display
        projected_vertices.append((x_screen, y_screen))

    pos1 = np.copy(drone_pos); pos1[0] = -300
    pos2 = np.copy(drone_pos); pos2[0] = 300
    pos3 = np.copy(drone_pos); pos3[2] = 600
    pygame.draw.circle(screen, BLUE, to_screenspace(pos1), 3)
    pygame.draw.circle(screen, BLUE, to_screenspace(pos2), 3)
    pygame.draw.circle(screen, BLUE, to_screenspace(pos3), 3)

    # Draw the edges of the drone
    for edge in prism_edges:
        pt1 = projected_vertices[edge[0]]
        pt2 = projected_vertices[edge[1]]
        pygame.draw.line(screen, BLUE, pt1, pt2, 2)

    # Display control instructions and drone data
    font = pygame.font.SysFont("Arial", 16)
    x_offset = 10
    y_offset = 10
    for line in control_text:
        text_surface = font.render(line, True, BLACK)
        screen.blit(text_surface, (x_offset, y_offset))
        y_offset += text_surface.get_height() + 2

    drone_data = [
        f"Position: x={drone_pos[0]:.2f}, y={drone_pos[1]:.2f}, z={drone_pos[2]:.2f}",
        f"Velocity: x={drone_vel[0]:.2f}, y={drone_vel[1]:.2f}, z={drone_vel[2]:.2f}",
        f"Acceleration: x={acceleration[0]:.2f}, y={acceleration[1]:.2f}, z={acceleration[2]:.2f}"
    ]
    for line in drone_data:
        text_surface = font.render(line, True, BLACK)
        screen.blit(text_surface, (x_offset, y_offset))
        y_offset += text_surface.get_height() + 2
    
    text_surface = font.render(f"Score: {score}", True, BLACK)
    screen.blit(text_surface, (x_offset, y_offset))

    # --- Update Matplotlib Plot in the Main Thread ---
    if frame_count % graph_update_interval == 0:
        update_plot(None)
        mypause(0.001)  # Short pause to allow the plot GUI to update
    if coin_collide(coin, drone_pos):
        score += 1
        coin = mk_coin()

    # --- Update Pygame Display ---
    pygame.display.flip()
    dt = clock.tick(FPS) / 1000.0  # update dt based on real frame time
    frame_count += 1  # Increment the frame counter each loop

# --- Clean Up ---
pygame.quit()
plt.close('all')  # Close all Matplotlib figures before exiting
sys.exit()
