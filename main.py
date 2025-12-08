# main.py
import pybullet as p
import time
import threading

from shapes import *
from simulation import *
from camera import *
from properties import *
from bodies import *
from files import *

# Initialize the duplication event for synchronization
duplication_done_event = threading.Event()

if __name__ == "__main__":
    # Sample initial objects
    base_sphere_data = [
        [1.0, [0, 0, 1], [0.8, 0.1, 0.1, 1], 0.2]
    ]
    base_box_data = [
        [1.0, [0,0,0.5], [0,0,0], [0.5,0.5,0.8,1], [0.1,0.1,0.1]]
    ]
    starting_cam_target_pos = [0,0,0]

    # Start simulation
    spheres, boxes = simulation_startup(base_sphere_data, base_box_data, starting_cam_target_pos)

    # Camera initial state
    cam_target_pos = starting_cam_target_pos.copy()
    cam_distance = 10
    cam_yaw = 50
    cam_pitch = -35
    cam_type = 1

    # Start terminal menu in separate thread
    threading.Thread(target=terminal_menu, daemon=True).start()

    # Main simulation loop
    while p.isConnected():
        try:
            cam_target_pos, cam_yaw, cam_pitch, cam_distance, cam_type = camera_controls(
                cam_target_pos, cam_yaw, cam_pitch, cam_distance, cam_type
            )
        except Exception as e:
            print("Camera control error:", e)

        p.stepSimulation()
        time.sleep(1./240.)

    # Graceful disconnect
    if p.isConnected():
        p.disconnect()