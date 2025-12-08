# CP3 Final with terminal-based object controls (No sliders, no selection)
# pip install pybullet pybullet_data numpy

import pybullet as p #type: ignore
import pybullet_data #type: ignore
import time
import math
import threading

from shapes import *
from simulation import *
from camera import *
from properties import *
from bodies import *
from files import *

# Initialize the event for duplication synchronization
duplication_done_event = threading.Event()

# defining basic shape properties
# mass
m = 1.0
# lateral friction
lf = 0.5
# rolling friction
rf = 0.1
# spinning friction
sf = 0.1
# linear damping
ld = 0.0
# angular damping
ad = 0.0

if __name__=="__main__":
    base_sphere_data = [
        # mass, x, y, z, 
        [1.0, [0, 0, 1], [0.8, 0.1, 0.1, 1], 0.2]
    ]
    base_box_data = [
        [1.0, [0,0,0.5], [0,0,0], [0.5,0.5,0.8,1], [0.1,0.1,0.1]]
    ]
    starting_cam_target_pos = [0,0,0]

    simulation_startup(base_sphere_data, base_box_data, starting_cam_target_pos)

    cam_distance = 10
    cam_yaw = 50
    cam_pitch = -35
    cam_type = 1

    # Start terminal input thread
    threading.Thread(target=terminal_menu, daemon=True).start()

    # Main loop
    while p.isConnected():
        try:
            cam_target_pos, cam_yaw, cam_pitch, cam_distance, cam_type = camera_controls(cam_target_pos, cam_yaw, cam_pitch, cam_distance, cam_type)
        except:
            pass

        p.stepSimulation()
        time.sleep(1./240.)

    # Graceful disconnect
    if p.isConnected():
        p.disconnect()