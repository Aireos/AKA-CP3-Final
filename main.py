"""
main.py - Entry point for the terminal-driven PyBullet simulation.
Starts the simulation, spawns the terminal menu thread, and runs the
main physics loop with camera controls.
"""
import time
import threading

import pybullet as p #type: ignore

from simulation import simulation_startup, terminal_menu
from camera import camera_controls

if __name__ == "__main__":
    # Initial objects (mass, start_pos, color, radius) for spheres
    initial_spheres = [
        (1.0, [0.0, 0.0, 1.0], [0.8, 0.1, 0.1, 1.0], 0.2),
    ]

    # Initial boxes: (mass, start_pos, start_orientation_euler, color, half_extents)
    initial_boxes = [
        (1.0, [0.0, 0.0, 0.5], [0.0, 0.0, 0.0], [0.5, 0.5, 0.8, 1.0], [0.1, 0.1, 0.1]),
    ]

    camera_target = [0.0, 0.0, 0.0]

    # Initialize PyBullet simulation (connect, create objects, camera)
    created_spheres, created_boxes = simulation_startup(
        initial_spheres, initial_boxes, camera_target
    )

    # Camera state
    cam_target = camera_target.copy()
    cam_distance = 10.0
    cam_yaw = 50.0
    cam_pitch = -35.0
    cam_type = 1

    # Start terminal menu thread (daemon so it stops with the main program)
    threading.Thread(target=terminal_menu, daemon=True).start()

    # Main loop: camera control + physics stepping
    try:
        while p.isConnected():
            try:
                cam_target, cam_yaw, cam_pitch, cam_distance, cam_type = camera_controls(
                    cam_target, cam_yaw, cam_pitch, cam_distance, cam_type
                )
            except Exception as err:
                # Print camera control errors without crashing
                print("Camera error:", err)

            p.stepSimulation()
            time.sleep(1.0 / 240.0)
    except KeyboardInterrupt:
        print("Interrupted by user, exiting.")
    finally:
        if p.isConnected():
            p.disconnect()