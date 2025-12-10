"""
simulation.py - initializes the PyBullet simulation and provides the
terminal menu thread. The terminal menu calls into other modules for
object creation, duplication, editing, and file operations.
"""
from typing import List, Tuple

import pybullet as p #type: ignore
import pybullet_data #type: ignore
import threading

from shapes import Sphere, Box, create_new_box, create_new_sphere
import bodies
import properties
import files

# Export duplication event for external modules if needed
duplication_event = bodies.duplication_done_event


def simulation_startup(
    spheres_data: List[Tuple[float, list, list, float]],
    boxes_data: List[Tuple[float, list, list, list, list]],
    camera_pos: list,
):
    """Connect to PyBullet, load the plane, spawn initial objects, and position the camera.

    Returns lists of created sphere and box body IDs.
    """
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0.0, 0.0, -9.81)
    p.loadURDF("plane.urdf")

    created_spheres = []
    created_boxes = []

    # Create spheres
    for mass, pos, color, radius in spheres_data:
        s = Sphere(mass, pos, color, radius)
        created_spheres.append(s.create())

    # Create boxes
    for data in boxes_data:
        # data: (mass, pos, euler, color, half_extents)
        b = Box(*data)
        created_boxes.append(b.create())

    p.resetDebugVisualizerCamera(10.0, 50.0, -35.0, camera_pos)
    print("Simulation started with initial objects.")
    return created_spheres, created_boxes


def terminal_menu():
    """Terminal-driven menu (runs on a separate thread)."""
    while True:
        try:
            print("\n--- Terminal Options ---")
            print("1. Duplicate object")
            print("2. Create new box")
            print("3. Create new sphere")
            print("4. List all objects")
            print("5. Edit object properties (mass/color/size)")
            print("6. List controls")
            print("7. Import shapes from file")
            print("8. Export shapes to file")
            print("9. Exit simulation")
            choice = input("> ").strip()

            if choice == "1":
                bodies.duplicate_object()
                # Wait until duplication completes
                bodies.duplication_done_event.wait()
            elif choice == "2":
                create_new_box()
            elif choice == "3":
                create_new_sphere()
            elif choice == "4":
                bodies.list_all_bodies()
            elif choice == "5":
                properties.change_basic_properties()
            elif choice == "6":
                print("\n--- Camera Controls ---")
                print("Arrow Keys: Move camera based on mode")
                print("j: Orbit mode")
                print("k: Pan mode")
                print("n: Elevate mode")
                print("m: Zoom mode")
            elif choice == "7":
                files.import_shapes()
            elif choice == "8":
                files.export_shapes()
            elif choice == "9":
                print("Closing simulation.")
                p.disconnect()
                return
            else:
                print("Invalid option.")
        except Exception as err:
            print("Menu error:", err)