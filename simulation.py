"""
simulation.py - initializes the PyBullet simulation and provides the
terminal menu thread. The terminal menu calls into other modules for
object creation, duplication, editing, and file operations.
"""
from typing import List, Tuple

import pybullet as p #type: ignore
import pybullet_data #type: ignore
import threading
import os

from shapes import Sphere, Box, create_new_box, create_new_sphere
import bodies
import properties
import files

# Export duplication event for external modules if needed
duplication_event = bodies.duplication_done_event


def simulation_startup(spheres_data_list, boxes_data_list, camera_pos):
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    # ---- FIX DOUBLE GRID ----
    # Try disabling the debug grid (new PyBullet)
    try:
        p.configureDebugVisualizer(p.COV_ENABLE_GRID, 0)
    except Exception:
        # Older PyBullet: grid is part of the GUI overlay
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

    # Load plane
    p.loadURDF("plane.urdf")

    # Create shapes
    spheres = [
        Sphere(mass, start_pos, color, radius)
        for (mass, start_pos, color, radius) in spheres_data_list
    ]
    boxes = [Box(*data) for data in boxes_data_list]

    for box in boxes:
        box.create()

    for sphere in spheres:
        sphere.create()

    # Set initial camera
    p.resetDebugVisualizerCamera(
        10, 50, -35, camera_pos
    )

    return spheres, boxes


def terminal_menu():
    os.system('cls')
    """Terminal-driven menu (runs on a separate thread)."""
    print("\n--- Camera Controls ---")
    print("Arrow Keys: Move camera based on mode")
    print("j: Orbit mode")
    print("k: Pan mode")
    print("n: Elevate mode")
    print("m: Zoom mode")
    checker = None
    while checker == None:
        checker = input("Please half screen your terminal window for optimal experience (press Enter to continue): ")
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
            choice = input("Input: ").strip()
            os.system('cls')
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