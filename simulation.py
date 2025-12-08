# Simulation setup

import pybullet as p #type: ignore
import pybullet_data #type: ignore
import time
import math
import threading

from shapes import *
from camera import *
from properties import *
from bodies import *
from files import *

def simulation_startup(spheres_data_list, boxes_data_list, camera_pos):
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0,0,-9.81)
    p.loadURDF("plane.urdf")

    spheres = [Sphere(mass, start_pos, color, radius) for (mass, start_pos, color, radius) in spheres_data_list]
    boxes = [Box(*data) for data in boxes_data_list]

    for box in boxes:
        box.create()
    for sphere in spheres:
        sphere.create()

    p.resetDebugVisualizerCamera(10,50,-35,camera_pos)
    return spheres, boxes

def terminal_menu():
    while True:
        print("\n--- Terminal Options ---")
        print("1. Duplicate object")
        print("2. Create new box")
        print("3. Create new sphere")
        print("4. List all objects")
        print("5. Edit object properties")
        print("6. List controls")
        print("7. Import shapes")
        print("8. Export shapes")
        print("9. Exit simulation")
        choice = input("> ")

        if choice == "1":
            duplicate_object()
            # Wait until duplication is finished before returning to menu
            duplication_done_event.wait()
        elif choice == "2":
            create_new_box()
        elif choice == "3":
            create_new_sphere()
        elif choice == "4":
            list_all_bodies()
        elif choice == "5":
            change_properties()
        elif choice == "6":
            print("\n--- Camera Controls ---")
            print("Arrow Keys: Move camera based on mode")
            print("j: Switch to Orbit Mode")
            print("k: Switch to Pan Mode")
            print("n: Switch to Elevate Mode")
            print("m: Switch to Zoom Mode")
        elif choice == "7":
            import_shapes()
        elif choice == "8":
            export_shapes()
        elif choice == "9":
            print("Closing simulation.")
            p.disconnect()
            return
        else:
            print("Invalid option.")