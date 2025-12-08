# Property editor

import pybullet as p #type: ignore
import pybullet_data #type: ignore
import time
import math
import threading

# from shapes import *
# from simulation import *
# from camera import *
# from bodies import *
# from files import *

def change_properties():
    global m, lf, rf, sf, ld, ad

    bodies = p.getNumBodies()
    if bodies == 0:
        print("No objects to edit.")
        return

    try:
        body_number_input = input("Enter body number to edit: ")
        body_number = int(body_number_input)
    except:
        print("Invalid input.")
        return

    if body_number < 0 or body_number >= bodies:
        print("Body number out of range.")
        return

    try:
        # Validate input and apply changes
        nm = input(f"Mass (current: {m}): ")
        if nm == 'reset':
            p.changeDynamics(body_number, -1, mass=1.0)
            print("Mass reset to 1.0")
        elif nm:
            m = float(nm)
            p.changeDynamics(body_number, -1, mass=m)
            print(f"Mass updated to {m}")

        nlf = input(f"Lateral Friction (current: {lf}): ")
        if nlf == 'reset':
            p.changeDynamics(body_number, -1, lateralFriction=0.5)
            print(f"Lateral Friction reset to {0.5}")
        elif nlf:
            lf = float(nlf)
            p.changeDynamics(body_number, -1, lateralFriction=lf)
            print(f"Lateral Friction updated to {lf}")

        nrf = input(f"Rolling Friction (current: {rf}): ")
        if nrf == 'reset':
            p.changeDynamics(body_number, -1, rollingFriction=0.1)
            print(f"Rolling Friction reset to {0.1}")
        elif nrf:
            rf = float(nrf)
            p.changeDynamics(body_number, -1, rollingFriction=rf)
            print(f"Rolling Friction updated to {rf}")

        nsf = input(f"Spinning Friction (current: {sf}): ")
        if nsf == 'reset':
            p.changeDynamics(body_number, -1, spinningFriction=0.1)
            print(f"Spinning Friction reset to {0.1}")
        elif nsf:
            sf = float(nsf)
            p.changeDynamics(body_number, -1, spinningFriction=sf)
            print(f"Spinning Friction updated to {sf}")

        nld = input(f"Lateral Damping (current: {ld}): ")
        if nld == 'reset':
            p.changeDynamics(body_number, -1, linearDamping=0.0)
            print(f"Linear Damping reset to {0.0}")
        elif nld:
            ld = float(nld)
            p.changeDynamics(body_number, -1, linearDamping=ld)
            print(f"Linear Damping updated to {ld}")

        nad = input(f"Angular Damping (current: {ad}): ")
        if nad == 'reset':
            p.changeDynamics(body_number, -1, angularDamping=0.0)
            print(f"Angular Damping reset to {0.0}")
        elif nad:
            ad = float(nad)
            p.changeDynamics(body_number, -1, angularDamping=ad)
            print(f"Angular Damping updated to {ad}")
    except Exception as e:
        print(f"Error updating properties: {e}")