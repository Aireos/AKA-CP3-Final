# properties.py
import pybullet as p
from shapes import Box, Sphere
import threading

lock = threading.Lock()

def change_properties():
    bodies = p.getNumBodies()
    if bodies <= 1:
        print("No objects to edit.")
        return

    try:
        body_number_input = input("Enter body number to edit: ")
        body_number = int(body_number_input)
    except:
        print("Invalid input.")
        return

    if body_number < 1 or body_number >= bodies:
        print("Body number out of range.")
        return

    # Get current info
    dyn_info = p.getDynamicsInfo(body_number, -1)
    mass = dyn_info[0]

    vis_data = p.getVisualShapeData(body_number)[0]
    color = vis_data[7]

    col_data = p.getCollisionShapeData(body_number, -1)[0]
    shape_type = col_data[2]
    dimensions = col_data[3]

    try:
        # --- Mass ---
        nm = input(f"Mass (current: {mass}): ")
        if nm:
            mass = float(nm)
            p.changeDynamics(body_number, -1, mass=mass)
            print(f"Mass updated to {mass}")

        # --- Color ---
        color_input = input(f"Color (r,g,b,a) (current: {color}): ")
        if color_input:
            try:
                r, g, b, a = map(float, color_input.split(','))
                color = [r, g, b, a]
                with lock:
                    p.changeVisualShape(body_number, -1, rgbaColor=color)
                print(f"Color updated to {color}")
            except:
                print("Invalid color input. Skipping.")

        # --- Size ---
        with lock:
            if shape_type == p.GEOM_BOX:
                size_input = input(f"Type x,y,z values (current: {dimensions}): ")
                # Box expects halfExtents
                try:
                    x, y, z = map(float, size_input.split(','))
                    half_extents = [x/2, y/2, z/2]
                    # Recreate box
                    pos, orn = p.getBasePositionAndOrientation(body_number)
                    p.removeBody(body_number)
                    Box(mass, pos, p.getEulerFromQuaternion(orn), color, half_extents).create()
                    print(f"Box size updated to {x},{y},{z}")
                except:
                    print("Invalid box size input. Skipping.")
            elif shape_type == p.GEOM_SPHERE:
                size_input = input(f"Type r value (current: {dimensions[0]}): ")
                try:
                    radius = float(size_input)
                    pos, orn = p.getBasePositionAndOrientation(body_number)
                    p.removeBody(body_number)
                    Sphere(mass, pos, color, radius).create()
                    print(f"Sphere radius updated to {radius}")
                except:
                    print("Invalid sphere size input. Skipping.")
    except Exception as e:
        print(f"Error updating properties: {e}")
