# bodies.py
import pybullet as p
import threading
from shapes import *

duplication_done_event = threading.Event()
lock = threading.Lock()  # ensure thread safety

def list_all_bodies():
    bodies = p.getNumBodies()
    if bodies == 0:
        print("No objects in the simulation.")
        return

    print("\n--- List of Objects ---")
    for i in range(1, bodies):  # skip ground plane
        try:
            pos, orn = p.getBasePositionAndOrientation(i)
            mass = p.getDynamicsInfo(i, -1)[0]
            color = p.getVisualShapeData(i)[0][7]
            size = p.getCollisionShapeData(i, -1)[0][3]
            print(f"Body #{i} | Pos:{pos} | Mass:{mass} | Color:{color} | Size:{size}")
        except Exception as e:
            print(f"Error for body {i}: {e}")

def duplicate_object():
    duplication_done_event.clear()
    bodies = p.getNumBodies()
    if bodies <= 1:
        print("No objects to duplicate.")
        duplication_done_event.set()
        return

    body_input = input("Enter body number to duplicate (blank = last object): ")
    body_number = int(body_input) if body_input else bodies-1
    if body_number < 1 or body_number >= bodies:
        print("Body number out of range.")
        duplication_done_event.set()
        return

    amount_input = input("Enter number of duplicates (default 1): ")
    duplication_amount = int(amount_input) if amount_input else 1

    new_ids = []
    for i in range(duplication_amount):
        try:
            pos, orn = p.getBasePositionAndOrientation(body_number)
            col_shape = p.getCollisionShapeData(body_number, -1)[0]
            vis_shape = p.getVisualShapeData(body_number)[0]
            shape_type = col_shape[2]
            dimensions = col_shape[3]
            color = vis_shape[7]
            offset = 0.2 * (i+1)
            with lock:
                if shape_type == p.GEOM_BOX:
                    half_extents = [d/2 for d in dimensions]
                    new_obj = Box(
                        mass=p.getDynamicsInfo(body_number,-1)[0],
                        start_pos=[pos[0]+offset,pos[1]+offset,pos[2]+offset],
                        start_orientation=p.getEulerFromQuaternion(orn),
                        color=color,
                        half_extents=half_extents
                    )
                elif shape_type == p.GEOM_SPHERE:
                    new_obj = Sphere(
                        mass=p.getDynamicsInfo(body_number,-1)[0],
                        start_pos=[pos[0]+offset,pos[1],pos[2]],
                        color=color,
                        radius=dimensions[0]
                    )
                else:
                    print(f"Unsupported shape {shape_type}.")
                    continue
                new_ids.append(new_obj.create())
        except Exception as e:
            print(f"Error duplicating body {body_number}: {e}")

    print(f"Duplicated IDs: {new_ids}")
    duplication_done_event.set()