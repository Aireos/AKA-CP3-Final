# Body managment

import pybullet as p #type: ignore
import pybullet_data #type: ignore
import time
import math
import threading

from shapes import *
# from simulation import *
# from camera import *
# from properties import *
# from files import *

def list_all_bodies():
    bodies = p.getNumBodies()
    if bodies == 0:
        print("No objects in the simulation.")
        return

    print("\n--- List of Objects ---")
    for i in range(bodies):
        if i > 0:
            try:
                pos, orn = p.getBasePositionAndOrientation(i)
                mass = p.getDynamicsInfo(i, -1)[0]
                shape_data = p.getVisualShapeData(i)
                color = shape_data[0][7] if shape_data else [1,1,1,1]
                collision_data = p.getCollisionShapeData(i, -1)
                size = collision_data[0][3] if collision_data else "N/A"
                print(f"Body #:{i}\nPosition:{pos}\nOrientation: {p.getEulerFromQuaternion(orn)}\nMass: {mass}\nColor: {color}\nSize: {size}\n")
            except Exception as e:
                print(f"Error retrieving data for body {i}: {e}")

def duplicate_object():
    global duplication_done_event
    duplication_done_event.clear()  # Reset the event

    bodies = p.getNumBodies()

    if bodies == 0:
        print("No objects to duplicate.")
        duplication_done_event.set()  # Signal completion even if nothing is done
        return
    
    body_number_input = input("Enter body number to duplicate (or blank for last object): ")

    if body_number_input == "":
        body_number = bodies - 1
    else:
        try:
            body_number = int(body_number_input)
        except ValueError:
            print("Invalid body number.")
            duplication_done_event.set()
            return

    if body_number < 0 or body_number >= bodies:
        print("Body number out of range.")
        duplication_done_event.set()
        return

    try:
        duplication_amount_input = input("Enter number of duplicates to create (default 1): ")
        if duplication_amount_input == "":
            duplication_amount = 1
        else:
            duplication_amount = int(duplication_amount_input)
    except:
        print("Invalid number. Using default 1.")
        duplication_amount = 1

    first_duplicated_id = None
    last_duplicated_id = None

    for i in range(duplication_amount):
        try:
            pos, orn = p.getBasePositionAndOrientation(body_number)
            mass = p.getDynamicsInfo(body_number, -1)[0]
            visual_shape_data = p.getVisualShapeData(body_number)
            color = visual_shape_data[0][7] if visual_shape_data else [1,1,1,1]
            col_shape_data = p.getCollisionShapeData(body_number, -1)
            if not col_shape_data:
                print("No collision shape data found; cannot duplicate.")
                break
            shape_info = col_shape_data[0]
            shape_type = shape_info[2]
            dimensions = shape_info[3]
            euler_orn = p.getEulerFromQuaternion(orn)
            offset_value = 0.2
            if shape_type == p.GEOM_BOX:
                offset = [offset_value, offset_value, offset_value]
                new_pos = [pos[0] + offset[0], pos[1] + offset[1], pos[2] + offset[2]]
                half_extents = [d / 2 for d in dimensions]
                new_obj = Box(
                    mass=mass,
                    start_pos=new_pos,
                    start_orientation=euler_orn,
                    color=color,
                    half_extents=half_extents
                )
            elif shape_type == p.GEOM_SPHERE:
                radius = float(dimensions[0])
                new_pos = [pos[0] + offset_value, pos[1], pos[2]]
                new_obj = Sphere(
                    mass=mass,
                    start_pos=new_pos,
                    color=color,
                    radius=radius
                )
            else:
                print(f"Unsupported shape type ({shape_type}) for duplication.")
                break
            if i == 0:
                first_duplicated_id = new_obj.create()
            elif i == duplication_amount - 1:
                last_duplicated_id = new_obj.create()
            else:
                new_obj.create()
        except Exception as e:
            print(f"Error during duplication: {e}")
            break

    print(f"Duplicated body {body_number} {duplication_amount} time(s) in id range {first_duplicated_id} to {last_duplicated_id}.")
    # Signal that duplication is complete
    duplication_done_event.set()