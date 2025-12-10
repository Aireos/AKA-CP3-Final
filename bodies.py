"""
bodies.py - object listing and duplication utilities.
This module contains thread-safe operations for duplicating bodies
and listing them in a human-friendly format.
"""
import threading
from typing import List

import pybullet as p #type: ignore

from shapes import Box, Sphere

duplication_done_event = threading.Event()
_create_lock = threading.Lock()


def _body_count() -> int:
    return p.getNumBodies()


def list_all_bodies() -> None:
    """Prints a summary of bodies (skips ground plane at index 0)."""
    count = _body_count()
    if count <= 1:
        print("No objects in the simulation.")
        return

    print("\n--- List of Objects ---")
    for body_index in range(1, count):
        try:
            pos, orn = p.getBasePositionAndOrientation(body_index)
            mass = p.getDynamicsInfo(body_index, -1)[0]
            visual = p.getVisualShapeData(body_index)
            color = visual[0][7] if visual else [1.0, 1.0, 1.0, 1.0]
            collision = p.getCollisionShapeData(body_index, -1)
            size = collision[0][3] if collision else "N/A"
            euler = p.getEulerFromQuaternion(orn)
            print(
                f"Body #{body_index} | Pos:{pos} | Euler:{euler} | Mass:{mass} | Color:{color} | Size:{size}"
            )
        except Exception as err:
            print(f"Error reading body {body_index}: {err}")


def duplicate_object() -> None:
    """
    Duplicate a chosen object. Prompts for body index (blank => last object)
    and amount (default 1). Uses a small offset per duplicate to prevent overlap.
    """
    duplication_done_event.clear()

    count = _body_count()
    if count <= 1:
        print("No objects to duplicate.")
        duplication_done_event.set()
        return

    raw = input("Enter body number to duplicate (blank = last object): ").strip()
    try:
        body_idx = int(raw) if raw != "" else count - 1
    except ValueError:
        print("Invalid body number.")
        duplication_done_event.set()
        return

    if body_idx < 1 or body_idx >= count:
        print("Body number out of range.")
        duplication_done_event.set()
        return

    raw_amount = input("Enter number of duplicates to create (default 1): ").strip()
    try:
        amount = int(raw_amount) if raw_amount != "" else 1
    except ValueError:
        print("Invalid amount; using 1.")
        amount = 1

    created_ids: List[int] = []

    for i in range(amount):
        try:
            pos, orn = p.getBasePositionAndOrientation(body_idx)
            dyn = p.getDynamicsInfo(body_idx, -1)
            mass = float(dyn[0])
            vis = p.getVisualShapeData(body_idx)
            color = vis[0][7] if vis else [1.0, 1.0, 1.0, 1.0]
            col = p.getCollisionShapeData(body_idx, -1)
            if not col:
                print("No collision information available; skipping.")
                continue
            shape_info = col[0]
            shape_type = shape_info[2]
            dimensions = shape_info[3]

            offset = 0.2 * (i + 1)
            with _create_lock:
                if shape_type == p.GEOM_BOX:
                    half_extents = [d / 2.0 for d in dimensions]
                    new_box = Box(
                        mass=mass,
                        start_pos=[pos[0] + offset, pos[1] + offset, pos[2] + offset],
                        start_orientation=p.getEulerFromQuaternion(orn),
                        color=color,
                        half_extents=half_extents,
                    )
                    created_ids.append(new_box.create())
                elif shape_type == p.GEOM_SPHERE:
                    radius = float(dimensions[0])
                    new_sphere = Sphere(
                        mass=mass,
                        start_pos=[pos[0] + offset, pos[1], pos[2]],
                        color=color,
                        radius=radius,
                    )
                    created_ids.append(new_sphere.create())
                else:
                    print(f"Unsupported shape type: {shape_type}; skipping.")
        except Exception as err:
            print("Duplication error:", err)

    print("Duplicated body IDs:", created_ids)
    duplication_done_event.set()