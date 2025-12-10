"""
properties_simple.py - edit only mass, color, and size for boxes and spheres.
Size changes are implemented by removing the existing body and creating
a new one with the same mass, color, position and orientation.
"""
import threading

import pybullet as p #type: ignore

from shapes import Box, Sphere

_lock = threading.Lock()


def change_basic_properties() -> None:
    count = p.getNumBodies()
    if count <= 1:
        print("No objects to edit.")
        return

    try:
        idx = int(input("Enter body number to edit: ").strip())
    except Exception:
        print("Invalid input.")
        return

    if idx < 1 or idx >= count:
        print("Body number out of range.")
        return

    try:
        dyn = p.getDynamicsInfo(idx, -1)
        mass = float(dyn[0])
        vis = p.getVisualShapeData(idx)
        color = vis[0][7] if vis else [1.0, 1.0, 1.0, 1.0]
        col = p.getCollisionShapeData(idx, -1)
        shape_type = col[0][2]
        dimensions = col[0][3]

        # Mass
        mass_in = input(f"Mass (current {mass}) (blank to keep): ").strip()
        if mass_in:
            try:
                mass = float(mass_in)
                p.changeDynamics(idx, -1, mass=mass)
                print("Mass updated.")
            except ValueError:
                print("Invalid mass; skipping.")

        # Color
        color_in = input(f"Color r,g,b,a (current {color}) (blank to keep): ").strip()
        if color_in:
            try:
                rgba = [float(x) for x in color_in.split(",")]
                if len(rgba) != 4:
                    raise ValueError
                with _lock:
                    p.changeVisualShape(idx, -1, rgbaColor=rgba)
                color = rgba
                print("Color updated.")
            except Exception:
                print("Invalid color; skipping.")

        # Size (recreate body)
        size_in = input("Size (box: x,y,z | sphere: radius) (blank to keep): ").strip()
        if size_in:
            with _lock:
                pos, orn = p.getBasePositionAndOrientation(idx)
                euler = p.getEulerFromQuaternion(orn)
                # Remove the old body then create the new one at same transform
                p.removeBody(idx)
                if shape_type == p.GEOM_BOX:
                    try:
                        x, y, z = [float(v) for v in size_in.split(",")]
                        half_extents = [x / 2.0, y / 2.0, z / 2.0]
                        Box(mass, pos, euler, color, half_extents).create()
                        print("Box resized.")
                    except Exception:
                        print("Invalid box dimensions; aborted resize.")
                elif shape_type == p.GEOM_SPHERE:
                    try:
                        radius = float(size_in)
                        Sphere(mass, pos, color, radius).create()
                        print("Sphere resized.")
                    except Exception:
                        print("Invalid radius; aborted resize.")
                else:
                    print("Unsupported shape for resizing.")
    except Exception as err:
        print("Error updating properties:", err)