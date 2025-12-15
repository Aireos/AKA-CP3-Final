#AKA CP3 Final Project

import threading

import pybullet as p #type: ignore

from shapes import Box, Sphere

from bodies import list_all_bodies

_lock = threading.Lock()


def change_basic_properties() -> None:
    count = p.getNumBodies()
    if count <= 1:
        print("No objects to edit.")
        return
    list_all_bodies()
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
        mass_in = input(f"Mass (current {mass}) (blank to keep) (max is 1,000): ").strip()
        if mass_in:
            try:
                new_mass = float(mass_in)
                if new_mass > 1000:
                    print("Mass too high; skipping.")
                else:
                    p.changeDynamics(bodyUniqueId=idx, linkIndex=-1, mass=new_mass)
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
        with _lock:
            pos, orn = p.getBasePositionAndOrientation(idx)
            euler = p.getEulerFromQuaternion(orn)
            # Remove the old body then create the new one at same transform
            
            if shape_type == p.GEOM_BOX:
                size_in = input("Enter new box dimensions x,y,z (blank to keep) (max is 1,000 for each): ").strip()
                if not size_in:
                    half_extents = [d / 2.0 for d in dimensions]
                else:
                    if any(float(s) > 1000 for s in size_in.split(",")):
                        print("Box dimensions too large; skipping.")
                        return
                    try:
                        x, y, z = [float(v) for v in size_in.split(",")]
                        half_extents = [x / 2.0, y / 2.0, z / 2.0]
                        p.removeBody(idx)
                        Box(mass, pos, euler, color, half_extents).create()
                        print("Box resized.")
                    except Exception:
                        print("Invalid box dimensions; skipping.")
            elif shape_type == p.GEOM_SPHERE:
                size_in = input("Enter new sphere radius (blank to keep) (max is 1,000): ").strip()
                if not size_in:
                    radius = float(dimensions[0])
                else:
                    if float(size_in) > 1000:
                        print("Sphere radius too large; skipping.")
                        return
                    try:
                        radius = float(size_in)
                        p.removeBody(idx)
                        Sphere(mass, pos, color, radius).create()
                        print("Sphere resized.")
                    except Exception:
                        print("Invalid radius; skipping.")
            else:
                print("Unsupported shape for resizing.")
    except Exception as err:
        print("Error updating properties:", err)