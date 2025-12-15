#AKA CP3 Final Project

import threading

import pybullet as p #type: ignore

from shapes import Box, Sphere

_io_lock = threading.Lock()

def import_shapes() -> None:
    name = input("Enter filename to import (txt): ").strip()
    if not name.endswith(".txt"):
        name += ".txt"

    try:
        with open(name, "r") as file:
            lines = [ln.strip() for ln in file if ln.strip()]
    except Exception as e:
        print("File error:", e)
        return

    with _io_lock:
        for line in lines:
            parts = line.split(",")
            if not parts:
                continue
            kind = parts[0].lower()
            try:
                if kind == "box" and len(parts) >= 15:
                    mass = float(parts[1])
                    pos = [float(parts[2]), float(parts[3]), float(parts[4])]
                    euler = [float(parts[5]), float(parts[6]), float(parts[7])]
                    color = [float(parts[8]), float(parts[9]), float(parts[10]), float(parts[11])]
                    half = [float(parts[12]), float(parts[13]), float(parts[14])]
                    Box(mass, pos, euler, color, half).create()
                elif kind == "sphere" and len(parts) >= 9:
                    mass = float(parts[1])
                    pos = [float(parts[2]), float(parts[3]), float(parts[4])]
                    color = [float(parts[5]), float(parts[6]), float(parts[7]), float(parts[8])]
                    radius = float(parts[9])
                    Sphere(mass, pos, color, radius).create()
                else:
                    print("Skipped malformed line:", line)
            except Exception as e:
                print("Import error for line:", line, "->", e)
    print("Import completed.")


def export_shapes() -> None:
    name = input("Enter filename to export (txt): ").strip()
    if not name.endswith(".txt"):
        name += ".txt"

    try:
        with open(name, "w") as fh:
            count = p.getNumBodies()
            if count <= 1:
                print("No objects to export.")
                return
            for i in range(1, count):
                try:
                    pos, orn = p.getBasePositionAndOrientation(i)
                    euler = p.getEulerFromQuaternion(orn)
                    mass = p.getDynamicsInfo(i, -1)[0]
                    vis = p.getVisualShapeData(i)[0]
                    color = vis[7]
                    col = p.getCollisionShapeData(i, -1)[0]
                    shape_type = col[2]
                    dims = col[3]
                    if shape_type == p.GEOM_BOX:
                        half = [d / 2.0 for d in dims]
                        line = (
                            f"box,{mass},{pos[0]},{pos[1]},{pos[2]},{euler[0]},{euler[1]},{euler[2]},"
                            f"{color[0]},{color[1]},{color[2]},{color[3]},{half[0]},{half[1]},{half[2]}\n"
                        )
                    elif shape_type == p.GEOM_SPHERE:
                        radius = dims[0]
                        line = (
                            f"sphere,{mass},{pos[0]},{pos[1]},{pos[2]},{color[0]},{color[1]},{color[2]},"
                            f"{color[3]},{radius}\n"
                        )
                    else:
                        continue
                    fh.write(line)
                except Exception as e:
                    print("Export error for body", i, ":", e)
        print("Export completed.")
    except Exception as err:
        print("File write error:", err)