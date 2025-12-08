# File pulling and pushing

import pybullet as p #type: ignore
import pybullet_data #type: ignore
import time
import math
import threading

from shapes import *
from simulation import *
from camera import *
from properties import *
from bodies import *

def import_shapes():
    file_name = input("Enter filename to import shapes from: ")
    file_name = file_name if file_name.endswith('.txt') else file_name + '.txt'
    try:
        with open(file_name, 'r') as f:
            lines = f.readlines()
            for line in lines:
                data = line.strip().split(',')
                shape_type = data[0]
                if shape_type == 'box':
                    mass = float(data[1])
                    pos = list(map(float, data[2:5]))
                    orn = list(map(float, data[5:8]))
                    color = list(map(float, data[8:12]))
                    half_extents = list(map(float, data[12:15]))
                    box = Box(mass, pos, orn, color, half_extents)
                    box.create()
                elif shape_type == 'sphere':
                    mass = float(data[1])
                    pos = list(map(float, data[2:5]))
                    color = list(map(float, data[5:9]))
                    radius = float(data[9])
                    sphere = Sphere(mass, pos, color, radius)
                    sphere.create()
        print("Import completed.")
    except Exception as e:
        print(f"Error importing shapes: {e}")

def export_shapes():
    file_name = input("Enter filename to export shapes to: ")
    file_name = file_name if file_name.endswith('.txt') else file_name + '.txt'
    try:
        with open(file_name, 'w') as f:
            bodies = p.getNumBodies()
            if bodies == 0:
                print("No objects to export.")
                return
            for i in range(bodies):
                if i == 0:
                    continue # Skip ground plane
                pos, orn = p.getBasePositionAndOrientation(i)
                mass = p.getDynamicsInfo(i, -1)[0]
                shape_data = p.getVisualShapeData(i)
                color = shape_data[0][7] if shape_data else [1,1,1,1]
                collision_data = p.getCollisionShapeData(i, -1)
                if not collision_data:
                    continue
                shape_info = collision_data[0]
                shape_type = shape_info[2]
                dimensions = shape_info[3]
                if shape_type == p.GEOM_BOX:
                    half_extents = [d / 2 for d in dimensions]
                    line = f"box,{mass},{pos[0]},{pos[1]},{pos[2]},{p.getEulerFromQuaternion(orn)[0]},{p.getEulerFromQuaternion(orn)[1]},{p.getEulerFromQuaternion(orn)[2]},{color[0]},{color[1]},{color[2]},{color[3]},{half_extents[0]},{half_extents[1]},{half_extents[2]}\n"
                    f.write(line)
                elif shape_type == p.GEOM_SPHERE:
                    radius = float(dimensions[0])
                    line = f"sphere,{mass},{pos[0]},{pos[1]},{pos[2]},{color[0]},{color[1]},{color[2]},{color[3]},{radius}\n"
                    f.write(line)
        print("Export completed.")
    except Exception as e:
        print(f"Error exporting shapes: {e}")