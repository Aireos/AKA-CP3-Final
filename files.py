# files.py
import pybullet as p
from shapes import Box, Sphere
import threading

lock = threading.Lock()  # ensure thread-safe creation

def import_shapes():
    file_name = input("Enter filename to import shapes from: ")
    if not file_name.endswith('.txt'):
        file_name += '.txt'

    try:
        with open(file_name, 'r') as f:
            lines = f.readlines()
            for line in lines:
                data = line.strip().split(',')
                shape_type = data[0].lower()
                with lock:
                    if shape_type == 'box':
                        mass = float(data[1])
                        pos = list(map(float, data[2:5]))
                        orn = list(map(float, data[5:8]))
                        color = list(map(float, data[8:12]))
                        half_extents = list(map(float, data[12:15]))
                        Box(mass, pos, orn, color, half_extents).create()
                    elif shape_type == 'sphere':
                        mass = float(data[1])
                        pos = list(map(float, data[2:5]))
                        color = list(map(float, data[5:9]))
                        radius = float(data[9])
                        Sphere(mass, pos, color, radius).create()
        print("Import completed.")
    except Exception as e:
        print(f"Error importing shapes: {e}")

def export_shapes():
    file_name = input("Enter filename to export shapes to: ")
    if not file_name.endswith('.txt'):
        file_name += '.txt'

    try:
        with open(file_name, 'w') as f:
            bodies = p.getNumBodies()
            if bodies <= 1:
                print("No objects to export.")
                return
            for i in range(1, bodies):
                pos, orn = p.getBasePositionAndOrientation(i)
                mass = p.getDynamicsInfo(i, -1)[0]
                vis_data = p.getVisualShapeData(i)[0]
                color = vis_data[7]
                col_data = p.getCollisionShapeData(i, -1)[0]
                shape_type = col_data[2]
                dimensions = col_data[3]

                if shape_type == p.GEOM_BOX:
                    half_extents = [d/2 for d in dimensions]
                    euler = p.getEulerFromQuaternion(orn)
                    line = f"box,{mass},{pos[0]},{pos[1]},{pos[2]},{euler[0]},{euler[1]},{euler[2]},{color[0]},{color[1]},{color[2]},{color[3]},{half_extents[0]},{half_extents[1]},{half_extents[2]}\n"
                    f.write(line)
                elif shape_type == p.GEOM_SPHERE:
                    radius = dimensions[0]
                    line = f"sphere,{mass},{pos[0]},{pos[1]},{pos[2]},{color[0]},{color[1]},{color[2]},{color[3]},{radius}\n"
                    f.write(line)
        print("Export completed.")
    except Exception as e:
        print(f"Error exporting shapes: {e}")
