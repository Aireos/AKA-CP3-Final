# Object classes

import pybullet as p #type: ignore
import pybullet_data #type: ignore
import time
import math
import threading

# from simulation import *
# from camera import *
# from properties import *
# from bodies import *
# from files import *

class Object:
    def __init__(self, mass=0.0, start_pos=[0,0,0], start_orientation=[0,0,0], color=[0,0,0,1]):
        self.mass = mass
        self.start_pos = start_pos
        if isinstance(start_orientation, (list,tuple)):
            self.start_orientation = p.getQuaternionFromEuler(start_orientation)
        else:
            self.start_orientation = start_orientation
        self.color = color
        self.body_id = None

    def create(self):
        pass

class Box(Object):
    def __init__(self, mass=0.0, start_pos=[0,0,0], start_orientation=[0,0,0], color=[1,1,1,1], half_extents=[0.1,0.1,0.1]):
        super().__init__(mass, start_pos, start_orientation, color)
        self.half_extents = half_extents

    def create(self):
        col_shape_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=self.half_extents)
        vis_shape_id = p.createVisualShape(p.GEOM_BOX, halfExtents=self.half_extents, rgbaColor=self.color)
        self.body_id = p.createMultiBody(self.mass, col_shape_id, vis_shape_id, self.start_pos, self.start_orientation)
        return self.body_id

class Sphere(Object):
    def __init__(self, mass=0.0, start_pos=[0,0,0], color=[0,0,0,1], radius=0.1):
        super().__init__(mass, start_pos, [0, 0, 0], color)
        self.radius = radius

    def create(self):
        col_shape_id = p.createCollisionShape(p.GEOM_SPHERE, radius=self.radius)
        vis_shape_id = p.createVisualShape(p.GEOM_SPHERE, radius=self.radius, rgbaColor=self.color)
        self.body_id = p.createMultiBody(self.mass, col_shape_id, vis_shape_id, self.start_pos, [0, 0, 0, 1])
        return self.body_id
    
def create_new_box():
    def safe_float_input(prompt, default):
        try:
            val = input(prompt)
            return float(val) if val != "" else default
        except:
            print(f"Invalid input. Using default {default}.")
            return default

    # gather inputs with error handling
    x_posisition = safe_float_input("Enter x position for new box (default 0): ", 0)
    y_position = safe_float_input("Enter y position for new box (default 0): ", 0)
    z_position = safe_float_input("Enter z position for new box (default 1): ", 1)
    color_input = input("Enter color for new box (r,g,b,a) (default 0.2,0.2,0.2,1): ")
    if color_input == "":
        color = [0.2,0.2,0.2,1]
    else:
        try:
            r, g, b, a = map(float, color_input.split(','))
            color = [r, g, b, a]
        except:
            print("Invalid color. Using default [0.2,0.2,0.2,1].")
            color = [0.2,0.2,0.2,1]
    pitch = safe_float_input("Enter pitch for new box in degrees (default 0): ", 0)
    roll = safe_float_input("Enter roll for new box in degrees (default 0): ", 0)
    yaw = safe_float_input("Enter yaw for new box in degrees (default 0): ", 0)
    length = safe_float_input("Enter box length (default 0.2): ", 0.2)
    width = safe_float_input("Enter box width (default 0.2): ", 0.2)
    height = safe_float_input("Enter box height (default 0.2): ", 0.2)
    mass = safe_float_input("Enter mass for new box (default 1): ", 1)

    box = Box(
        mass, 
        [x_posisition, y_position, z_position],
        [math.radians(roll), math.radians(pitch), math.radians(yaw)],
        color, 
        [length/2, width/2, height/2]
    )
    box.create()
    print("Created new box.")

def create_new_sphere():
    def safe_float_input(prompt, default):
        try:
            val = input(prompt)
            return float(val) if val != "" else default
        except:
            print(f"Invalid input. Using default {default}.")
            return default

    radius = safe_float_input("Enter radius for new sphere (default 0.2): ", 0.2)
    mass = safe_float_input("Enter mass for new sphere (default 1): ", 1)
    x_position = safe_float_input("Enter x position for new sphere (default 0): ", 0)
    y_position = safe_float_input("Enter y position for new sphere (default 0): ", 0)
    z_position = safe_float_input("Enter z position for new sphere (default 1): ", 1)
    color_input = input("Enter color for new sphere (r,g,b,a) (default 0.2,0.2,0.2,1): ")
    if color_input == "":
        color = [0.2,0.2,0.2,1]
    else:
        try:
            r, g, b, a = map(float, color_input.split(','))
            color = [r, g, b, a]
        except:
            print("Invalid color. Using default [0.2,0.2,0.2,1].")
            color = [0.2,0.2,0.2,1]
    sphere = Sphere(mass, [x_position, y_position, z_position], color, radius)
    sphere.create()
    print("Created new sphere.")