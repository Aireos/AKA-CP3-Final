# CP3 Final with terminal-based object controls (No sliders, no selection)
# pip install pybullet pybullet_data numpy

import pybullet as p #type: ignore
import pybullet_data #type: ignore
import time
import math
import numpy as np #type: ignore
import threading
import sys

# -------------------------------
# Object Classes
# -------------------------------
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
        # Removed start_orientation parameter
        super().__init__(mass, start_pos, [0, 0, 0], color)
        self.radius = radius

    def create(self):
        col_shape_id = p.createCollisionShape(p.GEOM_SPHERE, radius=self.radius)
        vis_shape_id = p.createVisualShape(p.GEOM_SPHERE, radius=self.radius, rgbaColor=self.color)
        # Use default orientation [0,0,0,1]
        self.body_id = p.createMultiBody(self.mass, col_shape_id, vis_shape_id, self.start_pos, [0, 0, 0, 1])
        return self.body_id

# -------------------------------
# Simulation Setup
# -------------------------------
def simulation(spheres_data_list, boxes_data_list, camera_pos):
    p.connect(p.GUI, options="--window_title=Custom Simulation")
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0,0,-9.81)
    p.loadURDF("plane.urdf")

    spheres = [Sphere(mass, start_pos, color, radius) for (mass, start_pos, color, radius) in spheres_data_list]
    boxes = [Box(*data) for data in boxes_data_list]

    for box in boxes:
        box.create()
    for sphere in spheres:
        sphere.create()

    p.resetDebugVisualizerCamera(10,50,-35,camera_pos)
    return spheres, boxes

# -------------------------------
# Camera Controls
# -------------------------------
def handle_camera_controls(camera_target_pos, camera_yaw, camera_pitch, camera_distance, cam_type):
    keys = p.getKeyboardEvents()
    move_speed = 0.5
    yaw_rad = math.radians(camera_yaw)

    if cam_type == 1:
        # Orbit yaw/pitch
        if keys.get(p.B3G_RIGHT_ARROW) == p.KEY_IS_DOWN:
            camera_yaw += 2
        if keys.get(p.B3G_LEFT_ARROW) == p.KEY_IS_DOWN:
            camera_yaw -= 2
        if keys.get(p.B3G_DOWN_ARROW) == p.KEY_IS_DOWN and camera_pitch < 88:
            camera_pitch += 2
        if keys.get(p.B3G_UP_ARROW) == p.KEY_IS_DOWN and camera_pitch > -88:
            camera_pitch -= 2

    elif cam_type == 2:
        # Pan in X-Y plane based on yaw
        forward_x = math.cos(yaw_rad) * move_speed
        forward_y = math.sin(yaw_rad) * move_speed
        strafe_x = math.cos(yaw_rad + math.pi/2) * move_speed
        strafe_y = math.sin(yaw_rad + math.pi/2) * move_speed
        if keys.get(p.B3G_RIGHT_ARROW) == p.KEY_IS_DOWN:
            camera_target_pos[0] += forward_x
            camera_target_pos[1] += forward_y
        if keys.get(p.B3G_LEFT_ARROW) == p.KEY_IS_DOWN:
            camera_target_pos[0] -= forward_x
            camera_target_pos[1] -= forward_y
        if keys.get(p.B3G_UP_ARROW) == p.KEY_IS_DOWN:
            camera_target_pos[0] += strafe_x
            camera_target_pos[1] += strafe_y
        if keys.get(p.B3G_DOWN_ARROW) == p.KEY_IS_DOWN:
            camera_target_pos[0] -= strafe_x
            camera_target_pos[1] -= strafe_y

    elif cam_type == 3:
        # Elevate
        if keys.get(p.B3G_UP_ARROW) == p.KEY_IS_DOWN:
            camera_target_pos[2] += move_speed
        if keys.get(p.B3G_DOWN_ARROW) == p.KEY_IS_DOWN:
            camera_target_pos[2] -= move_speed

    elif cam_type == 4:
        # Zoom
        if keys.get(p.B3G_UP_ARROW) == p.KEY_IS_DOWN:
            camera_distance -= 0.5
        if keys.get(p.B3G_DOWN_ARROW) == p.KEY_IS_DOWN:
            camera_distance += 0.5

    # Change camera mode
    if keys.get(ord('j')) == p.KEY_IS_DOWN:
        cam_type = 1
    if keys.get(ord('k')) == p.KEY_IS_DOWN:
        cam_type = 2
    if keys.get(ord('n')) == p.KEY_IS_DOWN:
        cam_type = 3
    if keys.get(ord('m')) == p.KEY_IS_DOWN:
        cam_type = 4

    p.resetDebugVisualizerCamera(camera_distance, camera_yaw, camera_pitch, camera_target_pos)
    return camera_target_pos, camera_yaw, camera_pitch, camera_distance, cam_type

# -------------------------------
# Object Actions
# -------------------------------
# Define default values for properties
rolling_friction = 0.1
spinning_friction = 0.1
linear_damping = 0.0
angular_damping = 0.0

def change_properties():
    global rolling_friction, spinning_friction, linear_damping, angular_damping

    bodies = p.getNumBodies()
    if bodies == 0:
        print("No objects to edit.")
        return

    body_number = int(input("Enter body number to edit: "))
    if body_number < 0 or body_number >= bodies:
        print("Body number out of range.")
        return

    # Get current info
    dyn_info = p.getDynamicsInfo(body_number, -1)

    # -------- Mass --------
    new_mass = input(f"Mass (current: {dyn_info[0]}): ")
    if new_mass == 'reset':
        p.changeDynamics(body_number, -1, mass=1.0)
        print("Mass reset to 1.0")
    elif new_mass:
        p.changeDynamics(body_number, -1, mass=float(new_mass))
        print(f"Mass updated to {new_mass}")
    print()
    # -------- Lateral Friction --------
    lat = input(f"Lateral Friction (current: {dyn_info[1]}): ")
    if lat:
        p.changeDynamics(body_number, -1, lateralFriction=float(lat))
        print(f"Lateral Friction updated to {lat}")
    print()
    # -------- Rolling Friction --------
    rf = input(f"Rolling Friction (current: {rolling_friction}): ")
    if rf == 'reset':
        p.changeDynamics(body_number, -1, rollingFriction=0.1)
        print(f"Rolling Friction reset to {0.1}")
    elif rf:
        val = float(rf)
        p.changeDynamics(body_number, -1, rollingFriction=val)
        print(f"Rolling Friction updated to {val}")
    print()
    # -------- Spinning Friction --------
    sf = input(f"Spinning Friction (current: {spinning_friction}): ")
    if sf == 'reset':
        p.changeDynamics(body_number, -1, spinningFriction=0.1)
        print(f"Spinning Friction reset to {0.1}")
    elif sf:
        val = float(sf)
        p.changeDynamics(body_number, -1, spinningFriction=val)
        print(f"Spinning Friction updated to {val}")
    print()
    # -------- Linear Damping --------
    print(f"Linear Damping (current: {linear_damping})")
    ld = input(f"New Linear Damping: ")
    if ld == 'reset':
        p.changeDynamics(body_number, -1, linearDamping=0.0)
        print(f"Linear Damping reset to {0.0}")
    elif ld:
        val = float(ld)
        p.changeDynamics(body_number, -1, linearDamping=val)
        print(f"Linear Damping updated to {val}")
    print()
    # -------- Angular Damping --------
    print(f"Angular Damping (current: {angular_damping})")
    ad = input(f"New Angular Damping: ")
    if ad == 'reset':
        p.changeDynamics(body_number, -1, angularDamping=0.0)
        print(f"Angular Damping reset to {0.0}")
    elif ad:
        val = float(ad)
        p.changeDynamics(body_number, -1, angularDamping=val)
        print(f"Angular Damping updated to {val}")

def list_all_bodies():
    bodies = p.getNumBodies()
    if bodies == 0:
        print("No objects in the simulation.")
        return

    print("\n--- List of Objects ---")
    for i in range(bodies):
        pos, orn = p.getBasePositionAndOrientation(i)
        mass = p.getDynamicsInfo(i, -1)[0]
        color = p.getVisualShapeData(i)[0][7] if p.getVisualShapeData(i) else [1,1,1,1]
        print(f"Body #:{i}\nPosition:{pos}\nOrientation: {p.getEulerFromQuaternion(orn)}\nMass: {mass}\nColor: {color}\nSize: {p.getCollisionShapeData(i, -1)[0][3]}")

def duplicate_object():
    bodies = p.getNumBodies()

    if bodies == 0:
        print("No objects to duplicate.")
        return
    
    body_number = input("Enter body number to duplicate (or blank for last object): ")

    if body_number == "":
        body_number = bodies - 1
    else:
        try:
            body_number = int(body_number)
        except ValueError:
            print("Invalid body number.")
            return

    if body_number < 0 or body_number >= bodies:
        print("Body number out of range.")
        return

    # Get pose & mass
    pos, orn = p.getBasePositionAndOrientation(body_number)
    mass = p.getDynamicsInfo(body_number, -1)[0]

    # Get visual shape data (for color)
    visual_shape_data = p.getVisualShapeData(body_number)
    color = visual_shape_data[0][7] if visual_shape_data else [1,1,1,1]

    # Get collision shape data for base link (-1)
    col_shape_data = p.getCollisionShapeData(body_number, -1)
    if not col_shape_data:
        print("No collision shape data found; cannot duplicate.")
        return

    shape_info = col_shape_data[0]
    shape_type = shape_info[2]
    dimensions = shape_info[3]  # Size parameters

    # Convert orientation to Euler for constructors
    euler_orn = p.getEulerFromQuaternion(orn)

    # Use a fixed small offset to avoid overlap
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
        return

    new_body_id = new_obj.create()
    print(f"Duplicated object {body_number} as new object (id: {new_body_id}, mass={mass}).")

def create_new_box():
    # x position
    x_posisition = input("Enter x position for new box (default 0): ")
    if x_posisition == "":
        x_posisition = 0.0
    else:
        try:
            x_posisition = float(x_posisition)
        except:
            print("Invalid x position. Using default 0.")
            x_posisition = 0.0
    # y position
    y_position = input("Enter y position for new box (default 0): ")
    if y_position == "":
        y_position = 0.0
    else:
        try:
            y_position = float(y_position)
        except:
            print("Invalid y position. Using default 0.")
            y_position = 0.0
    # z position
    z_position = input("Enter z position for new box (default 1): ")
    if z_position == "":
        z_position = 1.0
    else:
        try:
            z_position = float(z_position)
        except:
            print("Invalid z position. Using default 1.")
            z_position = 1.0
    # color
    color = input("Enter color for new box (r,g,b,a) (default 0.2,0.2,0.2,1): ")
    if color == "":
        color = [0.2,0.2,0.2,1]
    else:
        try:
            r, g, b, a = map(float, color.split(','))
            color = [r, g, b, a]
        except:
            print("Invalid color. Using default [0.2,0.2,0.2,1].")
            color = [0.2,0.2,0.2,1]
    # pitch
    pitch = input("Enter pitch for new box in degrees (default 0): ")
    if pitch == "":
        pitch = 0.0
    else:
        try:
            pitch = float(pitch)
        except:
            print("Invalid pitch. Using default 0.")
            pitch = 0.0
    # roll
    roll = input("Enter roll for new box in degrees (default 0): ")
    if roll == "":
        roll = 0.0
    else:
        try:
            roll = float(roll)
        except:
            print("Invalid roll. Using default 0.")
            roll = 0.0
    # yaw
    yaw = input("Enter yaw for new box in degrees (default 0): ")
    if yaw == "":
        yaw = 0.0
    else:
        try:
            yaw = float(yaw)
        except:
            print("Invalid yaw. Using default 0.")
            yaw = 0.0
    # length
    length = input("Enter box length (default 0.2): ")
    if length == "":
        length = 0.2
    else:
        try:
            length = float(length)
        except:
            print("Invalid length. Using default 0.2.")
            length = 0.2
    # width
    width = input("Enter box width (default 0.2): ")
    if width == "":
        width = 0.2
    else:
        try:
            width = float(width)
        except:
            print("Invalid width. Using default 0.2.")
            width = 0.2
    # height
    height = input("Enter box height (default 0.2): ")
    if height == "":
        height = 0.2
    else:
        try:
            height = float(height)
        except:
            print("Invalid height. Using default 0.2.")
            height = 0.2
    # mass
    mass = input("Enter mass for new box (default 1): ")
    if mass == "":
        mass = 1.0
    else:
        try:
            mass = float(mass)
        except:
            print("Invalid mass. Using default 1.")
            mass = 1.0

    box = Box(mass,[x_posisition,y_position,z_position],[math.radians(roll),math.radians(pitch),math.radians(yaw)],color,[length/2,width/2,height/2])
    box.create()
    print("Created new box.")

def create_new_sphere():
    # radius
    radius = input("Enter radius for new sphere (default 0.2): ")
    if radius == "":
        radius = 0.2
    else:
        try:
            radius = float(radius)
        except:
            print("Invalid radius. Using default 0.2.")
            radius = 0.2
    # mass
    mass = input("Enter mass for new sphere (default 1): ")
    if mass == "":
        mass = 1.0
    else:
        try:
            mass = float(mass)
        except:
            print("Invalid mass. Using default 1.")
            mass = 1.0
    # position x
    x_position = input("Enter x position for new sphere (default 0): ")
    if x_position == "":
        x_position = 0.0
    else:
        try:
            x_position = float(x_position)
        except:
            print("Invalid x position. Using default 0.")
            x_position = 0.0
    # position y
    y_position = input("Enter y position for new sphere (default 0): ")
    if y_position == "":
        y_position = 0.0
    else:
        try:
            y_position = float(y_position)
        except:
            print("Invalid y position. Using default 0.")
            y_position = 0.0
    # position z
    z_position = input("Enter z position for new sphere (default 1): ")
    if z_position == "":
        z_position = 1.0
    else:
        try:
            z_position = float(z_position)
        except:
            print("Invalid z position. Using default 1.")
            z_position = 1.0
    # color
    color = input("Enter color for new sphere (r,g,b,a) (default 0.2,0.2,0.2,1): ")
    if color == "":
        color = [0.2,0.2,0.2,1]
    else:
        try:
            r, g, b, a = map(float, color.split(','))
            color = [r, g, b, a]
        except:
            print("Invalid color. Using default [0.2,0.2,0.2,1].")
            color = [0.2,0.2,0.2,1]
    sphere = Sphere(mass,[x_position,y_position,z_position],color,radius)
    sphere.create()
    print("Created new sphere.")

# -------------------------------
# Terminal Menu Thread
# -------------------------------
def terminal_menu():
    while True:
        print("\n--- Terminal Options ---")
        print("1. Duplicate object")
        print("2. Create new box")
        print("3. Create new sphere")
        print("4. List all objects")
        print("5. Edit object properties")
        print("6. List controls")
        print("7. Exit simulation")
        choice = input("> ")

        if choice == "1":
            duplicate_object()
        elif choice == "2":
            create_new_box()
        elif choice == "3":
            create_new_sphere()
        elif choice == "4":
            list_all_bodies()
        elif choice == "5":
            change_properties()
        elif choice == "6":
            print("\n--- Camera Controls ---")
            print("Arrow Keys: Move camera based on mode")
            print("j: Switch to Orbit Mode")
            print("k: Switch to Pan Mode")
            print("n: Switch to Elevate Mode")
            print("m: Switch to Zoom Mode")
        elif choice == "7":
            print("Closing simulation.")
            p.disconnect()
            return
        else:
            print("Invalid option.")

# -------------------------------
# Main
# -------------------------------
if __name__=="__main__":
    spheres_data = [
    [1.0, [0, 0, 1], [0.8, 0.1, 0.1, 1], 0.2]
    ]
    boxes_data = [
        [1.0, [0,0,0.5], [0,0,0], [0.5,0.5,0.8,1], [0.1,0.1,0.1]]
    ]
    cam_target_pos = [0,0,0]

    simulation(spheres_data, boxes_data, cam_target_pos)

    cam_distance = 10
    cam_yaw = 50
    cam_pitch = -35
    cam_type = 1

    # Start terminal input thread
    threading.Thread(target=terminal_menu, daemon=True).start()

    # Main loop
    while p.isConnected():
        cam_target_pos, cam_yaw, cam_pitch, cam_distance, cam_type = handle_camera_controls(
            cam_target_pos, cam_yaw, cam_pitch, cam_distance, cam_type)

        p.stepSimulation()
        time.sleep(1./240.)