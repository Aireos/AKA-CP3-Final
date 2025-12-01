# CP3 Final with terminal-based object controls (No sliders, no selection)
# pip install pybullet pybullet_data numpy

import pybullet as p
import pybullet_data
import time
import math
import numpy as np
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
    def __init__(self, mass=0.0, start_pos=[0,0,0], start_orientation=[0,0,0], color=[0,0,0,1], radius=0.1):
        super().__init__(mass, start_pos, start_orientation, color)
        self.radius = radius

    def create(self):
        col_shape_id = p.createCollisionShape(p.GEOM_SPHERE, radius=self.radius)
        vis_shape_id = p.createVisualShape(p.GEOM_SPHERE, radius=self.radius, rgbaColor=self.color)
        self.body_id = p.createMultiBody(self.mass, col_shape_id, vis_shape_id, self.start_pos, self.start_orientation)
        return self.body_id

# -------------------------------
# Simulation Setup
# -------------------------------
def simulation(spheres_data_list, boxes_data_list, camera_pos):
    p.connect(p.GUI, options="--window_title=Custom Simulation")
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0,0,-9.81)
    p.loadURDF("plane.urdf")

    spheres = [Sphere(*data) for data in spheres_data_list]
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
def change_properties():
    bodies = p.getNumBodies()

    if bodies == 0:
        print("No objects to edit.")
        return

    body_number = input("Enter body number to edit: ")
    try:
        body_number = int(body_number)
    except ValueError:
        print("Invalid body number.")
        return

    if body_number < 0 or body_number >= bodies:
        print("Body number out of range.")
        return

    dyn_info = p.getDynamicsInfo(body_number, -1)
    current_mass = dyn_info[0]
    is_static = (current_mass == 0.0)

    print("Enter new properties (leave blank to keep current value, type 'reset' to set to base value):")

    # -------- Mass --------
    base_mass = current_mass  # treat current as "base"
    mass_input = input(f"New mass (current/base: {base_mass}): ")

    if mass_input == 'reset':
        # For safety, don't try to revert to some unknown original; just leave as‑is.
        print(f"Mass left at {base_mass} (no reset behavior for safety).")
    elif mass_input:
        try:
            new_mass = float(mass_input)
            if is_static and new_mass != 0.0:
                print("Warning: Converting a static object (mass=0) to dynamic can destabilize the simulation.")
                print("If you really want a dynamic copy, duplicate the object, then edit the duplicate's mass.")
            else:
                p.changeDynamics(body_number, -1, mass=new_mass)
                print(f"Mass updated to {new_mass}.")
                current_mass = new_mass
                is_static = (new_mass == 0.0)
        except ValueError:
            print("Invalid mass value.")

    # -------- Color --------
    visual = p.getVisualShapeData(body_number)
    if visual:
        current_color = visual[0][7]
    else:
        current_color = [1, 1, 1, 1]

    color_input = input(f"New color (r,g,b,a) (current: {current_color}): ")
    if color_input == 'reset':
        p.changeVisualShape(body_number, -1, rgbaColor=[1, 1, 1, 1])
        print("Color reset to [1, 1, 1, 1].")
    elif color_input:
        try:
            r, g, b, a = map(float, color_input.split(','))
            p.changeVisualShape(body_number, -1, rgbaColor=[r, g, b, a])
            print(f"Color updated to [{r}, {g}, {b}, {a}].")
        except ValueError:
            print("Invalid color value.")

    # -------- Lateral Friction --------
    base_lateral_friction = p.getDynamicsInfo(body_number, -1)[1]
    lateral_friction_input = input(f"New lateral friction (current: {base_lateral_friction}): ")
    if lateral_friction_input == 'reset':
        p.changeDynamics(body_number, -1, lateralFriction=0.5)
        print("Lateral friction reset to 0.5.")
    elif lateral_friction_input:
        try:
            new_friction = float(lateral_friction_input)
            p.changeDynamics(body_number, -1, lateralFriction=new_friction)
            print(f"Lateral friction updated to {new_friction}.")
        except ValueError:
            print("Invalid lateral friction value.")

    # -------- Rolling Friction --------
    base_rolling_friction = p.getDynamicsInfo(body_number, -1)[2]
    rolling_friction_input = input(f"New rolling friction (current: {base_rolling_friction}): ")
    if rolling_friction_input == 'reset':
        p.changeDynamics(body_number, -1, rollingFriction=0.1)
        print("Rolling friction reset to 0.1.")
    elif rolling_friction_input:
        try:
            new_rolling_friction = float(rolling_friction_input)
            p.changeDynamics(body_number, -1, rollingFriction=new_rolling_friction)
            print(f"Rolling friction updated to {new_rolling_friction}.")
        except ValueError:
            print("Invalid rolling friction value.")

    # -------- Spinning Friction --------
    base_spinning_friction = p.getDynamicsInfo(body_number, -1)[3]
    spinning_friction_input = input(f"New spinning friction (current: {base_spinning_friction}): ")
    if spinning_friction_input == 'reset':
        p.changeDynamics(body_number, -1, spinningFriction=0.1)
        print("Spinning friction reset to 0.1.")
    elif spinning_friction_input:
        try:
            new_spinning_friction = float(spinning_friction_input)
            p.changeDynamics(body_number, -1, spinningFriction=new_spinning_friction)
            print(f"Spinning friction updated to {new_spinning_friction}.")
        except ValueError:
            print("Invalid spinning friction value.")

    # -------- Linear Damping --------
    base_linear_damping = p.getDynamicsInfo(body_number, -1)[4]
    linear_damping_input = input(f"New linear damping (current: {base_linear_damping}): ")
    if linear_damping_input == 'reset':
        p.changeDynamics(body_number, -1, linearDamping=0.0)
        print("Linear damping reset to 0.0.")
    elif linear_damping_input:
        try:
            new_linear_damping = float(linear_damping_input)
            p.changeDynamics(body_number, -1, linearDamping=new_linear_damping)
            print(f"Linear damping updated to {new_linear_damping}.")
        except ValueError:
            print("Invalid linear damping value.")

    # -------- Angular Damping --------
    base_angular_damping = p.getDynamicsInfo(body_number, -1)[5]
    angular_damping_input = input(f"New angular damping (current: {base_angular_damping}): ")
    if angular_damping_input == 'reset':
        p.changeDynamics(body_number, -1, angularDamping=0.0)
        print("Angular damping reset to 0.0.")
    elif angular_damping_input:
        try:
            new_angular_damping = float(angular_damping_input)
            p.changeDynamics(body_number, -1, angularDamping=new_angular_damping)
            print(f"Angular damping updated to {new_angular_damping}.")
        except ValueError:
            print("Invalid angular damping value.")

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
        print(f"Body #:{i} Position: {pos} Orientation: {p.getEulerFromQuaternion(orn)} Mass: {mass} Color: {color} Size: {p.getCollisionShapeData(i, -1)[0][3]}")

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
        half_extents = list(dimensions)
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
            start_orientation=euler_orn,
            color=color,
            radius=radius
        )
    else:
        print(f"Unsupported shape type ({shape_type}) for duplication.")
        return

    new_body_id = new_obj.create()
    print(f"Duplicated object {body_number} as new object (id: {new_body_id}, mass={mass}).")

def create_new_box():
    box = Box(1,[0,0,1],[0,0,0],[1,1,1,1],[0.2,0.2,0.2])
    box.create()
    print("Created new box.")

def create_new_sphere():
    sphere = Sphere(1,[0,0,1],[0,0,0],[1,1,1,1],0.2)
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
        [1.0, [0,0,1], [0,0,0], [0.8,0.1,0.1,1], 0.2]
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