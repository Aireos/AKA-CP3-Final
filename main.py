# CP3 Final with terminal-based object controls (No sliders, no selection)
# pip install pybullet pybullet_data numpy

import pybullet as p #type: ignore
import pybullet_data #type: ignore
import time
import math
import threading

# Initialize the event for duplication synchronization
duplication_done_event = threading.Event()

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
# Object Propertys
# -------------------------------
# Define default values for properties
# mass
m = 1.0
# lateral friction
lf = 0.5
# rolling friction
rf = 0.1
# spinning friction
sf = 0.1
# linear damping
ld = 0.0
# angular damping
ad = 0.0

def change_properties():
    global m, lf, rf, sf, ld, ad

    bodies = p.getNumBodies()
    if bodies == 0:
        print("No objects to edit.")
        return

    try:
        body_number_input = input("Enter body number to edit: ")
        body_number = int(body_number_input)
    except:
        print("Invalid input.")
        return

    if body_number < 0 or body_number >= bodies:
        print("Body number out of range.")
        return

    try:
        # Validate input and apply changes
        nm = input(f"Mass (current: {m}): ")
        if nm == 'reset':
            p.changeDynamics(body_number, -1, mass=1.0)
            print("Mass reset to 1.0")
        elif nm:
            m = float(nm)
            p.changeDynamics(body_number, -1, mass=m)
            print(f"Mass updated to {m}")

        nlf = input(f"Lateral Friction (current: {lf}): ")
        if nlf == 'reset':
            p.changeDynamics(body_number, -1, lateralFriction=0.5)
            print(f"Lateral Friction reset to {0.5}")
        elif nlf:
            lf = float(nlf)
            p.changeDynamics(body_number, -1, lateralFriction=lf)
            print(f"Lateral Friction updated to {lf}")

        nrf = input(f"Rolling Friction (current: {rf}): ")
        if nrf == 'reset':
            p.changeDynamics(body_number, -1, rollingFriction=0.1)
            print(f"Rolling Friction reset to {0.1}")
        elif nrf:
            rf = float(nrf)
            p.changeDynamics(body_number, -1, rollingFriction=rf)
            print(f"Rolling Friction updated to {rf}")

        nsf = input(f"Spinning Friction (current: {sf}): ")
        if nsf == 'reset':
            p.changeDynamics(body_number, -1, spinningFriction=0.1)
            print(f"Spinning Friction reset to {0.1}")
        elif nsf:
            sf = float(nsf)
            p.changeDynamics(body_number, -1, spinningFriction=sf)
            print(f"Spinning Friction updated to {sf}")

        nld = input(f"Lateral Damping (current: {ld}): ")
        if nld == 'reset':
            p.changeDynamics(body_number, -1, linearDamping=0.0)
            print(f"Linear Damping reset to {0.0}")
        elif nld:
            ld = float(nld)
            p.changeDynamics(body_number, -1, linearDamping=ld)
            print(f"Linear Damping updated to {ld}")

        nad = input(f"Angular Damping (current: {ad}): ")
        if nad == 'reset':
            p.changeDynamics(body_number, -1, angularDamping=0.0)
            print(f"Angular Damping reset to {0.0}")
        elif nad:
            ad = float(nad)
            p.changeDynamics(body_number, -1, angularDamping=ad)
            print(f"Angular Damping updated to {ad}")
    except Exception as e:
        print(f"Error updating properties: {e}")

# -------------------------------
# Bodies
# -------------------------------

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

# -------------------------------
# Body Duplication
# -------------------------------

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

# -------------------------------
# Create New Bodys
# -------------------------------

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
            # Wait until duplication is finished before returning to menu
            duplication_done_event.wait()
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
        try:
            cam_target_pos, cam_yaw, cam_pitch, cam_distance, cam_type = handle_camera_controls(
                cam_target_pos, cam_yaw, cam_pitch, cam_distance, cam_type)
        except:
            pass

        p.stepSimulation()
        time.sleep(1./240.)

    # Graceful disconnect
    if p.isConnected():
        p.disconnect()