# AKA CP3 Final with custom physics properties

# pip install pybullet
# pip install pybullet_data

import pybullet as p # type: ignore
import time
import pybullet_data # type: ignore

class Object:
    def __init__(self, mass=0.0, start_pos = [0,0,0], start_orientation = [0, 0, 0], color = [0,0,0]):
        self.mass = mass
        self.start_pos = start_pos
        # Only convert orientation if it's provided as Euler angles (list/tuple)
        if isinstance(start_orientation, (list, tuple)):
            self.start_orientation = p.getQuaternionFromEuler(start_orientation)
        else:
            self.start_orientation = start_orientation # Assume it's already a quaternion
        self.color = color # RGB color with range 0-1
        # self.body_id is set in the child class __init__ after the connection
        self.body_id = None 

    def create(self):
        pass

    def change_dynamics(self, lateral_friction, restitution, linear_damping, angular_damping):
        p.changeDynamics(
            self.body_id, #defines which object to change
            -1, # Target link index (-1 for base)
            lateralFriction=lateral_friction, # How much objects resist sliding against surfaces (higher = more grip)
            restitution=restitution, # Bounciness; 1.0 = perfect bounce, 0 = no bounce
            linearDamping=linear_damping, # Resistance to linear motion; slows down movement over time
            angularDamping=angular_damping # Resistance to rotation; slows down spin over time
        )

class Box(Object):
    def __init__(self, mass=0.0, start_pos = [0,0,0], start_orientation = [0, 0, 0], color = [1,1,1], half_extents = [0.0,0.0,0.0]):
        super().__init__(mass, start_pos, start_orientation, color)
        self.half_extents = half_extents

    def create(self):
        col_shape_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=self.half_extents)
        vis_shape_id = p.createVisualShape(p.GEOM_BOX, halfExtents=self.half_extents, rgbaColor=self.color)
        self.body_id = p.createMultiBody(self.mass, col_shape_id, vis_shape_id, self.start_pos, self.start_orientation)
        return self.body_id

class Sphere(Object):
    def __init__(self, mass=0.0, start_pos = [0,0,0], start_orientation = [0, 0, 0], color = [0,0,0], radius = 0.0):
        super().__init__(mass, start_pos, start_orientation, color)
        self.radius = radius

    def create(self):
        col_shape_id = p.createCollisionShape(p.GEOM_SPHERE, radius=self.radius)
        vis_shape_id = p.createVisualShape(p.GEOM_SPHERE, radius=self.radius, rgbaColor=self.color)
        self.body_id = p.createMultiBody(self.mass, col_shape_id, vis_shape_id, self.start_pos, self.start_orientation)
        return self.body_id

def simulation(spheres_data_list, boxes_data_list, camera_pos):
    # --- Initialize PyBullet physics simulation ---
    p.connect(p.GUI)  # This MUST happen BEFORE creating any objects

    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    p.setGravity(0, 0, -9.81)
    # --- Create ground plane ---
    plane_id = p.loadURDF("plane.urdf")
    # --- Create object instances using list unpacking after connection ---
    spheres = [Sphere(*data) for data in spheres_data_list]
    boxes = [Box(*data) for data in boxes_data_list]
    
    # Call create() manually for each object now that server is connected
    for box in boxes:
        box.create()
    for sphere in spheres:
        sphere.create()
        
    # --- Set initial camera position ---
    p.resetDebugVisualizerCamera(10, 50, -35, camera_pos)
    return spheres, boxes # Return the objects themselves to use in main loop

# --- CORRECTED FUNCTION ---
def handle_camera_controls(camera_target_pos, camera_speed=0.2):
    """
    Checks for keyboard input and updates the camera target position.
    Returns the updated camera_target_pos.
    """
    if not p.isConnected():
        return camera_target_pos

    # Get all camera info at once.
    camera_info = p.getDebugVisualizerCamera()
    
    # The camera info tuple directly provides the forward and right vectors
    # camera_info[3] is the forward vector
    # camera_info[4] is the right vector
    forward = camera_info[3]
    right = camera_info[4]

    keys = p.getKeyboardEvents()

    # W: Move forward (case-insensitive)
    if (ord('w') in keys and keys[ord('w')] & p.B3G_KEY_IS_DOWN) or \
       (ord('W') in keys and keys[ord('W')] & p.B3G_KEY_IS_DOWN):
        camera_target_pos[0] += forward[0] * camera_speed
        camera_target_pos[1] += forward[1] * camera_speed
        camera_target_pos[2] += forward[2] * camera_speed
    
    # S: Move backward (case-insensitive)
    if (ord('s') in keys and keys[ord('s')] & p.B3G_KEY_IS_DOWN) or \
       (ord('S') in keys and keys[ord('S')] & p.B3G_KEY_IS_DOWN):
        camera_target_pos[0] -= forward[0] * camera_speed
        camera_target_pos[1] -= forward[1] * camera_speed
        camera_target_pos[2] -= forward[2] * camera_speed
    
    # A: Move left (strafe) (case-insensitive)
    if (ord('a') in keys and keys[ord('a')] & p.B3G_KEY_IS_DOWN) or \
       (ord('A') in keys and keys[ord('A')] & p.B3G_KEY_IS_DOWN):
        camera_target_pos[0] -= right[0] * camera_speed
        camera_target_pos[1] -= right[1] * camera_speed
        camera_target_pos[2] -= right[2] * camera_speed

    # D: Move right (strafe) (case-insensitive)
    if (ord('d') in keys and keys[ord('d')] & p.B3G_KEY_IS_DOWN) or \
       (ord('D') in keys and keys[ord('D')] & p.B3G_KEY_IS_DOWN):
        camera_target_pos[0] += right[0] * camera_speed
        camera_target_pos[1] += right[1] * camera_speed
        camera_target_pos[2] += right[2] * camera_speed
    
    # --- FIX IS HERE ---
    # Use the correct indices for your PyBullet version.
    cam_dist = camera_info[7]  # Distance
    cam_yaw = camera_info[8]   # Yaw
    cam_pitch = camera_info[9]  # Pitch

    p.resetDebugVisualizerCamera(cam_dist, cam_yaw, cam_pitch, camera_target_pos)

    return camera_target_pos

if __name__ == "__main__":
    # Define initial camera target position (where the camera is looking)
    initial_cam_target = [0, 0, 0]
    
    try:
        # Define data using lists (order matters!)
        sphere_params = [[1.0, [1, 0, 7], [0, 0, 0], [0, 0, 1], 0.5], [0.5, [2, 0, 10], [0, 0, 0], [0, 1, 0], 1.0]]
        box_params = [[1.0, [1, 0, 5], [15, 0, 0], [1, 0, 0], [0.5, 0.5, 0.5]], [0.5, [2, 0, 20], [15, 0, 0], [0, 1, 0], [1,0.2,2]]]

        # Pass the lists of data to the simulation function, using the initial target pos
        spheres, boxes = simulation(sphere_params, box_params, initial_cam_target)
        
        # Initialize the mutable camera position variable for use in the loop
        cam_pos_target = initial_cam_target[:] 

        while p.isConnected():
            p.stepSimulation()
            # Handle camera movement input in every step
            cam_pos_target = handle_camera_controls(cam_pos_target)

            time.sleep(1./240.)

    except KeyboardInterrupt:
        pass

    finally:
        # Ensure disconnection happens even if loop is broken manually
        p.disconnect()