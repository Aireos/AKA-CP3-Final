# AKA CP3 Final with custom physics properties
# pip install pybullet
# pip install pybullet_data
import pybullet as p
import time
import pybullet_data

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
            "-1", # Target link index (-1 for base)
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
    p.connect(p.GUI) # This MUST happen BEFORE creating any objects
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81) # FIX: Use float instead of string for gravity

    # --- Create ground plane ---
    plane_id = p.loadURDF("plane.urdf")

    # --- Create object instances using list unpacking after connection ---
    # Data structure adjusted in __main__ to match __init__ signature (mass, pos, orientation, color, dim)
    spheres = [Sphere(*data) for data in spheres_data_list]
    boxes = [Box(*data) for data in boxes_data_list]

    # Call create() manually for each object now that server is connected
    for box in boxes:
        box.create()
    for sphere in spheres:
        sphere.create()

    # --- Set initial camera position ---
    # FIX: Use float instead of string for camera pitch
    p.resetDebugVisualizerCamera(10, 50, -35, camera_pos)
    return spheres, boxes # Return the objects themselves to use in main loop

def handle_camera_controls(camera_target_pos, camera_yaw, camera_pitch, camera_distance):
    keys = p.getKeyboardEvents()
    
    # Camera rotation controls
    # FIX: Use == instead of & to prevent TypeError when key is not pressed
    if keys.get(p.B3G_LEFT_ARROW) == p.KEY_IS_DOWN:
        camera_yaw += 2.0
    if keys.get(p.B3G_RIGHT_ARROW) == p.KEY_IS_DOWN:
        camera_yaw -= 2.0
    if keys.get(p.B3G_UP_ARROW) == p.KEY_IS_DOWN:
        camera_pitch += 2.0
    if keys.get(p.B3G_DOWN_ARROW) == p.KEY_IS_DOWN:
        camera_pitch -= 2.0
    
    # Camera target position controls
    if keys.get(ord('w')) == p.KEY_IS_DOWN:
        camera_target_pos[1] += 0.125
    if keys.get(ord('s')) == p.KEY_IS_DOWN:
        camera_target_pos[1] -= 0.125
    if keys.get(ord('d')) == p.KEY_IS_DOWN:
        camera_target_pos[0] += 0.125
    if keys.get(ord('a')) == p.KEY_IS_DOWN:
        camera_target_pos[0] -= 0.125
    if keys.get(ord('r')) == p.KEY_IS_DOWN:
        camera_target_pos[2] += 0.125
    if keys.get(ord('f')) == p.KEY_IS_DOWN:
        camera_target_pos[2] -= 0.125
    
    # Camera distance controls (zoom)
    if keys.get(ord('q')) == p.KEY_IS_DOWN:
        camera_distance -= 0.5
    if keys.get(ord('e')) == p.KEY_IS_DOWN:
        camera_distance += 0.5
    
    # Apply all camera changes
    p.resetDebugVisualizerCamera(
        cameraDistance=camera_distance,
        cameraYaw=camera_yaw,
        cameraPitch=camera_pitch,
        cameraTargetPosition=camera_target_pos
    )
    
    return camera_target_pos, camera_yaw, camera_pitch, camera_distance

if __name__ == "__main__":
    # Define initial camera target position (where the camera is looking)
    initial_cam_target = [0, 0, 0]
    try:
        # Define data using lists with corrected order to match class __init__ signatures
        # Order: mass, start_pos, start_orientation, color, dimension
        sphere_params = [
            [1.0, [1, 0, 7], [0, 0, 0], [0, 0, 1, 1], 0.5], # Blue sphere
            [0.5, [2, 0, 10], [0, 0, 0], [0, 1, 0, 1], 1.0]  # Green sphere
        ]
        box_params = [
            [1.0, [1, 0, 5], [0, 0, 0], [1, 0, 0, 1], [0.5, 0.5, 0.5]], # Red box
            [0.5, [2, 0, 20], [0, 0, 0], [0, 1, 0, 1], [1, 0.2, 2]]   # Green box
        ]

        # Pass the lists of data to the simulation function, using the initial target pos
        spheres, boxes = simulation(sphere_params, box_params, initial_cam_target)

        # Initialize the mutable camera variables for use in the loop
        cam_pos_target = initial_cam_target[:]
        cam_yaw = 50  # Initial yaw
        cam_pitch = -35  # Initial pitch
        cam_distance = 10  # Initial distance
        
        while p.isConnected():
            p.stepSimulation()
            # Handle camera movement input in every step
            cam_pos_target, cam_yaw, cam_pitch, cam_distance = handle_camera_controls(
                cam_pos_target, cam_yaw, cam_pitch, cam_distance
            )
            time.sleep(1./240.)

    except KeyboardInterrupt:
        pass
    finally:
        # Ensure disconnection happens even if loop is broken manually
        p.disconnect()