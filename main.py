# AKA CP3 Final with custom physics properties
# pip install pybullet
# pip install pybullet_data
import pybullet as p
import time
import pybullet_data
import math # Import math library for sin and cos

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
    window_title = "My Custom Simulation Window"
    # Set the window title using the "--window_title" option
    p.connect(p.GUI, options=f"--window_title={window_title}")
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
    return spheres, boxes 

# Global variable to keep track of the text ID so we can replace it
# Initialize to -1 to indicate no text item exists yet
debug_text_id = -1 

def handle_camera_controls(camera_target_pos, camera_yaw, camera_pitch, camera_distance, type):
    global debug_text_id # Access the global variable
    keys = p.getKeyboardEvents()
    
    # Define a consistent movement speed
    move_speed = 0.125
    
    # Convert camera_yaw to radians for math functions
    yaw_rad = math.radians(camera_yaw)

    if type == 1:
        # Camera rotation controls (Yaw and Pitch)
        if keys.get(p.B3G_RIGHT_ARROW) == p.KEY_IS_DOWN:
            camera_yaw += 2.0
        if keys.get(p.B3G_LEFT_ARROW) == p.KEY_IS_DOWN:
            camera_yaw -= 2.0
        if keys.get(p.B3G_DOWN_ARROW) == p.KEY_IS_DOWN:
            if camera_pitch < 88:
                camera_pitch += 2.0
        if keys.get(p.B3G_UP_ARROW) == p.KEY_IS_DOWN:
            if camera_pitch > -88:
                camera_pitch -= 2.0

    elif type == 2:
        # Camera target position controls (X and Y movement relative to camera yaw)
        
        # Calculate forward/backward movement components
        forward_x = math.cos(yaw_rad) * move_speed
        forward_y = math.sin(yaw_rad) * move_speed
        
        # Calculate left/right (strafe) movement components
        # Strafe direction is yaw + 90 degrees (pi/2 radians)
        strafe_x = math.cos(yaw_rad + math.pi/2) * move_speed
        strafe_y = math.sin(yaw_rad + math.pi/2) * move_speed

        if keys.get(p.B3G_RIGHT_ARROW) == p.KEY_IS_DOWN: # Move Forward
            camera_target_pos[0] += forward_x
            camera_target_pos[1] += forward_y
        if keys.get(p.B3G_LEFT_ARROW) == p.KEY_IS_DOWN: # Move Backward
            camera_target_pos[0] -= forward_x
            camera_target_pos[1] -= forward_y
        if keys.get(p.B3G_UP_ARROW) == p.KEY_IS_DOWN: # Strafe Right
            camera_target_pos[0] += strafe_x
            camera_target_pos[1] += strafe_y
        if keys.get(p.B3G_DOWN_ARROW) == p.KEY_IS_DOWN: # Strafe Left
            camera_target_pos[0] -= strafe_x
            camera_target_pos[1] -= strafe_y
        

    elif type == 3:
        # Camera target position controls (Z axis / altitude)
        if keys.get(p.B3G_UP_ARROW) == p.KEY_IS_DOWN:
            camera_target_pos[2] += move_speed
        if keys.get(p.B3G_DOWN_ARROW) == p.KEY_IS_DOWN:
            camera_target_pos[2] -= move_speed

    elif type == 4:
        # Camera distance controls (zoom)
        if keys.get(p.B3G_UP_ARROW) == p.KEY_IS_DOWN:
            camera_distance -= 0.5
        if keys.get(p.B3G_DOWN_ARROW) == p.KEY_IS_DOWN:
            camera_distance += 0.5
    
    # Handle type switching keys outside the type-specific blocks
    if keys.get(ord('j')) == p.KEY_IS_DOWN:
        type = 1
    if keys.get(ord('k')) == p.KEY_IS_DOWN:
        type = 2
    if keys.get(ord('n')) == p.KEY_IS_DOWN:
        type = 3
    if keys.get(ord('m')) == p.KEY_IS_DOWN:
        type = 4
    
    # Determine the text to display based on the current control mode
    if type == 1:
        controls_text = "Mode 1: Camera Rotation (Arrows: Yaw/Pitch)"
    elif type == 2:
        controls_text = "Mode 2: Target Position (Arrows: X/Y relative to view)"
    elif type == 3:
        controls_text = "Mode 3: Altitude (Up/Down Arrows: Z axis)"
    elif type == 4:
        controls_text = "Mode 4: Distance/Zoom (Up/Down Arrows: Zoom)"
    else:
        controls_text = "Unknown Mode"
    
    controls_text += " | Controls: J (Rot), K (Move XY), N (Move Z), M (Zoom)"

    # Add or replace the debug text in the scene
    debug_text_id = p.addUserDebugText(
        text=controls_text,
        textPosition=[-100, 10, 2], # Position the text above the origin
        textColorRGB=[1, 1, 0], # Yellow text for visibility
        lifeTime=0, 
        replaceItemUniqueId=debug_text_id # Replace the existing text item
    )

    # Apply all camera changes
    p.resetDebugVisualizerCamera(
        cameraDistance=camera_distance,
        cameraYaw=camera_yaw,
        cameraPitch=camera_pitch,
        cameraTargetPosition=camera_target_pos
    )
    
    return camera_target_pos, camera_yaw, camera_pitch, camera_distance, type

if __name__ == "__main__":
    # Example simulation data for demonstration
    spheres_data = [
        # mass, pos, orientation (euler), color, radius
        [1.0, [0, 0, 1], [0, 0, 0], [0.8, 0.1, 0.1, 1], 0.2]
    ]
    boxes_data = [
         # mass, pos, orientation (euler), color, half_extents
        [0.0, [0, 0, 0.5], [0, 0, 0], [0.5, 0.5, 0.8, 1], [0.1, 0.1, 0.1]]
    ]
    
    initial_camera_target_pos = [0, 0, 0]

    # Start simulation (this calls p.connect())
    spheres, boxes = simulation(spheres_data, boxes_data, initial_camera_target_pos)

    # Initialize camera variables for the loop
    cam_target_pos = initial_camera_target_pos[:] # Use slice to copy list by value
    cam_distance = 10
    cam_yaw = 50
    cam_pitch = -35
    cam_type = 1 # Start in rotation mode

    # --- Main simulation loop ---
    while p.isConnected():
        p.stepSimulation()
        
        # Handle camera updates
        cam_target_pos, cam_yaw, cam_pitch, cam_distance, cam_type = handle_camera_controls(
            cam_target_pos, cam_yaw, cam_pitch, cam_distance, cam_type
        )
        
        time.sleep(1./240.) # Sleep to simulate a fixed time step
