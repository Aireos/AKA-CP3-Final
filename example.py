# AKA CP3 Final with custom physics properties

# pip install pybullet
# pip install pybullet_data

import pybullet as p
import time
import pybullet_data

def run_simulation():
    # 1. Connect to the PyBullet physics server in GUI mode (opens visualization window)
    physics_client = p.connect(p.GUI)
    
    # 2. Add the default search path for standard assets like the ground plane
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    
    # 3. Set gravity for the simulation environment (Earth gravity in negative Z direction)
    p.setGravity(0, 0, -10)

    # 4. Load the ground plane URDF (static object that won't move)
    plane_id = p.loadURDF("plane.urdf")

    # --- Create a box (dynamic object) ---
    # Define the size of the box (half extents for each dimension)
    box_half_extents = [0.5, 0.5, 0.5]
    # Create the collision shape for the box
    box_col_shape_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=box_half_extents)
    # Create the visual shape with color
    box_vis_shape_id = p.createVisualShape(p.GEOM_BOX, halfExtents=box_half_extents, rgbaColor=[1, 0, 0, 1])  # Red color
    # Set mass of the box
    box_mass = 1.0
    # Starting position and orientation
    box_start_pos = [1, 0, 5]
    box_start_orientation = p.getQuaternionFromEuler([15, 0, 0])
    # Create the rigid body for the box
    box_id = p.createMultiBody(box_mass, box_col_shape_id, box_vis_shape_id, box_start_pos, box_start_orientation)

    # --- Set physics properties for the box ---
    p.changeDynamics(
        box_id, -1,  # Target object and link index (-1 for base)
        lateralFriction=0.8,   # How much objects resist sliding against surfaces (higher = more grip)
        restitution=0.9,       # Bounciness; 1.0 = perfect bounce, 0 = no bounce
        linearDamping=0.1,     # Resistance to linear motion; slows down movement over time
        angularDamping=0.1     # Resistance to rotation; slows down spin over time
    )

    # --- Create a sphere (dynamic object) ---
    sphere_radius = 0.5
    # Collision shape for sphere
    sphere_col_shape_id = p.createCollisionShape(p.GEOM_SPHERE, radius=sphere_radius)
    # Visual shape with color
    sphere_vis_shape_id = p.createVisualShape(p.GEOM_SPHERE, radius=sphere_radius, rgbaColor=[0, 0, 1, 1])  # Blue
    # Set mass
    sphere_mass = 1.0
    # Starting position
    sphere_start_pos = [1, 0, 7]
    # Create the rigid body for the sphere
    sphere_id = p.createMultiBody(sphere_mass, sphere_col_shape_id, sphere_vis_shape_id, sphere_start_pos)

    # --- Set physics properties for the sphere ---
    p.changeDynamics(
        sphere_id, -1,
        lateralFriction=0.3,   # Less grip, more slippery surface
        restitution=0.4,       # Less bouncy
        linearDamping=0.05,    # Slight resistance to linear movement
        angularDamping=0.05    # Slight resistance to rotation
    )

    # --- Set initial camera position ---
    p.resetDebugVisualizerCamera(10, 50, -35, [0, 0, 0])

    print("Starting simulation loop. Press Ctrl+C to exit.")

    try:
        while p.isConnected():
            p.stepSimulation()  # Advance physics simulation
            time.sleep(1./240.) # Run at real-time speed
    except KeyboardInterrupt:
        pass

    p.disconnect()

if __name__ == "__main__":
    run_simulation()