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
        self.start_orientation = p.getQuaternionFromEuler(start_orientation)
        self.color = color # RGB color with range 0-1
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
        self.body_id = self.create_box()

    def create(self):
        col_shape_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=self.half_extents)
        vis_shape_id = p.createVisualShape(p.GEOM_BOX, halfExtents=self.half_extents, rgbaColor=self.color)
        body_id = p.createMultiBody(self.mass, col_shape_id, vis_shape_id, self.start_pos, self.start_orientation)
        return body_id
    
    def change_dynamics(self, lateral_friction, restitution, linear_damping, angular_damping):
        p.changeDynamics(
            self.body_id, -1,
            lateralFriction=lateral_friction,
            restitution=restitution,
            linearDamping=linear_damping,
            angularDamping=angular_damping
        )

class Sphere(Object):
    def __init__(self, mass=0.0, start_pos = [0,0,0], start_orientation = [0, 0, 0], color = [0,0,0], radius = 0.0):
        super().__init__(mass, start_pos, start_orientation, color)
        self.radius = radius
        self.body_id = self.create_sphere()

    def create(self):
        col_shape_id = p.createCollisionShape(p.GEOM_SPHERE, radius=self.radius)
        vis_shape_id = p.createVisualShape(p.GEOM_SPHERE, radius=self.radius, rgbaColor=self.color)
        body_id = p.createMultiBody(self.mass, col_shape_id, vis_shape_id, self.start_pos)
        return body_id
    
    def change_dynamics(self, lateral_friction, restitution, linear_damping, angular_damping):
        p.changeDynamics(
            self.body_id, -1,
            lateralFriction=lateral_friction,
            restitution=restitution,
            linearDamping=linear_damping,
            angularDamping=angular_damping
        )

def simulation(spheres,boxes,camera_pos):
    # --- Initialize PyBullet physics simulation ---
    p.connect(p.GUI)  # Use p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # To load plane.urdf
    p.setGravity(0, 0, -9.81)  # Set gravity

    # --- Create ground plane ---
    plane_id = p.loadURDF("plane.urdf")

    for box in boxes:
        box.create()
    for sphere in spheres:
        sphere.create()
    # --- Set initial camera position ---
    p.resetDebugVisualizerCamera(10, 50, -35, camera_pos)
    return p

if __name__ == "__main__":
    try:
        while p.isConnected():
            p.stepSimulation()  # Advance physics simulation
            time.sleep(1./240.) # Run at real-time speed
    except KeyboardInterrupt:
        pass

    p.disconnect()