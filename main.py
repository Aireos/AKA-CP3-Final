# CP3 Final with custom physics properties (Right-click editor + Camera)
# pip install pybullet pybullet_data numpy

import pybullet as p
import pybullet_data
import time
import math
import numpy as np

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

    def change_dynamics(self, lateral_friction, restitution, linear_damping, angular_damping):
        p.changeDynamics(self.body_id, -1, lateralFriction=lateral_friction,
                         restitution=restitution, linearDamping=linear_damping,
                         angularDamping=angular_damping)

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
debug_text_id = -1
def handle_camera_controls(camera_target_pos, camera_yaw, camera_pitch, camera_distance, cam_type):
    global debug_text_id
    keys = p.getKeyboardEvents()
    move_speed = 0.5
    yaw_rad = math.radians(camera_yaw)

    if cam_type==1:
        if keys.get(p.B3G_RIGHT_ARROW)==p.KEY_IS_DOWN: camera_yaw+=2
        if keys.get(p.B3G_LEFT_ARROW)==p.KEY_IS_DOWN: camera_yaw-=2
        if keys.get(p.B3G_DOWN_ARROW)==p.KEY_IS_DOWN and camera_pitch<88: camera_pitch+=2
        if keys.get(p.B3G_UP_ARROW)==p.KEY_IS_DOWN and camera_pitch>-88: camera_pitch-=2
    elif cam_type==2:
        forward_x = math.cos(yaw_rad)*move_speed
        forward_y = math.sin(yaw_rad)*move_speed
        strafe_x = math.cos(yaw_rad+math.pi/2)*move_speed
        strafe_y = math.sin(yaw_rad+math.pi/2)*move_speed
        if keys.get(p.B3G_RIGHT_ARROW)==p.KEY_IS_DOWN: camera_target_pos[0]+=forward_x; camera_target_pos[1]+=forward_y
        if keys.get(p.B3G_LEFT_ARROW)==p.KEY_IS_DOWN: camera_target_pos[0]-=forward_x; camera_target_pos[1]-=forward_y
        if keys.get(p.B3G_UP_ARROW)==p.KEY_IS_DOWN: camera_target_pos[0]+=strafe_x; camera_target_pos[1]+=strafe_y
        if keys.get(p.B3G_DOWN_ARROW)==p.KEY_IS_DOWN: camera_target_pos[0]-=strafe_x; camera_target_pos[1]-=strafe_y
    elif cam_type==3:
        if keys.get(p.B3G_UP_ARROW)==p.KEY_IS_DOWN: camera_target_pos[2]+=move_speed
        if keys.get(p.B3G_DOWN_ARROW)==p.KEY_IS_DOWN: camera_target_pos[2]-=move_speed
    elif cam_type==4:
        if keys.get(p.B3G_UP_ARROW)==p.KEY_IS_DOWN: camera_distance-=0.5
        if keys.get(p.B3G_DOWN_ARROW)==p.KEY_IS_DOWN: camera_distance+=0.5

    if keys.get(ord('j'))==p.KEY_IS_DOWN: cam_type=1
    if keys.get(ord('k'))==p.KEY_IS_DOWN: cam_type=2
    if keys.get(ord('n'))==p.KEY_IS_DOWN: cam_type=3
    if keys.get(ord('m'))==p.KEY_IS_DOWN: cam_type=4

    if cam_type==1: txt="Camera Mode 1: Rotation (Arrows: Yaw/Pitch)"
    elif cam_type==2: txt="Camera Mode 2: Target XY (Arrows)"
    elif cam_type==3: txt="Camera Mode 3: Altitude (Up/Down)"
    elif cam_type==4: txt="Camera Mode 4: Zoom (Up/Down)"
    else: txt="Unknown Mode"
    txt+=" | J/K/N/M to switch"
    debug_text_id = p.addUserDebugText(txt,[-100,10,2],[1,1,0],0,replaceItemUniqueId=debug_text_id)
    p.resetDebugVisualizerCamera(camera_distance,camera_yaw,camera_pitch,camera_target_pos)
    return camera_target_pos, camera_yaw, camera_pitch, camera_distance, cam_type

# -------------------------------
# Object Selection & Highlight
# -------------------------------
selected_body = None
highlight_body = None
property_sliders = {}
HIGHLIGHT_SCALE=1.05

def clear_highlight():
    global highlight_body
    if highlight_body is not None:
        try: p.removeBody(highlight_body)
        except: pass
        highlight_body=None

def apply_highlight(body_id):
    global highlight_body
    clear_highlight()
    visual = p.getVisualShapeData(body_id)
    if not visual: return
    shape = visual[0]
    geom_type = shape[2]; dims = shape[3]
    pos, orn = p.getBasePositionAndOrientation(body_id)
    if geom_type==p.GEOM_BOX:
        half_extents = [float(d)*HIGHLIGHT_SCALE for d in dims]
        vs = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents, rgbaColor=[1,1,0,0.3])
    elif geom_type==p.GEOM_SPHERE:
        radius = float(dims[0]) * HIGHLIGHT_SCALE
        vs = p.createVisualShape(p.GEOM_SPHERE, radius=radius, rgbaColor=[1,1,0,0.3])
    else: return
    highlight_body = p.createMultiBody(baseMass=0, baseVisualShapeIndex=vs, baseCollisionShapeIndex=-1, basePosition=pos, baseOrientation=orn)

def update_highlight_follow():
    global highlight_body, selected_body
    if selected_body is None or highlight_body is None: return
    pos,orn=p.getBasePositionAndOrientation(selected_body)
    p.resetBasePositionAndOrientation(highlight_body,pos,orn)

def get_ray_from_mouse(mouse_x, mouse_y):
    cam_info = p.getDebugVisualizerCamera()
    if cam_info is None: return [0,0,0],[0,0,0]
    w = cam_info[0]; h = cam_info[1]
    viewMat = np.array(cam_info[2]).reshape(4,4)
    projMat = np.array(cam_info[3]).reshape(4,4)
    camTarget = np.array(cam_info[10])
    camPos = np.array(cam_info[11])
    x_ndc = (mouse_x / float(w) - 0.5) * 2.0
    y_ndc = - (mouse_y / float(h) - 0.5) * 2.0
    ray_clip = np.array([x_ndc, y_ndc, -1.0, 1.0])
    try:
        inv_proj = np.linalg.inv(projMat)
        ray_eye = inv_proj.dot(ray_clip)
        ray_eye = np.array([ray_eye[0], ray_eye[1], -1.0, 0.0])
        inv_view = np.linalg.inv(viewMat)
        ray_world = inv_view.dot(ray_eye)[0:3]
        ray_world = ray_world / np.linalg.norm(ray_world)
    except:
        ray_world = camTarget - camPos
        norm = np.linalg.norm(ray_world)
        ray_world = ray_world / norm if norm>1e-6 else np.array([0,0,1])
    ray_start = camPos.tolist()
    ray_end = (camPos + ray_world*1000).tolist()
    return ray_start, ray_end

def handle_object_selection():
    global selected_body
    events = p.getMouseEvents()
    for e in events:
        # Debug print to confirm events
        # print("Mouse Event:", e)
        print("Mouse Event:", e)
        button = e[3]
        state = e[4]
        mouseX = e[1]
        mouseY = e[2]
        if button == 2 and state in (p.KEY_WAS_TRIGGERED, p.KEY_IS_DOWN):
            ray_start, ray_end = get_ray_from_mouse(mouseX, mouseY)
            hit = p.rayTest(ray_start, ray_end)[0]
            hit_body = hit[0]
            if hit_body >= 0:
                selected_body = hit_body
                apply_highlight(hit_body)
                create_property_sliders(hit_body)
                print("Selected body:", hit_body)

# -------------------------------
# Property Sliders
# -------------------------------
def clear_property_sliders():
    global property_sliders
    for s in property_sliders.values():
        try: p.removeUserDebugItem(s)
        except: pass
    property_sliders = {}

def create_property_sliders(body_id):
    global property_sliders
    clear_property_sliders()
    info = p.getDynamicsInfo(body_id, -1)
    mass = info[0] if len(info)>0 else 1.0
    friction = info[1] if len(info)>1 else 0.5
    restitution = info[5] if len(info)>5 else 0.0
    lin_damping = info[6] if len(info)>6 else 0.0
    ang_damping = info[7] if len(info)>7 else 0.0
    vis = p.getVisualShapeData(body_id)
    color = [1,1,1,1]
    if vis: color = list(vis[0][7])
    property_sliders["friction"] = p.addUserDebugParameter("Friction",0,5,friction)
    property_sliders["restitution"] = p.addUserDebugParameter("Restitution",0,1,restitution)
    property_sliders["lin_damping"] = p.addUserDebugParameter("Linear Damping",0,1,lin_damping)
    property_sliders["ang_damping"] = p.addUserDebugParameter("Angular Damping",0,1,ang_damping)
    property_sliders["R"] = p.addUserDebugParameter("Color R",0,1,color[0])
    property_sliders["G"] = p.addUserDebugParameter("Color G",0,1,color[1])
    property_sliders["B"] = p.addUserDebugParameter("Color B",0,1,color[2])
    property_sliders["A"] = p.addUserDebugParameter("Color A",0,1,color[3])
    property_sliders["duplicate"] = p.addUserDebugParameter("Duplicate Object",0,1,0)
    property_sliders["new_box"] = p.addUserDebugParameter("Create Box",0,1,0)
    property_sliders["new_sphere"] = p.addUserDebugParameter("Create Sphere",0,1,0)
    property_sliders["ok"] = p.addUserDebugParameter("OK",0,1,0)

def update_object_from_sliders():
    global selected_body
    if selected_body is None or not property_sliders: return
    s = property_sliders
    friction = p.readUserDebugParameter(s["friction"])
    restitution = p.readUserDebugParameter(s["restitution"])
    lin_damping = p.readUserDebugParameter(s["lin_damping"])
    ang_damping = p.readUserDebugParameter(s["ang_damping"])
    R = p.readUserDebugParameter(s["R"])
    G = p.readUserDebugParameter(s["G"])
    B = p.readUserDebugParameter(s["B"])
    A = p.readUserDebugParameter(s["A"])
    try:
        p.changeDynamics(selected_body, -1, lateralFriction=friction,
                         restitution=restitution, linearDamping=lin_damping,
                         angularDamping=ang_damping)
        p.changeVisualShape(selected_body, -1, rgbaColor=[R,G,B,A])
    except: pass

def handle_duplication_and_creation():
    global property_sliders, selected_body
    if not property_sliders or selected_body is None: return
    s = property_sliders
    try:
        if p.readUserDebugParameter(s["duplicate"])>0.5:
            pos,orn = p.getBasePositionAndOrientation(selected_body)
            pos = [pos[0]+0.2,pos[1],pos[2]+0.2]
            vis = p.getVisualShapeData(selected_body)
            if not vis: return
            vis0 = vis[0]; shape_type = vis0[2]; dims = vis0[3]; color = list(vis0[7])
            if shape_type==p.GEOM_BOX:
                new = Box(1,pos,[0,0,0],color,list(dims))
            elif shape_type==p.GEOM_SPHERE:
                new = Sphere(1,pos,[0,0,0],color,float(dims[0]))
            else: new=None
            if new: new.create(); print("Duplicated object")
            property_sliders["duplicate"]=p.addUserDebugParameter("Duplicate Object",0,1,0)
    except: pass

    try:
        if p.readUserDebugParameter(s["new_box"])>0.5:
            new = Box(1,[0,0,1],[0,0,0],[1,1,1,1],[0.2,0.2,0.2])
            new.create(); print("Created new box")
            property_sliders["new_box"]=p.addUserDebugParameter("Create Box",0,1,0)
    except: pass

    try:
        if p.readUserDebugParameter(s["new_sphere"])>0.5:
            new = Sphere(1,[0,0,1],[0,0,0],[1,1,1,1],0.2)
            new.create(); print("Created new sphere")
            property_sliders["new_sphere"]=p.addUserDebugParameter("Create Sphere",0,1,0)
    except: pass

    try:
        if p.readUserDebugParameter(s["ok"])>0.5:
            clear_property_sliders()
            clear_highlight()
            selected_body=None
    except: pass

# -------------------------------
# Main
# -------------------------------
if __name__=="__main__":
    spheres_data=[[1.0,[0,0,1],[0,0,0],[0.8,0.1,0.1,1],0.2]]
    boxes_data=[[0.0,[0,0,0.5],[0,0,0],[0.5,0.5,0.8,1],[0.1,0.1,0.1]]]
    cam_target_pos=[0,0,0]

    simulation(spheres_data, boxes_data, cam_target_pos)

    cam_distance=10; cam_yaw=50; cam_pitch=-35; cam_type=1

    while p.isConnected():
        handle_object_selection()
        update_highlight_follow()
        update_object_from_sliders()
        handle_duplication_and_creation()
        cam_target_pos, cam_yaw, cam_pitch, cam_distance, cam_type = handle_camera_controls(
            cam_target_pos, cam_yaw, cam_pitch, cam_distance, cam_type)
        p.stepSimulation()
        time.sleep(1./240.)
