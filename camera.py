# camera movement

import pybullet as p #type: ignore
import pybullet_data #type: ignore
import time
import math
import threading

# from shapes import *
# from simulation import *
# from properties import *
# from bodies import *
# from files import *

def camera_controls(camera_target_pos, camera_yaw, camera_pitch, camera_distance, cam_type):
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
            if camera_distance > 0.5:
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