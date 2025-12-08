# camera.py
import pybullet as p
import math

# Track last key state to prevent rapid mode switching
last_key_state = {}

def camera_controls(camera_target_pos, camera_yaw, camera_pitch, camera_distance, cam_type):
    global last_key_state
    keys = p.getKeyboardEvents()
    move_speed = 0.5

    # Prevent rapid key toggling
    def key_pressed(key):
        return keys.get(key, 0) & p.KEY_WAS_TRIGGERED

    yaw_rad = math.radians(camera_yaw)

    if cam_type == 1:
        # Orbit mode
        if keys.get(p.B3G_RIGHT_ARROW) == p.KEY_IS_DOWN:
            camera_yaw += 2
        if keys.get(p.B3G_LEFT_ARROW) == p.KEY_IS_DOWN:
            camera_yaw -= 2
        if keys.get(p.B3G_DOWN_ARROW) == p.KEY_IS_DOWN and camera_pitch < 88:
            camera_pitch += 2
        if keys.get(p.B3G_UP_ARROW) == p.KEY_IS_DOWN and camera_pitch > -88:
            camera_pitch -= 2

    elif cam_type == 2:
        # Pan mode (X-Y plane)
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
        # Elevate mode
        if keys.get(p.B3G_UP_ARROW) == p.KEY_IS_DOWN:
            camera_target_pos[2] += move_speed
        if keys.get(p.B3G_DOWN_ARROW) == p.KEY_IS_DOWN:
            camera_target_pos[2] -= move_speed

    elif cam_type == 4:
        # Zoom mode
        if keys.get(p.B3G_UP_ARROW) == p.KEY_IS_DOWN and camera_distance > 0.5:
            camera_distance -= 0.5
        if keys.get(p.B3G_DOWN_ARROW) == p.KEY_IS_DOWN:
            camera_distance += 0.5

    # Mode switching (trigger once per key press)
    if key_pressed(ord('j')): cam_type = 1
    if key_pressed(ord('k')): cam_type = 2
    if key_pressed(ord('n')): cam_type = 3
    if key_pressed(ord('m')): cam_type = 4

    p.resetDebugVisualizerCamera(camera_distance, camera_yaw, camera_pitch, camera_target_pos)
    return camera_target_pos, camera_yaw, camera_pitch, camera_distance, cam_type
