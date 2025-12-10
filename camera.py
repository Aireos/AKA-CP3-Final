"""
camera.py - camera_controls for different camera modes.
Mode keys (single-trigger):
j = orbit, k = pan, n = elevate, m = zoom
Arrow keys move/adjust camera depending on mode
"""

import math
import pybullet as p  # type: ignore

_last_key_trigger = set()

def _was_triggered(keys, key) -> bool:
    """Return True only when the key was triggered this frame."""
    state = keys.get(key, 0)
    return bool(state & p.KEY_WAS_TRIGGERED)

def camera_controls(camera_target, yaw, pitch, distance, cam_type):
    """Read keyboard and update camera parameters."""
    keys = p.getKeyboardEvents()
    move_speed = 0.25
    yaw_rad = math.radians(yaw)

    # --- Orbit Mode ---
    if cam_type == 1:
        if keys.get(p.B3G_RIGHT_ARROW) == p.KEY_IS_DOWN:
            yaw += 2.0
        if keys.get(p.B3G_LEFT_ARROW) == p.KEY_IS_DOWN:
            yaw -= 2.0
        if keys.get(p.B3G_DOWN_ARROW) == p.KEY_IS_DOWN and pitch < 88:
            pitch += 2.0
        if keys.get(p.B3G_UP_ARROW) == p.KEY_IS_DOWN and pitch > -88:
            pitch -= 2.0

    # --- Pan Mode ---
    elif cam_type == 2:
        forward_x = math.cos(yaw_rad + math.pi / 2) * move_speed
        forward_y = math.sin(yaw_rad + math.pi / 2) * move_speed
        right_x = math.cos(yaw_rad) * move_speed
        right_y = math.sin(yaw_rad) * move_speed
        if keys.get(p.B3G_UP_ARROW) == p.KEY_IS_DOWN:
            camera_target[0] += forward_x
            camera_target[1] += forward_y
        if keys.get(p.B3G_DOWN_ARROW) == p.KEY_IS_DOWN:
            camera_target[0] -= forward_x
            camera_target[1] -= forward_y
        if keys.get(p.B3G_RIGHT_ARROW) == p.KEY_IS_DOWN:
            camera_target[0] += right_x
            camera_target[1] += right_y
        if keys.get(p.B3G_LEFT_ARROW) == p.KEY_IS_DOWN:
            camera_target[0] -= right_x
            camera_target[1] -= right_y

    # --- Elevate Mode ---
    elif cam_type == 3:
        if keys.get(p.B3G_UP_ARROW) == p.KEY_IS_DOWN:
            camera_target[2] += move_speed
        if keys.get(p.B3G_DOWN_ARROW) == p.KEY_IS_DOWN:
            camera_target[2] -= move_speed

    # --- Zoom Mode ---
    elif cam_type == 4:
        # Compute distance from origin
        dist_from_origin = math.sqrt(
            camera_target[0] ** 2 + camera_target[1] ** 2 + camera_target[2] ** 2
        )

        # --- Zoom speed tuning ---
        base_zoom = 0.35           # base speed of zoom
        height_factor = 0.15       # how much height affects zoom speed
        distance_factor = 0.08     # how much distance affects zoom speed

        zoom_speed = base_zoom * (1 + camera_target[2] * height_factor) * (1 + dist_from_origin * distance_factor)
        zoom_speed = max(0.05, zoom_speed)  # never too slow

        # --- Minimum zoom tuning ---
        BASE_MIN_ZOOM = 0.25       # base min zoom when close
        min_shrink_factor = 0.1   # min zoom shrinks as distance grows
        ABSOLUTE_MIN = 0.005        # hard limit

        min_zoom = BASE_MIN_ZOOM / (1 + dist_from_origin * min_shrink_factor)
        min_zoom = max(min_zoom, ABSOLUTE_MIN)

        # --- Apply zoom ---
        if keys.get(p.B3G_UP_ARROW) == p.KEY_IS_DOWN:
            distance -= zoom_speed
        if keys.get(p.B3G_DOWN_ARROW) == p.KEY_IS_DOWN:
            distance += zoom_speed

        # Clamp to dynamic minimum
        if distance < min_zoom:
            distance = min_zoom

    # --- Mode switching ---
    if _was_triggered(keys, ord("j")):
        cam_type = 1
    if _was_triggered(keys, ord("k")):
        cam_type = 2
    if _was_triggered(keys, ord("n")):
        cam_type = 3
    if _was_triggered(keys, ord("m")):
        cam_type = 4

    # Update camera
    p.resetDebugVisualizerCamera(distance, yaw, pitch, camera_target)
    return camera_target, yaw, pitch, distance, cam_type