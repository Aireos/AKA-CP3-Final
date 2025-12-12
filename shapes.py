"""
shapes.py - Defines shape classes (Box, Sphere) and helper functions
to create new shapes from terminal input.
"""

from typing import List
import math
import pybullet as p  # type: ignore


# ============================================================
# Utility helpers
# ============================================================

def _safe_float(prompt: str, default: float, min_val=None, max_val=None) -> float:
    """Read a float safely and clamp it if needed."""
    val = input(prompt).strip()
    if val == "":
        return default

    try:
        num = float(val)
    except ValueError:
        print(f"Invalid number. Using default {default}.")
        return default

    if min_val is not None and num < min_val:
        print(f"Value too small. Clamped to {min_val}.")
        return min_val

    if max_val is not None and num > max_val:
        print(f"Value too large. Clamped to {max_val}.")
        return max_val

    return num


def _read_color(default=None):
    """Read RGBA color with clamping."""
    if default is None:
        default = [0.2, 0.2, 0.2, 1.0]

    val = input("Color r,g,b,a (default 0.2,0.2,0.2,1) (each 0–1): ").strip()

    if val == "":
        return default

    parts = val.split(",")
    if len(parts) != 4:
        print("Invalid color format. Using default.")
        return default

    try:
        rgba = [float(c) for c in parts]
    except ValueError:
        print("Invalid color values. Using default.")
        return default

    # Clamp each channel
    rgba = [max(0.0, min(1.0, c)) for c in rgba]
    return rgba


# ============================================================
# Base object class
# ============================================================

class SimObject:
    """Base class for objects placed into the PyBullet world."""

    def __init__(
        self,
        mass: float = 1.0,
        start_pos: List[float] = None,
        start_orientation: List[float] = None,
        color: List[float] = None,
    ) -> None:

        self.mass = float(mass)
        self.start_pos = start_pos or [0.0, 0.0, 0.0]

        # Orientation: convert Euler (3 values) → quaternion
        if start_orientation is None:
            self.start_orientation = p.getQuaternionFromEuler([0, 0, 0])
        elif isinstance(start_orientation, (list, tuple)) and len(start_orientation) == 3:
            self.start_orientation = p.getQuaternionFromEuler(start_orientation)
        else:
            self.start_orientation = start_orientation

        self.color = color or [0.5, 0.5, 0.5, 1.0]
        self.body_id = None

    def create(self) -> int:
        raise NotImplementedError

# ============================================================
# Box object
# ============================================================

class Box(SimObject):
    """Axis-aligned box defined by half-extents."""

    def __init__(
        self,
        mass: float = 1.0,
        start_pos: List[float] = None,
        start_orientation: List[float] = None,
        color: List[float] = None,
        half_extents: List[float] = None,
    ) -> None:

        super().__init__(mass, start_pos, start_orientation, color)
        self.half_extents = half_extents or [0.1, 0.1, 0.1]

    def create(self) -> int:
        col_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=self.half_extents)
        vis_id = p.createVisualShape(p.GEOM_BOX, halfExtents=self.half_extents, rgbaColor=self.color)
        self.body_id = p.createMultiBody(
            self.mass, col_id, vis_id, self.start_pos, self.start_orientation
        )
        return self.body_id

# ============================================================
# Sphere object
# ============================================================

class Sphere(SimObject):
    """Sphere object defined by radius."""

    def __init__(
        self,
        mass: float = 1.0,
        start_pos: List[float] = None,
        color: List[float] = None,
        radius: float = 0.1,
    ) -> None:

        super().__init__(mass, start_pos, [0, 0, 0], color)
        self.radius = float(radius)

    def create(self) -> int:
        col_id = p.createCollisionShape(p.GEOM_SPHERE, radius=self.radius)
        vis_id = p.createVisualShape(p.GEOM_SPHERE, radius=self.radius, rgbaColor=self.color)
        self.body_id = p.createMultiBody(
            self.mass, col_id, vis_id, self.start_pos, self.start_orientation
        )
        return self.body_id

# ============================================================
# Creation helpers used by terminal menu
# ============================================================

def create_new_box() -> None:
    """Interactive creation of a new box."""

    # Positions
    x = _safe_float("Enter x position (default 0) (-100 to 100): ", 0.0, -100, 100)
    y = _safe_float("Enter y position (default 0) (-100 to 100): ", 0.0, -100, 100)
    z = _safe_float("Enter z position (default 1) (-100 to 100): ", 1.0, -100, 100)

    # Orientation (Euler degrees)
    roll = _safe_float("Roll degrees (default 0) (-180 to 180): ", 0.0, -180, 180)
    pitch = _safe_float("Pitch degrees (default 0) (-89 to 89): ", 0.0, -89, 89)
    yaw = _safe_float("Yaw degrees (default 0) (-360 to 360): ", 0.0, -360, 360)

    # Size
    length = _safe_float("Length (default 0.2) (0.001 to 20): ", 0.2, 0.001, 20)
    width = _safe_float("Width (default 0.2) (0.001 to 20): ", 0.2, 0.001, 20)
    height = _safe_float("Height (default 0.2) (0.001 to 20): ", 0.2, 0.001, 20)

    mass = _safe_float("Mass (default 1) (0.001 to 100): ", 1.0, 0.001, 100)

    color = _read_color()

    half_extents = [length / 2, width / 2, height / 2]

    box = Box(
        mass=mass,
        start_pos=[x, y, z],
        start_orientation=[
            math.radians(roll),
            math.radians(pitch),
            math.radians(yaw),
        ],
        color=color,
        half_extents=half_extents,
    )

    box.create()
    print("New box created.")


def create_new_sphere() -> None:
    """Interactive creation of a new sphere."""

    radius = _safe_float("Radius (default 0.2) (0.001 to 20): ", 0.2, 0.001, 20)
    mass = _safe_float("Mass (default 1) (0.001 to 100): ", 1.0, 0.001, 100)

    x = _safe_float("Enter x position (default 0) (-100 to 100): ", 0.0, -100, 100)
    y = _safe_float("Enter y position (default 0) (-100 to 100): ", 0.0, -100, 100)
    z = _safe_float("Enter z position (default 1) (-100 to 100): ", 1.0, -100, 100)

    color = _read_color()

    sphere = Sphere(
        mass=mass,
        start_pos=[x, y, z],
        color=color,
        radius=radius,
    )

    sphere.create()
    print("New sphere created.")