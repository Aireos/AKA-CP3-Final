"""
shapes.py - Defines shape classes (Box, Sphere) and helper functions
to create new shapes from terminal input.
"""
from typing import List, Tuple

import math
import pybullet as p #type: ignore


class SimObject:
    """Base class for objects placed into the PyBullet world."""

    def __init__(
        self,
        mass: float = 0.0,
        start_pos: List[float] = None,
        start_orientation: List[float] = None,
        color: List[float] = None,
    ) -> None:
        self.mass = float(mass)
        self.start_pos = start_pos or [0.0, 0.0, 0.0]
        # Accept Euler angles or quaternion; convert Euler to quaternion
        if start_orientation is None:
            self.start_orientation = p.getQuaternionFromEuler([0.0, 0.0, 0.0])
        elif isinstance(start_orientation, (list, tuple)) and len(start_orientation) == 3:
            self.start_orientation = p.getQuaternionFromEuler(start_orientation)
        else:
            self.start_orientation = start_orientation
        self.color = color or [0.5, 0.5, 0.5, 1.0]
        self.body_id = None

    def create(self) -> int:
        """Create the object in PyBullet and return its body id."""
        raise NotImplementedError


class Box(SimObject):
    """Axis-aligned box defined by half extents."""

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


class Sphere(SimObject):
    """Sphere defined by radius."""

    def __init__(
        self,
        mass: float = 1.0,
        start_pos: List[float] = None,
        color: List[float] = None,
        radius: float = 0.1,
    ) -> None:
        super().__init__(mass, start_pos, [0.0, 0.0, 0.0], color)
        self.radius = float(radius)

    def create(self) -> int:
        col_id = p.createCollisionShape(p.GEOM_SPHERE, radius=self.radius)
        vis_id = p.createVisualShape(p.GEOM_SPHERE, radius=self.radius, rgbaColor=self.color)
        self.body_id = p.createMultiBody(self.mass, col_id, vis_id, self.start_pos, self.start_orientation)
        return self.body_id


# Helper functions used by the terminal menu to create new objects
def _safe_float(prompt: str, default: float) -> float:
    val = input(prompt)
    if val == "":
        return default
    try:
        return float(val)
    except ValueError:
        print("Invalid number; using default:", default)
        return default


def create_new_box() -> None:
    """Interactive creation of a new box (called from terminal menu)."""
    x = _safe_float("Enter x position (default 0): ", 0.0)
    y = _safe_float("Enter y position (default 0): ", 0.0)
    z = _safe_float("Enter z position (default 1): ", 1.0)
    roll = _safe_float("Roll degrees (default 0): ", 0.0)
    pitch = _safe_float("Pitch degrees (default 0): ", 0.0)
    yaw = _safe_float("Yaw degrees (default 0): ", 0.0)
    length = _safe_float("Length (default 0.2): ", 0.2)
    width = _safe_float("Width (default 0.2): ", 0.2)
    height = _safe_float("Height (default 0.2): ", 0.2)
    mass = _safe_float("Mass (default 1): ", 1.0)
    color_str = input("Color r,g,b,a (default 0.2,0.2,0.2,1): ")
    if color_str:
        try:
            color = [float(x) for x in color_str.split(",")]
            if len(color) != 4:
                raise ValueError
        except Exception:
            print("Invalid color; using default.")
            color = [0.2, 0.2, 0.2, 1.0]
    else:
        color = [0.2, 0.2, 0.2, 1.0]

    half_extents = [length / 2.0, width / 2.0, height / 2.0]
    box = Box(mass, [x, y, z], [math.radians(roll), math.radians(pitch), math.radians(yaw)], color, half_extents)
    box.create()
    print("New box created.")


def create_new_sphere() -> None:
    """Interactive creation of a new sphere (called from terminal menu)."""
    radius = _safe_float("Radius (default 0.2): ", 0.2)
    mass = _safe_float("Mass (default 1): ", 1.0)
    x = _safe_float("x (default 0): ", 0.0)
    y = _safe_float("y (default 0): ", 0.0)
    z = _safe_float("z (default 1): ", 1.0)
    color_str = input("Color r,g,b,a (default 0.2,0.2,0.2,1): ")
    if color_str:
        try:
            color = [float(c) for c in color_str.split(",")]
            if len(color) != 4:
                raise ValueError
        except Exception:
            print("Invalid color; using default.")
            color = [0.2, 0.2, 0.2, 1.0]
    else:
        color = [0.2, 0.2, 0.2, 1.0]

    sphere = Sphere(mass, [x, y, z], color, radius)
    sphere.create()
    print("New sphere created.")