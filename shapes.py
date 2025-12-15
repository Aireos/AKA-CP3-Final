#AKA CP3 Final Project

from typing import List
import math
import pybullet as p  # type: ignore

def _safe_float_input(prompt: str, default: float, min_val=None, max_val=None) -> float:
    val = input(prompt).strip()
    if val == "":
        return default

    try:
        num = float(val)
    except ValueError:
        print(f"Invalid number. Using default {default}.")
        return default

    if min_val is not None and num < min_val:
        print(f"Value too small. Set to {min_val}.")
        return min_val

    if max_val is not None and num > max_val:
        print(f"Value too large. Set to {max_val}.")
        return max_val

    return num


def _read_color(default=None):
    if default is None:
        default = [0.2, 0.2, 0.2, 1.0]

    color_list = input("Color r,g,b,a (default 0.2,0.2,0.2,1) (each 0–1): ").strip()

    if color_list == "":
        return default

    parts = color_list.split(",")
    if len(parts) != 4:
        print("Invalid color format. Using default.")
        return default

    try:
        rgba = [float(p) for p in parts]
    except ValueError:
        print("Invalid color values. Using default.")
        return default

    rgba = [max(0.0, min(1.0, c)) for c in rgba]
    return rgba


class Object:

    def __init__(
        self,
        mass: float = 1.0,
        start_pos: List[float] = None,
        start_orientation: List[float] = None,
        color: List[float] = None,
    ) -> None:

        self.mass = float(mass)
        self.start_pos = start_pos or [0.0, 0.0, 0.0]

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

class Box(Object):

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

class Sphere(Object):

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

def create_new_box() -> None:

    # Positions
    x = _safe_float_input("Enter x position (default 0) (-100 to 100): ", 0.0, -100, 100)
    y = _safe_float_input("Enter y position (default 0) (-100 to 100): ", 0.0, -100, 100)
    z = _safe_float_input("Enter z position (default 1) (0 to 100): ", 1.0, 0, 100)

    # Orientation (Euler degrees)
    roll = _safe_float_input("Roll degrees (default 0) (-180 to 180): ", 0.0, -180, 180)
    pitch = _safe_float_input("Pitch degrees (default 0) (-89 to 89): ", 0.0, -89, 89)
    yaw = _safe_float_input("Yaw degrees (default 0) (-360 to 360): ", 0.0, -360, 360)

    # Size
    length = _safe_float_input("Length (default 0.2) (0.001 to 20): ", 0.2, 0.001, 20)
    width = _safe_float_input("Width (default 0.2) (0.001 to 20): ", 0.2, 0.001, 20)
    height = _safe_float_input("Height (default 0.2) (0.001 to 20): ", 0.2, 0.001, 20)

    mass = _safe_float_input("Mass (default 1) (0.001 to 100): ", 1.0, 0.001, 100)

    color = _read_color()

    half_extents = [length / 2, width / 2, height / 2]

    box = Box(
        mass=mass,
        start_pos=[x, y, z + half_extents[2]],
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

    radius = _safe_float_input("Radius (default 0.2) (0.001 to 20): ", 0.2, 0.001, 20)
    mass = _safe_float_input("Mass (default 1) (0.001 to 100): ", 1.0, 0.001, 100)

    x = _safe_float_input("Enter x position (default 0) (-100 to 100): ", 0.0, -100, 100)
    y = _safe_float_input("Enter y position (default 0) (-100 to 100): ", 0.0, -100, 100)
    z = _safe_float_input("Enter z position (default 1) (0 to 100): ", 1.0, 0, 100)

    color = _read_color()

    sphere = Sphere(
        mass=mass,
        start_pos=[x, y, z+ radius],
        color=color,
        radius=radius,
    )

    sphere.create()
    print("New sphere created.")