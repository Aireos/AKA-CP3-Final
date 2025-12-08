# properties.py
import pybullet as p

def change_properties():
    bodies = p.getNumBodies()
    if bodies <= 1:
        print("No objects to edit.")
        return

    try:
        body_number_input = input("Enter body number to edit: ")
        body_number = int(body_number_input)
    except:
        print("Invalid input.")
        return

    if body_number < 1 or body_number >= bodies:
        print("Body number out of range.")
        return

    # Read current properties dynamically
    dyn_info = p.getDynamicsInfo(body_number, -1)
    mass = dyn_info[0]
    lateral_friction = dyn_info[1]
    rolling_friction = dyn_info[2]
    spinning_friction = dyn_info[3]
    linear_damping = dyn_info[6]
    angular_damping = dyn_info[7]

    try:
        # Mass
        nm = input(f"Mass (current: {mass}): ")
        if nm.lower() == 'reset':
            p.changeDynamics(body_number, -1, mass=1.0)
            print("Mass reset to 1.0")
        elif nm:
            mass = float(nm)
            p.changeDynamics(body_number, -1, mass=mass)
            print(f"Mass updated to {mass}")

        # Lateral friction
        nlf = input(f"Lateral Friction (current: {lateral_friction}): ")
        if nlf.lower() == 'reset':
            p.changeDynamics(body_number, -1, lateralFriction=0.5)
            print("Lateral Friction reset to 0.5")
        elif nlf:
            lateral_friction = float(nlf)
            p.changeDynamics(body_number, -1, lateralFriction=lateral_friction)
            print(f"Lateral Friction updated to {lateral_friction}")

        # Rolling friction
        nrf = input(f"Rolling Friction (current: {rolling_friction}): ")
        if nrf.lower() == 'reset':
            p.changeDynamics(body_number, -1, rollingFriction=0.1)
            print("Rolling Friction reset to 0.1")
        elif nrf:
            rolling_friction = float(nrf)
            p.changeDynamics(body_number, -1, rollingFriction=rolling_friction)
            print(f"Rolling Friction updated to {rolling_friction}")

        # Spinning friction
        nsf = input(f"Spinning Friction (current: {spinning_friction}): ")
        if nsf.lower() == 'reset':
            p.changeDynamics(body_number, -1, spinningFriction=0.1)
            print("Spinning Friction reset to 0.1")
        elif nsf:
            spinning_friction = float(nsf)
            p.changeDynamics(body_number, -1, spinningFriction=spinning_friction)
            print(f"Spinning Friction updated to {spinning_friction}")

        # Linear damping
        nld = input(f"Linear Damping (current: {linear_damping}): ")
        if nld.lower() == 'reset':
            p.changeDynamics(body_number, -1, linearDamping=0.0)
            print("Linear Damping reset to 0.0")
        elif nld:
            linear_damping = float(nld)
            p.changeDynamics(body_number, -1, linearDamping=linear_damping)
            print(f"Linear Damping updated to {linear_damping}")

        # Angular damping
        nad = input(f"Angular Damping (current: {angular_damping}): ")
        if nad.lower() == 'reset':
            p.changeDynamics(body_number, -1, angularDamping=0.0)
            print("Angular Damping reset to 0.0")
        elif nad:
            angular_damping = float(nad)
            p.changeDynamics(body_number, -1, angularDamping=angular_damping)
            print(f"Angular Damping updated to {angular_damping}")

    except Exception as e:
        print(f"Error updating properties: {e}")
