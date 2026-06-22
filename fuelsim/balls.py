import math
import matplotlib.pyplot as plt
import numpy as np

ball_mass = 0.22  # kg
ball_diameter = 0.15  # m
ball_radius = ball_diameter / 2
ball_frontal_area = math.pi * ball_radius**2

g = 9.80665  # m/s/s
sphere_cd = 0.47
air_density = 1.225  # kg/m3

GOAL_HEIGHT = 72 * 0.0254
GOAL_RADIUS = 20 * 0.0254  # the actual size of the goal for drawing
RADIAL_TOLERANCE = 0.5 * 0.0254  # tolerance used to filter acceptable shots
RIM_CLEARANCE = ball_diameter * 2

def step_motion(x, y, vx, vy, omega, dt=0.01):
    # assume ball is launched into +X +Y direction, +omega is counterclockwise
    Fg = ball_mass * -g

    v = math.hypot(vx, vy)

    Fdrag_x = Fdrag_y = 0
    Fmagnus_x = Fmagnus_y = 0

    if v > 1e-6:
        Fdrag_mag = 0.5 * air_density * sphere_cd * ball_frontal_area * v**2
        Fdrag_x = -Fdrag_mag * vx / v
        Fdrag_y = -Fdrag_mag * vy / v

        if abs(omega) > 1e-6:
            S = abs(omega) * ball_radius / v
            Cl = min(0.5, 0.2 * S)

            Fmagnus_mag = 0.5 * air_density * ball_frontal_area * Cl * v**2

            spin_sign = 1 if omega > 0 else -1

            Fmagnus_x = spin_sign * Fmagnus_mag * (-vy / v)
            Fmagnus_y = spin_sign * Fmagnus_mag * (vx / v)

    omega *= math.exp(-0.1 * dt)

    # sum forces
    Fnet_x = Fdrag_x + Fmagnus_x
    Fnet_y = Fdrag_y + Fmagnus_y + Fg

    # semi-implicit Euler
    vx += Fnet_x / ball_mass * dt
    vy += Fnet_y / ball_mass * dt

    x += vx * dt
    y += vy * dt

    return x, y, vx, vy, omega


main_wheel_diameter = 4 * 0.0254  # meters
hood_wheel_diameter = 1 * 0.0254  # meters

main_wheel_radius = main_wheel_diameter / 2  # meters
hood_wheel_radius = hood_wheel_diameter / 2  # meters

gear_ratio = 24 / 22 * 24 / 21  # hood roller spins 1.25 times faster than main roller
main_slip_factor = 1.0
hood_slip_factor = 1.0


def exit_from_flywheel(RPM):
    """
    calculate exit linear and angular velocity from RPM of main wheel
    """
    # wheel angular speeds
    omega_main_wheel = (2 * math.pi * RPM) / 60
    omega_hood_wheel = (2 * math.pi * RPM * gear_ratio) / 60

    # surface speeds
    v_main = omega_main_wheel * main_wheel_radius * main_slip_factor
    v_hood = omega_hood_wheel * hood_wheel_radius * hood_slip_factor

    # exit linear velocity
    # v = (v_main + v_hood) / 2
    v = 0.002746 * RPM + 1.870  # just gave up and fit to test data lol

    # exit spin
    omega = (v_main - v_hood) / (2 * ball_radius)
    return v, omega


def simulate_arc(distance, v, omega, angle):
    """
    angle in degrees between horizontal and exit vector
    """
    x = -distance
    y = 0.25  # top of flywheel to the ground
    vx = v * math.cos(math.radians(angle))
    vy = v * math.sin(math.radians(angle))

    points_x = [x]
    points_y = [y]

    while True:
        x, y, vx, vy, omega = step_motion(x, y, vx, vy, omega)
        points_x.append(x)
        points_y.append(y)
        
        if (
            vy > 0
            and y < GOAL_HEIGHT + RIM_CLEARANCE
            and x > -GOAL_RADIUS - RIM_CLEARANCE
        ):
            make = False
            break
        if vy < 0 and y < GOAL_HEIGHT:
            if abs(x) < RADIAL_TOLERANCE:
                make = True
            else:
                make = False
            break

    return points_x, points_y, make


def feet_to_meters(feet):
    return feet * 12 * 0.0254


def test_lut_point(dist, rpm, angle):
    """
    converts numbers from lookup table to true values

    gets angle perpendicular to line between flywheels
    """
    return simulate_arc(feet_to_meters(dist), *exit_from_flywheel(rpm), 90 - angle)


test_shots = [
    # range, rpm, angle
    # this angle is defined as the angle between the ground plane and a line through the shooter wheels, not release angle
    # rpm from code, adjusted for the fact we set wrong gear ratio on the motors all season...
    (7, 1842, 21),
    (8, 1950, 21),
    (9, 2058, 21),
    (10, 2167, 23),
    (11, 2167, 26),
    (12, 2167, 29),
    (13, 2221, 31),
    (14, 2275, 32),
    (15, 2329, 33),
    (16, 2383, 33),
    (17, 2492, 34),
    (18, 2600, 35),
    (19, 2708, 37),
]

# given the ranges and angles in the previous table, these exit velocities make the ball hit center of the goal
# maybe_velocities = [
#     6.86,
#     7.20,
#     7.54,
#     7.66,
#     7.71,
#     7.80,
#     7.96,
#     8.17,
#     8.36,
#     8.59,
#     8.78,
#     8.97,
#     9.14,
# ]

# rpms = [rpm for _, rpm, _ in test_shots]
# calc_vels = [exit_from_flywheel(rpm)[0] for _, rpm, _ in test_shots]

# plt.plot(rpms, maybe_velocities)
# plt.plot(rpms, calc_vels)


# distance, RPM, angle = test_shots[12]
# test_v = maybe_velocities[12]
# data = simulate_arc(
#     feet_to_meters(distance + 3), test_v, exit_from_flywheel(RPM)[1], 90 - angle
# )
# plt.plot(data[0], data[1])


# for shot in test_shots:
#     data = test_lut_point(*shot)
#     plt.plot(data[0], data[1])

for dist in np.arange(7, 20, 0.25):
    for v in np.arange(6.5, 9.5, 0.1):
        for angle in np.arange(50, 72, 0.5):
            # print(dist, v, angle)
            x, y, make = simulate_arc(feet_to_meters(dist), v, 50, angle)
            if make:
                # print(f"{v:.2f}")
                plt.plot(x, y)

plt.plot([-GOAL_RADIUS, GOAL_RADIUS], [GOAL_HEIGHT, GOAL_HEIGHT])

plt.axis("equal")
plt.show()
