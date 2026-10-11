import numpy as np
import matplotlib.pyplot as plt
import matplotlib.transforms as transforms
import copy
import math
import sys
import pathlib
from matplotlib.patches import Rectangle

sys.path.append(str(pathlib.Path(__file__).parent.parent))

from ego_vehicle import parameter
from obs_vehicle import obstacle_planning
from utilis import *
from CubicSplinePlanner import cubic_spline_planner

show_animation = True

def generate_target_course(x, y):
    csp = cubic_spline_planner.CubicSpline2D(x, y)
    s = np.arange(0, csp.s[-1], 0.1)

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = csp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(csp.calc_yaw(i_s))
        rk.append(csp.calc_curvature(i_s))

    return rx, ry, ryaw, rk, csp


def main():
    print(__file__ + " start!!")

    # Define smoother waypoints for a longer gently curved road
    wx = [0.0, 20.0, 40.0, 60.0, 80.0, 100.0, 120.0, 140.0, 160.0, 180.0, 200.0, 220.0, 240.0, 250.0]
    wy = [0.0, 20.0, 20.0, 30.0, 10.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0]

    tx, ty, tyaw, tc, csp = generate_target_course(wx, wy)

    area = 25.0  # Visualization area

    # Create road boundaries using tyaw for visualization
    csp.ux = [x + 7.5 * math.cos(i_yaw + math.pi / 2.0) for x, i_yaw in zip(tx, tyaw)]
    csp.uy = [y + 7.5 * math.sin(i_yaw + math.pi / 2.0) for y, i_yaw in zip(ty, tyaw)]
    csp.lx = [x - 2.5 * math.cos(i_yaw + math.pi / 2.0) for x, i_yaw in zip(tx, tyaw)]
    csp.ly = [y - 2.5 * math.sin(i_yaw + math.pi / 2.0) for y, i_yaw in zip(ty, tyaw)]

    # Initial state of obstacle vehicle
    obs_s0 = 40.0  # Current position
    obs_speed = 15.0 / 3.6  # Current speed [m/s]
    obs_acc = 0.0  # Current acceleration [m/s^2]
    obs_d = 0.0  # Lateral position [m]
    obs_d_d = 0.0  # Lateral velocity [m/s]
    obs_d_dd = 0.0  # Lateral acceleration [m/s^2]

    # Initial state of ego vehicle
    c_speed = 30.0 / 3.6  # Current speed [m/s]
    c_accel = 0.0  # Current acceleration [m/s^2]
    c_d = 0.0  # Current lateral position [m]
    c_d_d = 0.0  # Current lateral speed [m/s]
    c_d_dd = 0.0  # Current lateral acceleration [m/s^2]
    s0 = 0.0  # Current course position

    for i in range(SIM_LOOP):
        obs_path = obstacle_planning(csp, obs_s0, obs_speed, obs_acc, obs_d, obs_d_d, obs_d_dd)
        path, fplist = parameter(csp, s0, c_speed, c_accel, c_d, c_d_d, c_d_dd, obs_path)

        if path is None:
            break

        # Check if the ego vehicle has reached the goal
        if np.hypot(path.x[1] - tx[-1], path.y[1] - ty[-1]) <= 0.0:
            print("Goal")
            break

        # Update obstacle vehicle state
        obs_s0 = obs_path.s[1]
        obs_d = obs_path.d[1]
        obs_d_d = obs_path.d_d[1]
        obs_d_dd = obs_path.d_dd[1]
        obs_speed = obs_path.s_d[1]
        obs_acc = obs_path.s_dd[1]

        # Update ego vehicle state
        s0 = path.s[1]
        c_d = path.d[1]
        c_d_d = path.d_d[1]
        c_d_dd = path.d_dd[1]
        c_speed = path.s_d[1]
        c_accel = path.s_dd[1]

        if show_animation:
            plt.cla()
            plt.gcf().canvas.mpl_connect('key_release_event', lambda event: [exit(0) if event.key == 'escape' else None])

            # Draw road boundaries
            plt.plot(csp.ux, csp.uy, '-k')#Upper boundary
            plt.plot(csp.lx, csp.ly, '-k')#Lower boundary

            # Calculate and plot the center line between boundaries
            mid_x = [(ux + lx) / 2 for ux, lx in zip(csp.ux, csp.lx)]
            mid_y = [(uy + ly) / 2 for uy, ly in zip(csp.uy, csp.ly)]
            plt.plot(mid_x, mid_y, '--k')

            # Draw ego vehicle
            ego_yaw = path.yaw[1] * 180 / np.pi
            ego_vehicle = Rectangle(
                (path.x[1] - ego_length / 2, path.y[1] - ego_width / 2),
                ego_length, ego_width,
                edgecolor="blue", facecolor="none"
            )
            t = transforms.Affine2D().rotate_deg_around(path.x[1], path.y[1], ego_yaw) + plt.gca().transData
            ego_vehicle.set_transform(t)
            plt.gca().add_patch(ego_vehicle)

            # Draw obstacle vehicle
            obs_yaw = obs_path.yaw[1] * 180 / np.pi
            obs_vehicle = Rectangle(
                (obs_path.x[1] - obs_length / 2, obs_path.y[1] - obs_width / 2),
                obs_length, obs_width,
                edgecolor="red", facecolor="none"
            )
            t = transforms.Affine2D().rotate_deg_around(obs_path.x[1], obs_path.y[1], obs_yaw) + plt.gca().transData
            obs_vehicle.set_transform(t)
            plt.gca().add_patch(obs_vehicle)

            # Plot paths
            plt.plot(tx, ty, label="Target Path")
            plt.plot(obs_path.x[1:], obs_path.y[1:], "-r", label="Obstacle Path")
            plt.plot(path.x[1:], path.y[1:], "-g", label="Ego Path")

            # Visualize paths from fplist (top 10 paths)
            for idx, fp in enumerate(fplist[:10]):  # Only plot the top 10 paths
                plt.plot(fp.x, fp.y, linestyle="--", label=f"Path {idx + 1}")

            plt.xlim(path.x[1] - area, path.x[1] + area)
            plt.ylim(path.y[1] - area, path.y[1] + area)
            plt.title(f"Ego v[km/h]: {c_speed * 3.6:.2f}, Obs v[km/h]: {obs_speed * 3.6:.2f}")
            plt.grid(True)
            plt.legend(loc='center left', bbox_to_anchor=(0.9, 0.5))
            plt.pause(0.0001)

    print("Finish")
    if show_animation:
        plt.grid(True)
        plt.show()

if __name__ == '__main__':
    main()
