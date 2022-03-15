import numpy as np
import pandas as pd
from math import atan2, sin, cos
from scipy import interpolate

"""
                Controller iteration code block.
                Controller Feedback Variables:
                    x                                        : Current X position (meters)
                    y                                        : Current Y position (meters)
                    yaw                                      : Current yaw pose (radians)
                    v                                        : Current forward speed (meters per second)
                    waypoints                                : Current waypoints to track
                                                               Format: [[x0, y0],
                                                                        [x1, y1],
                                                                         ...
                                                                        [xn, yn]]
                                                               Example:
                                                                       waypoints[2][1]:
                                                                       Returns the 3rd waypoint's y position
                                                                       waypoints[5]:
                                                                       Returns [x5, y5] (6th waypoint)
"""

def load_waypoints():

    dataframe = pd.read_csv("C:/Users/USER/Desktop/FullStanleyController/Ampera/data/InfoKNMT.csv")
    dataframe = dataframe[["Y", "X"]][:440]

    point_x, point_y = dataframe["Y"].multiply(
        -1).tolist(), dataframe["X"].tolist()
    points = np.array([point_x, point_y]).T
    points = np.round_(points, decimals=3)

    single_points = np.unique(points, axis=0, return_index=True)
    single_points = np.concatenate(
        (single_points[0], single_points[1][:, None]), axis=1)
    single_points = single_points[single_points[:, 2].argsort()]
    x, y = zip(*single_points[:, :2])
    tckp, u = interpolate.splprep([x, y], k=2)
    xnew, ynew = interpolate.splev(np.linspace(0, 1, 10000), tckp)
    waypoints = np.concatenate(
        (np.reshape(xnew, (10000, 1)), np.reshape(ynew, (10000, 1))), axis=1)

    return waypoints


class Controller(object):
    def __init__(self):

        self._Kcte               = 1.5
        self._Kvel               = 1.3

    def steering(self, waypoints, x, y, yaw, v): 

        # Heading error
        last_point_on_trajectory = waypoints[-1]
        first_point_on_trajectory = waypoints[0]
        yaw_path = np.arctan2(last_point_on_trajectory[:,1] - first_point_on_trajectory[:,1],
                        last_point_on_trajectory[:,0] - first_point_on_trajectory[:,0]) #Calcula a curvatura aproximada da trajetoria
        yaw_diff = yaw_path - yaw

        normalize_angle = lambda angle : atan2(sin(angle), cos(angle))
        yaw_diff = normalize_angle(yaw_diff)

        heading_error = yaw_diff

        # Cross track error
        center_axle_current = np.array([x, y])
        crosstrack_error = np.min(np.sum((center_axle_current - waypoints[:, :2]) ** 2, axis=1))

        yaw_cross_track = np.arctan2(y - first_point_on_trajectory[:,1], x - first_point_on_trajectory[:,0])
        yaw_diff_of_path_cross_track = normalize_angle(yaw_path - yaw_cross_track)
        crosstrack_error = abs(crosstrack_error) if yaw_diff_of_path_cross_track > 0 else -abs(crosstrack_error)

        yaw_diff_crosstrack = np.arctan(self._Kcte * crosstrack_error / (self._Kvel + v)) 

        expected_steering_angle = max(-1.22, min(1.22, normalize_angle(yaw_diff + yaw_diff_crosstrack)))

        return heading_error, crosstrack_error, expected_steering_angle

