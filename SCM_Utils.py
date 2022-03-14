import numpy as np
import pandas as pd

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
                Controller Output Variables:
                                      lim_steering_angle    : Steer output (-1.22 rad to 1.22 rad)
"""

def load_waypoints():

    dataframe = pd.read_csv("C:/Users/USER/Desktop/FullStanleyController/Ampera/data/InfoKNMT.csv")
    dataframe = dataframe[["Y", "X", "Orientação"]][:440]

    point_x, point_y = dataframe["Y"].multiply(-1).tolist(), dataframe["X"].tolist()

    points = np.asarray([point_x, point_y]).T
    points = np.round_(points, decimals=4)

    xnew = points[:, 0]
    xnew = np.reshape(xnew, (440, 1))
    ynew = points[:, 1]
    ynew = np.reshape(ynew, (440, 1))

    waypoints = np.concatenate((xnew, ynew), axis = 1)
    waypoints = np.asarray(waypoints)

    return waypoints

class Controller(object):
    def __init__(self):

        self._cte_ref_dist       = 0.4 # Distance from vehicle centre to front axle (m)
        self._eps_lookahead      = 10**(-3) # Epsilon distance approximation  (m)
        self._conv_rad_to_steer  = 180.0 / 70.0 / np.pi

        self._Kcte               = 1.5
        self._Ksoft              = 1e-5
        self._Kvel               = 1.3

    def get_distance(self, x1, y1, x2, y2):
        return np.sqrt((x1 - x2)**2 + (y1 - y2)**2)

    def get_lookahead_point_index(self, x, y, waypoints, lookahead_dis): #  Aqui pode ter um erro ainda, veremos no teste
        for i in range(len(waypoints)):
            dis = self.get_distance(x, y, waypoints[i][0], waypoints[i][1])
            if abs(dis - lookahead_dis) <= self._eps_lookahead:
                return i
        return i

    def get_steering_direction(self, v1, v2):
        corss_prod = v1[0] * v2[1] - v1[1] * v2[0]
        if corss_prod >= 0:
            return -1
        return 1

    def get_crosstrack_error(self, x, y, waypoints):
        P = np.asarray([x, y])
        P = np.reshape(P, (1,2))
        P = P[0]

        i = self.get_lookahead_point_index(x, y, waypoints, self._cte_ref_dist) # Get current waypoint index

        if i == 0:
            A = np.asarray([waypoints[i][0], waypoints[i][1]])
            B = np.asarray([waypoints[i+1][0], waypoints[i+1][1]])
        else:
            A = np.asarray([waypoints[i-1][0], waypoints[i-1][1]])
            B = np.asarray([waypoints[i][0], waypoints[i][1]])

        n = B-A
        m = P-A

        dirxn = self.get_steering_direction(n, m)

        if np.sqrt((B[0] - A[0]) ** 2 + (B[1] - A[1]) ** 2) == 0.0:
            crosstrack_error = 0.0
        else:
            crosstrack_error = dirxn * (np.abs(((B[0] - A[0]) * (A[1] - P[1])) - ((A[0] - P[0]) * (B[1] - A[1]))) / np.sqrt((B[0] - A[0]) ** 2 + (B[1] - A[1]) ** 2))

        return crosstrack_error

    def get_heading_error(self, waypoints, current_yaw):
        waypoint_delta_x = waypoints[1][0] - waypoints[0][0]
        waypoint_delta_y = waypoints[1][1] - waypoints[0][1]
        
        if waypoint_delta_x == 0.0: 
            waypoint_heading = 0.0
        else: 
            waypoint_heading = np.arctan(waypoint_delta_y / waypoint_delta_x)

        heading_error_mod = divmod((waypoint_heading - current_yaw), np.pi)[1]
        if np.pi / 2 < heading_error_mod < np.pi:
            heading_error_mod -= np.pi

        return heading_error_mod   

    def calculate_steering(self, x, y, yaw, waypoints, v):
        heading_error = self.get_heading_error(waypoints, yaw)
        cte_term = self._Kcte * self.get_crosstrack_error(x, y, waypoints)
        cte_term = np.arctan(cte_term/(self._Ksoft+self._Kvel*v))
        cte_term = divmod(cte_term, np.pi)[1]
        if cte_term > np.pi/2 and cte_term < np.pi:
            cte_term -= np.pi
        steering =  (heading_error + cte_term) # Stanley control law
        #print("O seu steering no ", i+1, " par é ", steering)
        return steering

    def set_steer(self, steering_input_in_rad):
        # Convert radians to [-1, 1]
        input_steer = self._conv_rad_to_steer * steering_input_in_rad
        # Clamp the steering command to valid bounds
        lim_steering           = np.fmax(np.fmin(input_steer, 1.0), -1.0)
        #print("O seu steering limitado no  ", i+1, " par é ", lim_steering)
        return lim_steering
