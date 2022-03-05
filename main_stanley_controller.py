import sys
import time
from trajectory import load_trajectory

import numpy as np
import math
from functions import *


#fsds_lib_path = r"C:\Users\USER\Desktop\Formula-Student-Driverless-Simulator\python\fsds"
fsds_lib_path = os.path.join(os.path.expanduser("~"), "Formula-Student-Driverless-Simulator", "python")
sys.path.insert(0, fsds_lib_path)

import fsds

# Frequencia em Hz
frequency = 50
dt = 1 / frequency

# ConexÃ£o ao AirSim do FSDS
client = fsds.FSDSClient()

# Confirmar conexÃ£o com o FSDS
client.confirmConnection()

# Habilitar controle
client.enableApiControl(True)

car_controls = fsds.CarControls()

x,y = load_trajectory()
ds = 0.05 


px, py, pyaw, _ = generate_cubic_spline() 

# Parametros iniciais 

k = 8.0                                                     #(float) time constant [1/s]
ksoft = 1.0                                                 #(float) softening gain [m/s]
kyaw = 0.0                                                  #(float) yaw rate gain [rad]
ksteer = 0.0                                                #(float) steering damp gain
wheelbase = 2.5                                             #(float) vehicle's wheelbase [m]
fps = 50.0
dt = 1/fps
max_steer = np.deg2rad(33)                                  #(float) vehicle's steering limits [rad]
steering_angle = 0.0                                        #(float) vehicle's steering angle [rad] (COMPLETAR AMANHA)
#:param x:                                                  #(float) vehicle's x-coordinate [m]
#:param y:                                                  #(float) vehicle's y-coordinate [m]
#:param yaw:                                                #(float) vehicle's heading [rad]
#:param steering_angle:                                     #(float) vehicle's steering angle [rad]
#:limited_steering_angle:                                   #(float) steering angle after imposing steering limits [rad]
#:param: px                                                 #(numpy.ndarray) list of x-coordinates along the path
#:param: py                                                 #(numpy.ndarray) list of y-coordinates along the path
#:param: pyaw                                               #(numpy.ndarray) list of discrete yaw values along the path

t0 = time.time()


tracker = StanleyController(k, ksoft, kyaw, ksteer, max_steer, wheelbase, px, py, pyaw)
kbm = KinematicBicycleModel(wheelbase, dt)

while time.time()-t0 < 60*5:


    state = client.getCarState()
    gss = client.getGroundSpeedSensorData(vehicle_name='FSCar')

    car_controls.throttle = 0.025

    v = np.linalg.norm(gss.linear_velocity.to_numpy_array()[:2])
    
    x, y, yaw = kbm.kinematic_model(x, y, yaw, v, steering_angle)
    lim_steering_angle, target_id, crosstrack_error = tracker.stanley_control(x ,y, yaw, v, steering_angle)
    
    car_state = [x+0.4 * math.cos(yaw), y+0.4 * math.sin(yaw)]

    car_controls.steering = lim_steering_angle 
    client.setCarControls(car_controls)

client.enableApiControl(False)