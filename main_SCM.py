from SCM_Utils import Controller, load_waypoints
import fsds
import sys
import math
import numpy as np
from utils_OF import get_orientation
import warnings
warnings.filterwarnings("ignore")
import pandas as pd

fsds_lib_path = r"C:\Users\merte\Formula-Student-Driverless-Simulator\python\fsds" #exemplo do Mertens
sys.path.insert(0, fsds_lib_path)

# ConexÃ£o ao AirSim do FSDS
client = fsds.FSDSClient()
# Confirmar conexÃ£o com o FSDS
client.confirmConnection()
# Habilitar controle
client.enableApiControl(True)
car_controls = fsds.CarControls()

waypoints = load_waypoints()

client = fsds.FSDSClient()
state = client.getCarState()
gss = client.getGroundSpeedSensorData(vehicle_name='FSCar')
car_controls = fsds.CarControls()

controlador = Controller()
accel = 0.01
velocity = 0.25 #teste para velocidade constante
car_controls.throttle = accel

steering = Controller.calculate_steering()
car_controls.steering = steering

while True:

    state = client.getCarState()
    gss = client.getGroundSpeedSensorData(vehicle_name='FSCar')

    #velocity = np.linalg.norm(gss.linear_velocity.to_numpy_array()[:2])

    pos_x = -state.kinematics_estimated.position.y_val
    pos_y = state.kinematics_estimated.position.x_val
    yaw = math.radians(get_orientation(state.kinematics_estimated.orientation))

    steering_angle = controlador.calculate_steering(pos_x, pos_y, yaw, waypoints, velocity)
    lim_steering_angle = controlador.set_steer(steering_angle)
    car_controls.steering = lim_steering_angle
    
    client.setCarControls(car_controls)