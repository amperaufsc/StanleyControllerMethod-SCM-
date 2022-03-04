# Integração SCM-FSDS

<img alt="Python" src="https://img.shields.io/badge/python%20-%2314354C.svg?&style=for-the-badge&logo=python&logoColor=white"/>

<p>

```python
import pandas as pd
import numpy as np
print(pandas.__version__) #To see what is your version of Pandas in Python
print(numpy.__version__) #To see what is your version of Numpy in Python
print(matplotlib.__version__) #To see what is your version of Numpy in Python
```
#### MATPLOTLIB : 3.4.0
#### PANDAS : 1.2.3
#### NUMPY : 1.19.2
</p>
</details>

<hr>

+ functions.py
  - Contem as funções de controle do carro e calculos da trajetoria

+ main_stanley_controller.py
  - Comunica com o simulador utilizando as funções em functions.py

+ trajectory.py
  - Contem os waypoints da trajetoria

<hr>

### VARIÁVEIS

<details><summary>DESCRIÇÃO</summary>
  
<p>
```python
:param control_gain:                (float) time constant [1/s]
:param softening_gain:              (float) softening gain [m/s]
:param yaw_rate_gain:               (float) yaw rate gain [rad]
:param steering_damp_gain:          (float) steering damp gain
:param max_steer:                   (float) vehicle's steering limits [rad]
:param wheelbase:                   (float) vehicle's wheelbase [m]
:param px:                          (numpy.ndarray) list of x-coordinates along the path
:param py:                          (numpy.ndarray) list of y-coordinates along the path
:param pyaw:                        (numpy.ndarray) list of discrete yaw values along the path
:param dt:                          (float) discrete time period [s]
:param x:                           (float) vehicle's x-coordinate [m]
:param y:                           (float) vehicle's y-coordinate [m]
:param yaw:                         (float) vehicle's heading [rad]
:param target_velocity:             (float) vehicle's velocity [m/s]
:param steering_angle:              (float) vehicle's steering angle [rad]

:return limited_steering_angle:     (float) steering angle after imposing steering limits [rad]
:return target_index:               (int) closest path index
:return crosstrack_error:           (float) distance from closest path index [m]
```
</p>
</details>

<hr>
