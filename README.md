# Integração SCM-FSDS

<img alt="Python" src="https://img.shields.io/badge/python%20-%2314354C.svg?&style=for-the-badge&logo=python&logoColor=white"/>

<p>

```python
import pandas as pd
import numpy as np
print(pandas.__version__) #To see what is your version of Pandas in Python
print(numpy.__version__) #To see what is your version of Numpy in Python
```
#### PANDAS : 1.2.3
#### NUMPY : 1.19.2
</p>
</details>

<hr>

+ SCM_Utils.py
  - Contem as funções de controle do carro e calculos do steering_angle

+ main_SCM.py
  - Comunica com o simulador utilizando as funções em SCM_Utils.py

+ teste_funções.py
  - Contém o teste completo iterativo de todas as funções para quantos pontos quiser testar

+ utils_OF.py
  - Contém a função que pega o yaw do simulador

<hr>

### VARIÁVEIS

<details><summary>DESCRIÇÃO</summary>
  

```python
  
#Na linha 27 de SMC_Utils.py lembre-se de trocar o path dos waypoints de acordo com o caminho na sua máquina
dataframe = pd.read_csv("C:/.../InfoKNMT.csv")

#Na linha 11 de main_SCM.py lembre-se de trocar o path da pasta "fsds" de acordo com o caminho da sua máquina
fsds_lib_path = r"C:\...\Formula-Student-Driverless-Simulator\python\fsds"
  
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
  
func load_waypoints() : Pega o nosso .csv e trata os dados para que se torne um array (440,2) com as coordenadas x e y dos waypoints; 
class Controller() : Define todas as funções a seguir e mantém armazenado todas as constantes do cálculo; 
func get_distance() : Calcula a distância entre o waypoint i e o waypoint i+1; 
func get_lookahead_point_index() : Nos diz em que posição do array estamos;
func get_steering_direction() : Faz um produto vetorial para descobrir se o steering é negativo ou positivo; 
func get_crosstrack_error() : Calcula a distância entre o waypoint mais próximo e a parte da frente do carro; 
func get_heading_error() : Calcula o angulo entre a reta da trajetória e a reta do carro; 
func calculate_steering() : A partir das duas funções acima, calcula o steering angle em RAD; 
func set_steer() : Coloca o steering angle dentro dos limites.
```

</details>

<hr>
