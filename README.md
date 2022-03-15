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

+ SCM_Utils_optimized.py
  - Contem as funções de controle do carro e calculos do steering_angle

+ main_SCM.py
  - Comunica com o simulador utilizando as funções em SCM_Utils_optimized.py

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
"""
  
func load_waypoints() : Pega o nosso .csv e trata os dados para que se torne um array (440,2) com as coordenadas x e y dos waypoints; 
class Controller() : Define todas as funções a seguir e mantém armazenado todas as constantes do cálculo; 
func steering() : Calcula heading_error, crosstrack_error e steering_angle baseado nos dois erros.
```

</details>

<hr>
