import numpy as np
import pandas as pd

## LOAD_WAYPOINTS OK
dataframe = pd.read_csv("C:/Users/USER/Desktop/FullStanleyController/Ampera/data/InfoKNMT.csv")
dataframe = dataframe[["Y", "X", "Orientação"]][:440]


point_x, point_y = dataframe["Y"].multiply(-1).tolist(), dataframe["X"].tolist()
points = np.asarray([point_x, point_y]).T
points = np.round_(points, decimals=4)

xnew = points[:, 0]
xnew = np.reshape(xnew, (440, 1))
ynew = points[:, 1]
ynew = np.reshape(ynew, (440, 1))
#print(xnew)
#print(ynew.shape)

_cte_ref_dist       = 0.4 
_eps_lookahead      = 10**(-3) 
_conv_rad_to_steer  = 180.0 / 70.0 / np.pi
_Kcte               = 1.5
_Ksoft              = 1e-5
_Kvel               = 1.3

#VALORES DE TESTE
v = 0.25
yawnew = dataframe["Orientação"].tolist()
yawnew = np.round_(yawnew, decimals=4)
yawnew = np.asarray(yawnew)
yawnew = np.reshape(yawnew, (440, 1))

#print(yawnew.shape)
#print(yawnew[0]) #436
#print(yawnew)

waypoints = np.concatenate((xnew, ynew), axis = 1)
waypoints = np.asarray(waypoints)

#print(waypoints)
#print(waypoints.shape)
#print(type(yawnew))
#print(type(waypoints))

#print(waypoints)
#print(waypoints[5][0])
#print(waypoints[5][1])


index = 0
##### TESTE de 1 a 20
for i in range (0,50, 1): 
    yaw = yawnew[i]
    #print("o valor de yaw é ", yaw)
    x1 = xnew[i]
    #print("o valor de x1 é ", x1)
    y1 = ynew[i] 
    #print("o valor de y1 é ", y1)
    x2 = xnew[i+1] 
    #print("o valor de x2 é ", x2)
    y2 = ynew[i+1]
    #print("o valor de y2 é ",y2)


    #get_lookahed_point_index
    dis = np.sqrt((x1 - x2)**2 + (y1 - y2)**2)
    #print(dis)
    if abs(dis - _cte_ref_dist) <= _eps_lookahead:
        index = i 
        #print(index)
    index = i
    #print("estamos no ", index, " dado")

    #get_crosstrack_error
    P = np.asarray([x1, y1])
    P = np.reshape(P, (1,2))
    P = P[0]
    if index == 0: 
        A = np.asarray([waypoints[index][0], waypoints[index][1]])
        B = np.asarray([waypoints[index+1][0], waypoints[index+1][1]])
    else: 
        A = np.asarray([waypoints[index-1][0], waypoints[index-1][1]])
        B = np.asarray([waypoints[index][0], waypoints[index][1]])

    n = B - A
    m = P - A 
    #print(P, "\n") 
    #print(A, "\n") 
    #print(B, "\n")  
    #print(n, "\n") 
    #print(m, "\n") 

    #get_steering_direction
    corss_prod = n[0] * m[1] - n[1] * m[0] #realiza o produto vetorial para obter a direção 
    #print(corss_prod)
    if corss_prod >= 0: 
        dirxn = -1
        #print("O seu steering do ", i+1 ," par é positivo")
    else:
        dirxn = 1
        #print("O seu steering do ", i+1 ," par é negativo")
    
    if np.sqrt((B[0] - A[0]) ** 2 + (B[1] - A[1]) ** 2) == 0.0:
        crosstrack_error = 0.0
        #print("O crosstrack do seu ", i+1 ,"par é de ", crosstrack_error)
    else:
        crosstrack_error = dirxn * (np.abs(((B[0] - A[0]) * (A[1] - P[1])) - ((A[0] - P[0]) * (B[1] - A[1]))) / np.sqrt((B[0] - A[0]) ** 2 + (B[1] - A[1]) ** 2))
        #print("O crosstrack do seu ", i+1 ,"par é de ", crosstrack_error)

    #get_heading_error
    waypoint_delta_x = waypoints[1][0] - waypoints[0][0]
    waypoint_delta_y = waypoints[1][1] - waypoints[0][1]

    if waypoint_delta_x == 0.0: 
        waypoint_heading = 0.0
    else: 
        waypoint_heading = np.arctan(waypoint_delta_y / waypoint_delta_x)
    

    heading_error_mod = divmod((waypoint_heading - yaw), np.pi)[1]
    #print("O seu heading error do ", i+1 , " par é de ", heading_error_mod)

    if np.pi / 2 < heading_error_mod < np.pi:
        heading_error_mod -= np.pi
        #print("O seu heading error do ", i+1 , " par é de ", heading_error_mod)


    #calculate_steering
    heading_error = heading_error_mod
    cte_term = _Kcte * crosstrack_error
    cte_term = np.arctan(cte_term/(_Ksoft+_Kvel*v))
    cte_term = divmod(cte_term, np.pi)[1]
    if cte_term > np.pi/2 and cte_term < np.pi:
        cte_term -= np.pi
    steering =  (heading_error + cte_term) # Stanley control law
    #print("O seu steering no ", i+1, " par é ", steering)

    #set_steer
    # Convert radians to [-1, 1]
    input_steer = _conv_rad_to_steer * steering
    # Clamp the steering command to valid bounds
    lim_steering           = np.fmax(np.fmin(input_steer, 1.0), -1.0)
    lim_steering = lim_steering[0]
    print("O seu steering limitado no  ", i+1, " par é ", lim_steering)