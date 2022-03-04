import pandas as pd
import numpy as np

def load_trajectory():
    dir_path = 'C:/Users/USER/Desktop/FullstanleyController/Ampera/data/InfoKNMT.csv'
    dataframe = pd.read_csv(dir_path)
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
    
    return x,y
