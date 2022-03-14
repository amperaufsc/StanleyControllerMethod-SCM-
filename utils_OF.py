import math


def get_orientation(vector):
    w_val = vector.w_val
    z_val = vector.z_val
    a_acos = math.acos(w_val)
    if z_val < 0:
        angle = math.degrees(-a_acos)
    else:
        angle = math.degrees(a_acos)

    return 2 * angle



