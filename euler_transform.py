
import numpy as np

def transform_point(origin_pose, goal_point):
    '''
    Transforma un punto en el espacio 3D desde un sistema de referencia original
    definido por una posición y orientación en ángulos de Euler (en radianes).

    Parámetros:
        origin_pose: lista de 6 valores [x, y, z, alpha_x, alpha_y, alpha_z]
                     donde alpha_* son ángulos de rotación en radianes
        goal_point: lista de 3 valores [goal_x, goal_y, goal_z]

    Retorna:
        Lista con las coordenadas del punto transformado [x', y', z']
    '''

    # Extraer posición y orientación
    x, y, z, alpha_x, alpha_y, alpha_z = origin_pose
    gx, gy, gz = goal_point

    # Rotación en Euler: ZYX (roll-pitch-yaw)
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(alpha_x), -np.sin(alpha_x)],
        [0, np.sin(alpha_x),  np.cos(alpha_x)]
    ])

    Ry = np.array([
        [np.cos(alpha_y), 0, np.sin(alpha_y)],
        [0, 1, 0],
        [-np.sin(alpha_y), 0, np.cos(alpha_y)]
    ])

    Rz = np.array([
        [np.cos(alpha_z), -np.sin(alpha_z), 0],
        [np.sin(alpha_z),  np.cos(alpha_z), 0],
        [0, 0, 1]
    ])

    # Matriz de rotación compuesta: R = Rz @ Ry @ Rx
    R = Rz @ Ry @ Rx

    # Aplicar rotación al punto objetivo
    rotated_point = R @ np.array([gx, gy, gz])

    # Aplicar traslación
    transformed_point = rotated_point + np.array([x, y, z])

    return transformed_point.tolist()
