import socket
import urx
import time
import numpy as np
from Toolkit import point_transformation

from euler_transform import transform_point
resultado = transform_point([1, 2, 3, 0.1, 0.2, 0.3], [1, 0, 0])
print(resultado)

# POSITION CHANGE   
# X = 
# Y = 
# Z = 74.3
# ALPHA_X = -4°
# ALPHA_Y = 0.0
# ALPHA_Z = 0.0 


# ENVIROMENT PARAMETERS
robot_ip = '157.253.197.147'
home_point = [-0.00106, -0.1945, 0.75, 0.006, 2.224, -2.224]
robot = urx.Robot(robot_ip)

def main_menu():
    global robot_ip
    global home_point
    sentinel = False
    function_dict = {
        '1': go_to_home_pose,
        '2': go_to_manipulation_pose,
        '3': go_to_target_point,
        '4': turn_on_VG_urx,
        '5': turn_of_VG_urx
    }
    print('============== UR MOVEMENT SYSTEM =================')
    print('==================== VERSION 1.0 ==================')
    while not sentinel:
        print('1. Go to home pose')
        print('2. Go to manipulation pose')
        print('3. Move the robot')
        print('4. Turn on the VG')
        print('5. Turn off the VG')
        print('0. Close program')
        input_signal = input("----------------------------------------------\nEnter a mobility option for the robot:")
        if input_signal == '0':
            print('Closing program....')
            time.sleep(3)
            return
        else:
            time.sleep(3)
            print(f'\nExecuting function {str(function_dict[input_signal])}')
            if input_signal == '1' or input_signal == '2' or input_signal == '3' or input_signal == '4' or input_signal == '5':
                function_dict[input_signal]()
            if input_signal == 'A':
                function_dict[input_signal](goal_point = home_point)
    return

def go_to_home_pose():
    global robot_ip
    movej_instruction = """
    movej([0.0, -1.57, 0.0, -1.57, 0.0, 0.0], a=1.0, v=0.5)
    """
    execute_instruction(robot_ip, movej_instruction)
    return

def go_to_manipulation_pose():
    global robot_ip
    angle_base = np.radians(-44.43)
    angle_shoulder = np.radians(-106.25)
    angle_elbow = np.radians(-12.8)
    angle_wrist_1 = np.radians(-90.0)
    angle_wrist_2 = np.radians(87.81)
    angle_wrist_3 = np.radians(0.0)
    movej_instruction = f"""
    movej([{angle_base}, {angle_shoulder}, {angle_elbow}, {angle_wrist_1}, {angle_wrist_2}, {angle_wrist_3}], a=1.0, v=0.5)
    """
    execute_instruction(robot_ip, movej_instruction)
    return

def go_to_target_point():
    global robot_ip
    go_to_manipulation_pose()
    x = float(input("Enter the X coordinate of the target point: "))
    y = float(input("Enter the Y coordinate of the target point: "))
    z = float(input("Enter the Z coordinate of the target point: "))
    B = np.array([x, y, z + 0.105])
    A = np.array([-0.5705, 0.131, 0.074])
    alpha_x = np.radians(0)
    alpha_y = np.radians(0)
    alpha_z = np.radians(-45)

    x_transformado, y_transformado, z_transformado = point_transformation(B, A, alpha_x, alpha_y, alpha_z)
    punto_transformado = [x_transformado, y_transformado, z_transformado, 1.163, -2.946, 0.036]
    print(f"El punto transformado es: {punto_transformado}")
    # Enviar el punto transformado al robot
    move_linear(punto_transformado)
    print("Nuestra trayectoria ha sido ejecutada por el robot")
    return

def turn_on_VG_urx():
    global robot_ip 
    robot = urx.Robot(robot_ip)
    robot.set_digital_out(8, True)
    robot.set_digital_out(9, True)
    print("Tool outputs 0 y 1 encendidos")
    robot.close()
    return

def turn_of_VG_urx():
    global robot_ip 
    robot = urx.Robot(robot_ip)
    robot.set_digital_out(8, False)
    robot.set_digital_out(9, False)
    print("Tool outputs 0 y 1 encendidos")
    robot.close()
    return

def turn_on_VG():
    global robot_ip
    turn_on_instruction = """
    set_tool(0)
    set_payload(1.62, [0, 0, 0])
    set_tool_digital_output(0, True)
    set_tool_digital_output(1, True)
    """
    execute_instruction(robot_ip, turn_on_instruction)
    print("VG encendido")
    return

def move_linear(goal_point):
    global robot_ip
    x, y, z, angle_x, angle_y, angle_z = goal_point
    movel_instruction = f"""
    movel(p[{x}, {y}, {z}, {angle_x}, {angle_y}, {angle_z}], a=1.0, v=0.5)
    """
    print(f"\n{movel_instruction}\n")
    execute_instruction(robot_ip, movel_instruction)
    return

def move_to_pose(goal_point, resolution):
    global robot_ip
    global home_point
    x_home, y_home, z_home, home_angle_x, home_angle_y, home_angle_z = home_point
    x_goal, y_goal, z_goal, goal_angle_x, goal_angle_y, goal_angle_z = goal_point
    list_movement = [[x_home, y_home, z_home, home_angle_x, home_angle_y, home_angle_z],
                    [x_goal, y_goal, resolution*z_home, home_angle_x, home_angle_y, home_angle_z],
                    [x_goal, y_goal, z_goal, goal_angle_x, goal_angle_y, goal_angle_z]]
    
    executemovement = True
    i=0
    while executemovement == True:
        x, y, z, angle_x, angle_y, angle_z = list_movement[i]

        movel_instruction = f"""
        movel(p[{x}, {y}, {z}, {angle_x}, {angle_y}, {angle_z}], a=1.0, v=0.5)
        """

        i += 1
    print('Movement finished')
    return

def execute_instruction(robot_ip, movel_instruction):
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(5)
            s.connect((robot_ip, 30002))
            s.send(movel_instruction.encode())
            response = s.recv(1024)
            print(f"Respuesta raw: {response}")
    except Exception as e:
        print(f"Error: {e}")

# Valores ajustados (aceleración y velocidad típicas)
# script = """
# movej([0.0, -1.57, 1.57, -1.57, -1.57, 0.0], a=1.0, v=0.5)
# """
movel_instruction = """
movel(p[-0.29, -0.11, -0.09, 2.22, 2.22, 0.001], a=1.0, v=0.5)
"""


# prueba_facilita()
if __name__ == "__main__":
    main_menu()

