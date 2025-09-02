import socket
import urx
import time
import numpy as np

class GEOMETRIC_TRANSFORMATIONS:
    
    """A class for performing basic geometric transformations in 3D space, including rotations about the principal axes and point transformations between reference frames.
    Methods
    -------
    rot_x(alpha):
        Returns the rotation matrix for a rotation about the X-axis by angle alpha (in radians).
    rot_y(alpha):
        Returns the rotation matrix for a rotation about the Y-axis by angle alpha (in radians).
    rot_z(alpha):
        Returns the rotation matrix for a rotation about the Z-axis by angle alpha (in radians).
    point_transformation(B, A, alpha_x, alpha_y, alpha_z):
        Transforms point B to a new reference frame with origin at A and rotated by angles alpha_x, alpha_y, and alpha_z (in radians) about the X, Y, and Z axes, respectively."""

    def __init__(self):
        
        return
    
    def rot_x(self, alpha):
        return np.array([
        [1, 0, 0],
        [0, np.cos(alpha), -np.sin(alpha)],
        [0, np.sin(alpha), np.cos(alpha)]
    ])

    def rot_y(self, alpha):
        return np.array([
            [np.cos(alpha), 0, np.sin(alpha)],
            [0, 1, 0],
            [-np.sin(alpha), 0, np.cos(alpha)]
        ])

    def rot_z(self, alpha):
        return np.array([
            [np.cos(alpha), -np.sin(alpha), 0],
            [np.sin(alpha), np.cos(alpha), 0],
            [0, 0, 1]
        ])

    def point_transformation(self, B, A, alpha_x, alpha_y, alpha_z):
        """
        Transforma el punto B a un nuevo sistema de referencia con origen en A
        y rotado con ángulos alpha_x, alpha_y, alpha_z.
        
        Parámetros:
        - B: np.array([x_B, y_B, z_B])
        - A: np.array([x_A, y_A, z_A])
        - alpha_x, alpha_y, alpha_z: ángulos de rotación en radianes
        
        Retorna:
        - np.array([x', y', z']) coordenadas de B en el nuevo sistema
        """
        # Traslado de origen
        B_trasladado = B - A
        
        # Matriz de rotación compuesta: R = R_x * R_y * R_z
        R = self.rot_x(alpha_x) @ self.rot_y(alpha_y) @ self.rot_z(alpha_z)
        
        # Aplicar rotación
        B_transformado = (R @ B_trasladado).tolist()
        return B_transformado

class ROBOT:
    def __init__(self, robot_ip):
        self.robot_ip = robot_ip
        # self.robot = urx.Robot(robot_ip)
        self.home_point = [-0.00106, -0.1945, 0.75, 0.006, 2.224, -2.224]
        self.methods = urx.Robot(robot_ip)
        return

    def close(self):
        try:
            self.methods.close()
        except Exception:
            pass

    def get_current_pose(self):
        t = self.methods.get_pose()            
        pv = t.pose_vector                     
        try:
            return list(pv)
        except TypeError:
            import numpy as np
            return np.array(pv, dtype=float).tolist()

class MANIPULATION:
    def go_to_home_pose(self, robot):
        robot_ip = robot.robot_ip
        movej_instruction = """
        movej([0.0, -1.57, 0.0, -1.57, 0.0, 0.0], a=1.0, v=0.5)
        """
        self.execute_instruction(robot, movej_instruction)
        return
    
    def go_to_pose(self, robot, pose):
        robot_ip = robot.robot_ip
        angle_base = pose[0]
        angle_shoulder = pose[1]
        angle_elbow = pose[2]
        angle_wrist_1 = pose[3]
        angle_wrist_2 = pose[4]
        angle_wrist_3 = pose[5]
        angle_base = np.radians(angle_base)
        angle_shoulder = np.radians(angle_shoulder)
        angle_elbow = np.radians(angle_elbow)
        angle_wrist_1 = np.radians(angle_wrist_1)
        angle_wrist_2 = np.radians(angle_wrist_2)
        angle_wrist_3 = np.radians(angle_wrist_3)
        movej_instruction = f"""
        movej([{angle_base}, {angle_shoulder}, {angle_elbow}, {angle_wrist_1}, {angle_wrist_2}, {angle_wrist_3}], a=1.0, v=0.5)
        """
        self.execute_instruction(robot, movej_instruction)
        return

    def go_to_manipulation_pose(self, robot):
        robot_ip = robot.robot_ip
        angle_base = np.radians(-44.43)
        angle_shoulder = np.radians(-106.25)
        angle_elbow = np.radians(-12.8)
        angle_wrist_1 = np.radians(-90.0)
        angle_wrist_2 = np.radians(87.81)
        angle_wrist_3 = np.radians(0.0)
        movej_instruction = f"""
        movej([{angle_base}, {angle_shoulder}, {angle_elbow}, {angle_wrist_1}, {angle_wrist_2}, {angle_wrist_3}], a=1.0, v=0.5)
        """
        self.execute_instruction(robot, movej_instruction)
        return

    def go_to_target_point(self, robot, point):
        object_point_transformation = GEOMETRIC_TRANSFORMATIONS()
        robot_ip = robot.robot_ip
        # self.go_to_manipulation_pose(robot=robot)
        if point is None:
            x = float(input("Enter the X coordinate of the target point: "))
            y = float(input("Enter the Y coordinate of the target point: "))
            z = float(input("Enter the Z coordinate of the target point: "))
        else:
            x = point[0]
            y = point[1]
            z = point[2]
        B = np.array([x, y, z + 0.105])
        A = np.array([-0.5705, 0.131, 0.074])
        alpha_x = np.radians(0)
        alpha_y = np.radians(0)
        alpha_z = np.radians(-45)

        x_transformado, y_transformado, z_transformado = object_point_transformation.point_transformation(B, A, alpha_x, alpha_y, alpha_z)
        punto_transformado = [x_transformado, y_transformado, z_transformado, 1.163, -2.946, 0.036]
        print(f"El punto transformado es: {punto_transformado}")
        # Enviar el punto transformado al robot
        self.move_linear(robot=robot, goal_point=punto_transformado)
        print("Nuestra trayectoria ha sido ejecutada por el robot")
        return

    def robot_origin_go_to_target_point(self, robot, point):
        object_point_transformation = GEOMETRIC_TRANSFORMATIONS()
        robot_ip = robot.robot_ip
        # self.go_to_manipulation_pose(robot=robot)
        if point is None:
            x = float(input("Enter the X coordinate of the target point: "))
            y = float(input("Enter the Y coordinate of the target point: "))
            z = float(input("Enter the Z coordinate of the target point: "))
        else:
            x = point[0]
            y = point[1]
            z = point[2]
        B = np.array([x, y, z + 0.105])
        A = np.array([0.0, 0.0, 0.0])
        alpha_x = np.radians(0)
        alpha_y = np.radians(0)
        alpha_z = np.radians(-45)

        x_transformado, y_transformado, z_transformado = object_point_transformation.point_transformation(B, A, alpha_x, alpha_y, alpha_z)
        punto_transformado = [x_transformado, y_transformado, z_transformado, 1.163, -2.946, 0.036]
        print(f"El punto transformado es: {punto_transformado}")
        # Enviar el punto transformado al robot
        self.move_linear(robot=robot, goal_point=punto_transformado)
        print("Nuestra trayectoria ha sido ejecutada por el robot")
        return

    def turn_on_VG_urx(self, robot):
        robot_ip = robot.robot_ip 
        robot = urx.Robot(robot_ip)
        robot.set_digital_out(8, True)
        robot.set_digital_out(9, True)
        print("Tool outputs 0 y 1 encendidos")
        robot.close()
        return

    def turn_off_VG_urx(self, robot):
        robot_ip = robot.robot_ip 
        robot = urx.Robot(robot_ip)
        robot.set_digital_out(8, False)
        robot.set_digital_out(9, False)
        print("Tool outputs 0 y 1 encendidos")
        robot.close()
        return

    def turn_on_VG(self, robot):
        robot_ip = robot.robot_ip
        turn_on_instruction = """
        set_tool(8)
        set_payload(1.62, [0, 0, 0])
        set_tool_digital_output(0, True)
        set_tool_digital_output(1, True)
        """
        self.execute_instruction(robot, turn_on_instruction)
        print("VG encendido")
        return

    def move_linear(self, robot, goal_point):
        robot_ip = robot.robot_ip
        x, y, z, angle_x, angle_y, angle_z = goal_point
        movel_instruction = f"""
        movel(p[{x}, {y}, {z}, {angle_x}, {angle_y}, {angle_z}], a=1.0, v=0.5)
        """
        print(f"\n{movel_instruction}\n")
        self.execute_instruction(robot, movel_instruction)
        return

    def move_to_pose(self, robot ,goal_point, resolution):
        robot_ip = robot.robot_ip
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

    def execute_instruction(self, robot, movel_instruction):
        print("Estoy en execute_instruction", robot)
        robot_ip = robot.robot_ip
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(5)
                s.connect((robot_ip, 30002))
                s.send(movel_instruction.encode())
                response = s.recv(1024)
                print(f"Respuesta raw: {response}")
        except Exception as e:
            print(f"Error: {e}")

class PERCEPTION:
    def transform_point(self, origin_pose, goal_point):
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
