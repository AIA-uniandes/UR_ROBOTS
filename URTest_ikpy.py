from ikpy.chain import Chain
import matplotlib.pyplot
from mpl_toolkits.mplot3d import Axes3D
import Toolkit
import math

ur3_robot = Chain.from_urdf_file("ur3.urdf")

answer = ur3_robot.inverse_kinematics([ -0.084-0.4, 0.100, 0.05], target_orientation=[0.0, 0.0, -1.0], orientation_mode="Z")
answer_degrees = [math.degrees(joint) for joint in answer]
print(f"Esta es la respuesta Answer en grados: {answer_degrees}")

ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d')

ur3_robot.plot(answer, ax)
# matplotlib.pyplot.show()

base = answer_degrees[2]
shoulder = answer_degrees[3]
elbow = answer_degrees[4]
wrist_1 = answer_degrees[5]
wrist_2 = answer_degrees[6]
wrist_3 = answer_degrees[7]

# shoulder = input("Escribe el angulo de shoulder: ")
# elbow = input("Escribe el angulo de elbow: ")
# wrist_1 = input("Escribe el angulo de wrist_1: ")
# wrist_2 = input("Escribe el angulo de wrist_2: ")
# wrist_3 = input("Escribe el angulo de wrist_3: ")

robot = Toolkit.ROBOT(robot_ip="157.253.231.86")
manipulation = Toolkit.MANIPULATION()
manipulation.go_to_pose(robot=robot ,pose=[float(base), float(shoulder), float(elbow), float(wrist_1), float(wrist_2), float(wrist_3)])