import numpy as np
import mujoco
import mujoco.viewer
from gradIK import GradientDescentIK
from ik_calculator import ScipyIK

# Path to UR5e Robot Arm
arm = "C:/Users/comrade/Downloads/mujoco_menagerie-main/mujoco_menagerie-main/universal_robots_ur5e/scene.xml"
model = mujoco.MjModel.from_xml_path(arm)
data = mujoco.MjData(model)
renderer = mujoco.Renderer(model)

# Custom Camera angle
camera = mujoco.MjvCamera()
mujoco.mjv_defaultFreeCamera(model, camera)
camera.distance = 1

body_id = model.body('wrist_3_link').id

# Initialize variables for IK
goal = [0.49, 0.13, 0.59]
init_q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

choice = int(input('Enter your choice\n1. Gradient Descent Optimization\n2.Limited-Memory BFGS Optimization\n3.Exit\n'))

if (choice == 1):
    gradIK_solver = GradientDescentIK(model, data)
    #Calculate desired joint angles for the goal position.
    gradIK_solver.calculate(goal, init_q, body_id)

elif (choice == 2):
    ik_solver = ScipyIK (model, data)
    # Calculate desired joint angles for the goal position.
    ik_solver.calculate(goal, init_q, body_id, method="L-BFGS-B")

else:
    print('Thanks for trying :)')
    exit(0)
    
result = data.qpos.copy()

# Key Callback to pause the animation
pause = False

def key_callback(keycode):
    global pause
    if chr(keycode) == ' ':
        pause = not pause

# Animation setup
num_steps = 10000  
interpolated_qpos = np.linspace(init_q, result, num_steps)

with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
    for i in range(num_steps):
        if viewer.is_running():
            data.qpos[:] = interpolated_qpos[i]
            mujoco.mj_forward(model, data)
            viewer.sync()

            if pause:
                viewer.close()
                break

print("Results")
print("Final position =>", data.body('wrist_3_link').xpos)
