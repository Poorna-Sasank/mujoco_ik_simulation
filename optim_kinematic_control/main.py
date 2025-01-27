import numpy as np
import mujoco
import mujoco.viewer
from gradIK import GradientDescentIK
from ik_calculator import ScipyIK

# Constants
MODEL_PATH = "C:/Users/comrade/Downloads/mujoco_menagerie-main/mujoco_menagerie-main/universal_robots_ur5e/scene.xml"
GOAL_POSITION = [0.49, 0.13, 0.59]
INITIAL_JOINT_ANGLES = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
NUM_STEPS = 10000

# Load the MuJoCo model and data
model = mujoco.MjModel.from_xml_path(MODEL_PATH)
data = mujoco.MjData(model)

# Set up the camera
camera = mujoco.MjvCamera()
mujoco.mjv_defaultFreeCamera(model, camera)
camera.distance = 1

# Get the body ID for the wrist_3_link
wrist_body_id = model.body('wrist_3_link').id

# Function to handle key callbacks
def key_callback(keycode):
    global pause
    if chr(keycode) == ' ':
        pause = not pause

# Function to perform inverse kinematics
def perform_ik(choice, goal, init_q, body_id):
    if choice == 1:
        solver = GradientDescentIK(model, data)
    elif choice == 2:
        solver = ScipyIK(model, data)
        return solver.calculate(goal, init_q, body_id, method="L-BFGS-B")
    else:
        raise ValueError("Invalid choice")
    return solver.calculate(goal, init_q, body_id)

# Main function
def main():
    # User input for IK method choice
    choice = int(input('Enter your choice\n1. Gradient Descent Optimization\n2. Limited-Memory BFGS Optimization\n3. Exit\n'))

    if choice == 3:
        print('Thanks for trying :)')
        return

    # Perform IK
    try:
        result_qpos = perform_ik(choice, GOAL_POSITION, INITIAL_JOINT_ANGLES, wrist_body_id)
    except ValueError as e:
        print(e)
        return

    # Interpolate joint angles for smooth animation
    interpolated_qpos = np.linspace(INITIAL_JOINT_ANGLES, result_qpos, NUM_STEPS)

    # Animation loop
    pause = False
    with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
        for qpos in interpolated_qpos:
            if viewer.is_running():
                data.qpos[:] = qpos
                mujoco.mj_forward(model, data)
                viewer.sync()

                if pause:
                    viewer.close()
                    break

    # Print final results
    print("Results")
    print("Final position =>", data.body('wrist_3_link').xpos)

if __name__ == "__main__":
    main()
