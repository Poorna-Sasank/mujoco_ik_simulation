import numpy as np
import mujoco
from scipy.optimize import minimize

class ScipyIK:
    
    def __init__(self, model, data):
        self.model = model
        self.data = data

    def calculate(self, goal, init_q, body_id, method="SLSQP"):
        """
        Calculate joint angles for the desired position using SciPy optimization toolbox.
        """
        
        # Cost Function (error between goal pos and effector current pose)
        def cost_function(q):
            self.data.qpos[:] = q  # Current guess for joint angles
            mujoco.mj_forward(self.model, self.data) 
            current_pose = self.data.body(body_id).xpos  # Get end-effector position
            return np.linalg.norm(current_pose - goal)  # Euclidean distance to goal

        # Define joint limits as bounds for optimization
        bounds = [(self.model.jnt_range[i][0], self.model.jnt_range[i][1]) for i in range(len(init_q))]

        # Run the optimization
        result = minimize(
            cost_function,
            x0=init_q,               # Initial guess for joint angles
            method=method,           # SQP
            bounds=bounds            # bounds
        )

        if result.success:
            optimized_qpos = result.x  # Optimized joint angles
            self.data.qpos[:] = optimized_qpos  # Update MuJoCo with the result
            return optimized_qpos  
        else:
            raise ValueError("Inverse kinematic optim failed.")