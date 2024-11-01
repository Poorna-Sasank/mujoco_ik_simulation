import numpy as np
import mujoco

class GradientDescentIK:
    
    def __init__(self, model, data, step_size=0.5, tol=0.01, alpha=0.5):
        self.model = model
        self.data = data
        self.step_size = step_size
        self.tol = tol
        self.alpha = alpha
        self.jacp = np.zeros((3, model.nv))  # translation jacobian
        self.jacr = np.zeros((3, model.nv))  # rotational jacobian

    def check_joint_limits(self, q):
        """Check if the joints are within their limits."""
        for i in range(len(q)):
            q[i] = max(self.model.jnt_range[i][0], 
                       min(q[i], self.model.jnt_range[i][1]))

    def calculate(self, goal, init_q, body_id):
        """Calculate desired joint angles for a goal position."""
        self.data.qpos = init_q
        mujoco.mj_forward(self.model, self.data)
        current_pose = self.data.body(body_id).xpos
        error = np.subtract(goal, current_pose)

        while (np.linalg.norm(error) >= self.tol):
            mujoco.mj_jac(self.model, self.data, self.jacp, self.jacr, goal, body_id)
            grad = self.alpha * self.jacp.T @ error
            self.data.qpos += self.step_size * grad
            self.check_joint_limits(self.data.qpos)
            mujoco.mj_forward(self.model, self.data) 
            error = np.subtract(goal, self.data.body(body_id).xpos)
