import numpy as np
import numpy as np
import matplotlib.pyplot as plt
import gurobipy as gp
from gurobipy import GRB
import math


class MPC:
    def __init__(self ,A,B, N, Q, R) -> None:
        """
        MPC controller using Gurobi optimization.

        Parameters:
        - A, B: System matrices.
        - x0: Initial state.
        - 
        - N: Prediction horizon.
        - Q, R: Weight matrices for the state and input.
        - umin, umax: Minimum and maximum control inputs.
        
        """

        self.A = A
        self.B = B
        self.N =  N
        self.Q = Q
        self.R = R

        control_bound = 35
        self.umax =  control_bound
        self.umin = -control_bound


        self.nx = len(Q) # Number of states
        self.nu = len(R) # Number of inputs
           
    def control_gurobi(self, x0, ref, Dist):
        print('x_0 = ', x0)
        print('ref= ', ref)
        # print('ref_shape =', ref.shape)
        # print('R = ', self.R)
        # print('Q=', self.Q)
        # print('A=', self.A)
        # print('B= ', self.B)
    
        """
        calculate the control signal.
        - x0: Initial state.
        - Dist: Estimation of the Disturbance
        - ref: Reference trajectory (Nx1 vector).
        Returns:
        - u_opt: Optimal control input for the current time step.
        - predict_traj: prediction of the trajectory
        """

        # env = gp.Env(empty=True)

       

        # Turn off Gurobi output to the console
        # env.setParam('OutputFlag', 0)

        # Now you can create your model and solve it without seeing output in the console
        # model = gp.Model("example_model", env=env)

        
        # Create a new model
        m = gp.Model("mpc")
       

        x0 = np.reshape(x0, 2)
        # Decision variables for states and inputs
        x = m.addMVar((self.N+1, self.nx), lb=-GRB.INFINITY, name="x")
        u = m.addMVar((self.N, self.nu), lb= self.umin, ub=self.umax, name="u")
        
        # Initial state constraint
        m.addConstr(x[0, :] == x0, name="init")
    


        # Dynamics constraints
        for t in range(self.N):
            m.addConstr(x[t+1, :] ==  x[t, :] + self.B @ u[t, :]+Dist, name=f"dyn_{t}")
            # m.addConstr(u[t, 0]**2+u[t,1]**2 ==  1, name=f"sincos_{t}")
      
        # State constraints
        # for t in range(N+1):
        #     m.addConstr(x[t, :] >= xmin, name=f"xmin_{t}")
        #     m.addConstr(x[t, :] <= xmax, name=f"xmax_{t}")

        # Objective: Minimize cost function
        cost = 0
        gamma = 1
        for t in range(self.N):
            cost += gamma**t*(x[t, :] - ref[t, :]) @ self.Q @ (x[t, :] - ref[t, :]) + u[t, :] @ self.R @ u[t, :]
        
        # cost+=1000* (x[t, :] - ref[t, :]) @ Q @ (x[t, :] - ref[t, :])
        # for t in range(N-1):
        #     cost += (u[t, :]-u[t+1,:]) @ R @ (u[t, :]-u[t+1,:])
   
        m.setObjective(cost, GRB.MINIMIZE)

        
        # Optimize model
        # m.params.NonConvex = 2
        m.optimize()
     
        
        u_opt = u.X[0, :]  # Get optimal control input for the current time step
        predict_traj = x.X
        return u_opt, predict_traj
    

    """def control_cvx(self, x0, Dist, ref):
        # Variables
        x = cp.Variable((self.N+1, self.nx))
        u = cp.Variable((self.N, self.nu))

        # Constraints
        constraints = [x[0, :] == x0]  # Initial state constraint

        # Dynamics constraints
        for t in range(self.N):
            constraints.append(x[t+1, :] == x[t, :] + self.B @ u[t, :] + Dist[t, :])

        # Input constraints
        constraints += [
            u >= self.umin,
            u <= self.umax
        ]

        # Define the cost function
        cost = 0
        gamma = 1
        for t in range(self.N):
            cost += cp.quad_form(x[t, :] - ref[t, :], self.Q) + cp.quad_form(u[t, :], self.R)

        # Define and solve the problem
        problem = cp.Problem(cp.Minimize(cost), constraints)
        problem.solve()

        # Check if the problem was successfully solved
        if problem.status not in ["infeasible", "unbounded"]:
            # Assuming the problem is feasible and bounded
            u_opt = u.value[0, :]  # Optimal control input for the current time step
            predict_traj = x.value
            return u_opt, predict_traj
        else:
            raise Exception("The problem is infeasible or unbounded.")"""
    

    def convert_control(self, u_mpc):

        f_t = np.linalg.norm(u_mpc)
        alpha_t = math.atan2(-u_mpc[1], u_mpc[0]) - np.pi/2
        return f_t, alpha_t
                
            
 