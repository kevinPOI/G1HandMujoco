import math
import torch
import pytorch_kinematics as pk
import os
_GP4_XML = os.path.join(
    os.path.dirname(__file__),
    '../assets/robots/gp4/gp4nm.xml',
)
chain = pk.build_serial_chain_from_mjcf(open(_GP4_XML).read(), "eef_site")
device = torch.device("cpu")
# goals are specified as Transform3d poses in the **robot frame**
# so if you have the goals specified in the world frame, you also need the robot frame in the world frame
pos = torch.tensor([0.0, 0.0, 0.0], device=device, dtype=torch.float32)
rot = torch.tensor([0.0, 0.0, 0.0], device=device, dtype=torch.float32)
rob_tf = pk.Transform3d(pos=pos, rot=rot, device=device)

# specify goals as Transform3d poses in world frame
# gpos = torch.tensor([0.0, 0.0, 0.0], device=device)
# grot = torch.tensor([0.0, 0.0, 0.0], device=device)
# rob_tf = pk.Transform3d(pos=gpos, rot=grot, device=device)
# goal_in_world_frame_tf = 
# convert to robot frame (skip if you have it specified in robot frame already, or if world = robot frame)
goal_in_rob_frame_tf = rob_tf

# get robot joint limits
lim = torch.tensor(chain.get_joint_limits(), device=device)

# create the IK object
# see the constructor for more options and their explanations, such as convergence tolerances
ik = pk.PseudoInverseIK(chain, max_iterations=30, num_retries=10,
                        joint_limits=lim.T,
                        early_stopping_any_converged=True,
                        early_stopping_no_improvement="all",
                        debug=False,
                        lr=0.2)
# solve IK
sol = ik.solve(goal_in_rob_frame_tf)
# num goals x num retries x DOF tensor of joint angles; if not converged, best solution found so far
print(sol.solutions)
# num goals x num retries can check for the convergence of each run
print(sol.converged)
# num goals x num retries can look at errors directly
print(sol.err_pos)
print(sol.err_rot)