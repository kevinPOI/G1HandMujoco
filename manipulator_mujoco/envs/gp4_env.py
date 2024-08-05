import time
import os 
import threading
import numpy as np
from dm_control import mjcf
from dm_control.utils import inverse_kinematics as ik
import mujoco.viewer
import gymnasium as gym
from gymnasium import spaces
from manipulator_mujoco.arenas import StandardArena
from manipulator_mujoco.robots import AuboI5, AG95, G1Hand, Block, GP4,Lego, LegoTool
from manipulator_mujoco.props import Primitive
from manipulator_mujoco.mocaps import Target
from manipulator_mujoco.controllers import OperationalSpaceController, JointEffortController
from manipulator_mujoco.utils.transform_utils import (
    mat2quat,
)
counter = 0
m1_counter = 0
mode = 0
import scipy.spatial.transform

def set_y_rotation(quaternion, y):
    # Create a rotation object from the quaternion
    rotation = scipy.spatial.transform.Rotation.from_quat(quaternion)
    
    # Convert quaternion to Euler angles
    euler_angles = rotation.as_euler('xyz', degrees=False)
    
    # Set the y-axis rotation to zero
    euler_angles[1] = y
    
    # Convert back to quaternion
    new_rotation = scipy.spatial.transform.Rotation.from_euler('xyz', euler_angles)
    new_quaternion = new_rotation.as_quat()
    
    return new_quaternion
def set_x_rotation(quaternion, x):
    # Create a rotation object from the quaternion
    rotation = scipy.spatial.transform.Rotation.from_quat(quaternion)
    
    # Convert quaternion to Euler angles
    euler_angles = rotation.as_euler('xyz', degrees=False)
    
    # Set the x-axis rotation to zero
    euler_angles[0] = x
    
    # Convert back to quaternion
    new_rotation = scipy.spatial.transform.Rotation.from_euler('xyz', euler_angles)
    new_quaternion = new_rotation.as_quat()
    
    return new_quaternion

def set_z_rotation(quaternion, z):
    # Create a rotation object from the quaternion
    rotation = scipy.spatial.transform.Rotation.from_quat(quaternion)
    
    # Convert quaternion to Euler angles
    euler_angles = rotation.as_euler('xyz', degrees=False)
    
    # Set the y-axis rotation to zero
    euler_angles[2] = z
    
    # Convert back to quaternion
    new_rotation = scipy.spatial.transform.Rotation.from_euler('xyz', euler_angles)
    new_quaternion = new_rotation.as_quat()
    
    return new_quaternion

def xyz_to_quat(xyz):
    new_rotation = scipy.spatial.transform.Rotation.from_euler('xyz', xyz)
    quat = new_rotation.as_quat()
    return quat

def pose_xyz_to_quat(pose):
    xyz = pose[3:]
    new_rotation = scipy.spatial.transform.Rotation.from_euler('xyz', xyz)
    quat = new_rotation.as_quat()
    return np.concatenate([pose[:3], quat])

def quat_to_xyz(quat):
    new_rotation = scipy.spatial.transform.Rotation.from_quat(quat)
    xyz = new_rotation.as_euler('xyz')
    return xyz

class GP4Env(gym.Env):

    metadata = {
        "render_modes": ["human", "rgb_array"],
        "render_fps": None,
    }  # TODO add functionality to render_fps

    def __init__(self, render_mode=None):
        self.tool = False
        # TODO come up with an observation space that makes sense
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(6,), dtype=np.float64
        )

        # TODO come up with an action space that makes sense
        self.action_space = spaces.Box(
            low=-0.1, high=0.1, shape=(6,), dtype=np.float64
        )

        assert render_mode is None or render_mode in self.metadata["render_modes"]
        self._render_mode = render_mode

        ############################
        # create MJCF model
        ############################
        
        # checkerboard floor
        self._arena = StandardArena()

        # mocap target that OSC will try to follow
        self._target = Target(self._arena.mjcf_model)

        # aubo i5 arm
        self._arm = GP4()
        _GP4_XML = os.path.join(
            os.path.dirname(__file__),
            '../assets/robots/gp4/gp4.xml',
        )
        # self._arm_physics = mujoco.Physics.from_xml_path(_GP4_XML)
        # self._arm_physics.mjcf.Physics.from_mjcf_model(self._arena.mjcf_model)
        self._block = Block()
        self._lego_2x8 = Lego("lego_2x8.xml")
        #self._gripper = AG95()
        self._gripper = G1Hand()
        self._tool = LegoTool()
        # attach gripper to arm
        if self.tool:
            self._arm.attach_tool(self._tool.mjcf_model, pos=[0.09725, 0, 0], quat=xyz_to_quat([0,1.57,0]))
            self._arena.attach_free(self._block.mjcf_model, pos=[0.5,0.2,0])
            self._arena.attach_free(self._lego_2x8.mjcf_model, pos = [0.5, 0, 0], quat=[0.7,0,0, 0.7])
        else:
            self._arm.attach_tool(self._gripper.mjcf_model, pos=[0.05, 0, 0], quat=[0.5, 0.5, 0.5, 0.5])
            self._arena.attach_free(
                self._block.mjcf_model, pos=[0.6,0,0]
            )
            self._arena.attach_free(self._lego_2x8.mjcf_model, pos = [0.6, 0.1, 0], quat=[0.7,0,0, 0.7])
        # attach arm to arena
        self._arena.attach(
            self._arm.mjcf_model, pos=[0,0,0]
        )

        # attach box to arena as free joint
        
       
        # generate model
        self._physics = mjcf.Physics.from_mjcf_model(self._arena.mjcf_model)

        # set up OSC controller
        self._controller = OperationalSpaceController(
            physics=self._physics,
            joints=self._arm.joints,
            eef_site=self._arm.eef_site,
            min_effort=-15000.0,
            max_effort=15000.0,
            kp=3000,
            ko=3000,
            kv=100,
            vmax_xyz=1.5,
            vmax_abg=2.0,
        )

        # self._gripper_controller = JointEffortController(
        #     physics=self._physics,
        #     joints=self._gripper.joint,
        #     min_effort=-150.0,
        #     max_effort=150.0
        # )

        # self._gripper_controller = OperationalSpaceController(
        #     physics=self._physics,
        #     joints=[self._gripper.joint],
        #     eef_site=self._arm.eef_site,
        #     min_effort=-150.0,
        #     max_effort=150.0,
        #     kp=300,
        #     ko=300,
        #     kv=30,
        #     vmax_xyz=1.0,
        #     vmax_abg=2.0,
        # )

        # for GUI and time keeping
        self._timestep = self._physics.model.opt.timestep
        self._viewer = None
        self._step_start = None
        self.mode = 0
    
    def _get_obs(self) -> np.ndarray:
        # TODO come up with an observations that makes sense for your RL task
        eef_site=self._arm.eef_site
        ee_pos = self._physics.bind(eef_site).xpos
        ee_quat = mat2quat(self._physics.bind(eef_site).xmat.reshape(3, 3))
        ee_pose = np.concatenate([ee_pos, ee_quat])


        block = self._physics.bind(self._block._mjcf_root.find("body", "block"))
        block_pos = block.xpos
        block_quat = mat2quat(block.xmat.reshape(3, 3))
        block_pose = np.concatenate([block_pos, block_quat])
        return ee_pose, block_pose

    def _get_info(self) -> dict:
        # TODO come up with an info dict that makes sense for your RL task
        return {}

    def reset(self, seed=None, options=None) -> tuple:
        super().reset(seed=seed)

        # reset physics
        with self._physics.reset_context():
            # put arm in a reasonable starting position
            self._physics.bind(self._arm.joints).qpos = [0, 0, 1.5707, 0, 1.5707]
            # put target in a reasonable starting position
            self._target.set_mocap_pose(self._physics, position=[0.48, 0, 0.25], quaternion=[0, 0, 0, 1])

        observation = self._get_obs()
        info = self._get_info()

        return observation, info
    def gripper_to(self, target):
        joint_pos = self._physics.bind(self._gripper.joints).qpos
        joint_vel = self._physics.bind(self._gripper.joints).qvel
        #print("err", (target - joint_pos)," joint_vel: ", joint_vel)
        effort = (target - joint_pos)*70 - joint_vel * 0.3
        #self._physics.bind(self._gripper.joint).qfrc_applied = effort
        self._physics.bind(self._gripper.actuators).ctrl = effort
    def step(self, mode) -> tuple:
        # TODO use the action to control the arm
        global counter
        global m1_counter
        #global mode
        counter += 1
        ee_pose, block_pose = self._get_obs()

        # get mocap target pose
        target_r = xyz_to_quat([0,0,1.5])
        target_pose = pose_xyz_to_quat(np.array([0.45,0,0.3,0,0,1.5]))
        
        dest_pose = np.asarray([0.5,0.2,0.2,0,0,0,1])

        # run OSC controller to move to target pose
        block_coe = np.asarray([0,-1,-1,1,1,1,1])
        lego_coe = np.asarray([0,-0.3,-0.3,1,1,1,1])
        m = block_coe
        block_init = np.asarray([0,-0.3,-0.6,0.3,0.3,0.3,0.3])#[thumb_pan, thumb1, thumb2, index_1, index_2, middle_1, middle_2]
        lego_init = np.asarray([0,-0.4,-0.4,0.9,0.6,0.9,0.6])
        b = block_init
        if isinstance(mode, list):#list: set ee position
            target_pose_raw = np.asarray(mode)
            quat = xyz_to_quat(target_pose_raw[3:])
            target_pose = np.concatenate([target_pose_raw[:3], quat])
            action = 0
        else:#number: set grip
            action = mode
        
        grip_target = action * m + b
        if(not self.tool):
            self.gripper_to(grip_target)

        if self.tool:
            #target_pose[3:] = set_y_rotation(ee_pose[3:],0)
            target_pose[3:] = set_x_rotation(ee_pose[3:], 4.6)
            #target_pose[3:] =  ee_pose[3:]
            pass
            
        else:
            target_rot = target_pose[3:]
            target_rot_xyz = quat_to_xyz(target_pose[3:])
            ee_pose_xyz = quat_to_xyz(ee_pose[3:])
            target_rot_xyz[0] = 3.5
            target_rot_xyz[1] = 0
            target_rot_xyz[2] = 0.02 * target_rot_xyz[2] + 0.98 * ee_pose_xyz[2]
            target_pose[3:] = xyz_to_quat(target_rot_xyz)
        
        # if counter % 50 == 0:
        #     print("ee_pose: ", ee_pose, "\ntarget_pose: ", target_pose)
        #     ee_rot = scipy.spatial.transform.Rotation.from_quat(ee_pose[3:]).as_euler('xyz', degrees=False)
        #     target_rot = scipy.spatial.transform.Rotation.from_quat(target_pose[3:]).as_euler('xyz', degrees=False)
        #     print("ee_rot: ", ee_rot, "target_rot:", target_rot)
        self._controller.run_v2(target_pose)

        # step physics
        self._physics.step()

        # render frame
        if self._render_mode == "human":
            self._render_frame()
        
        # TODO come up with a reward, termination function that makes sense for your RL task
        observation = self._get_obs()
        reward = 0
        terminated = False
        info = self._get_info()

        return observation, reward, terminated, False, info

    def render(self) -> np.ndarray:
        """
        Renders the current frame and returns it as an RGB array if the render mode is set to "rgb_array".

        Returns:
            np.ndarray: RGB array of the current frame.
        """
        if self._render_mode == "rgb_array":
            return self._render_frame()

    def _render_frame(self) -> None:
        """
        Renders the current frame and updates the viewer if the render mode is set to "human".
        """
        if self._viewer is None and self._render_mode == "human":
            # launch viewer
            self._viewer = mujoco.viewer.launch_passive(
                self._physics.model.ptr,
                self._physics.data.ptr,
            )
        if self._step_start is None and self._render_mode == "human":
            # initialize step timer
            self._step_start = time.time()

        if self._render_mode == "human":
            # render viewer
            self._viewer.sync()

            # TODO come up with a better frame rate keeping strategy
            time_until_next_step = self._timestep - (time.time() - self._step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

            self._step_start = time.time()

        else:  # rgb_array
            return self._physics.render()

    def close(self) -> None:
        """
        Closes the viewer if it's open.
        """
        if self._viewer is not None:
            self._viewer.close()

