import time
import threading
import numpy as np
from dm_control import mjcf
import mujoco.viewer
import gymnasium as gym
from gymnasium import spaces
from manipulator_mujoco.arenas import StandardArena
from manipulator_mujoco.robots import AuboI5, AG95, G1Hand, Block
from manipulator_mujoco.props import Primitive
from manipulator_mujoco.mocaps import Target
from manipulator_mujoco.controllers import OperationalSpaceController, JointEffortController
from manipulator_mujoco.utils.transform_utils import (
    mat2quat,
)
counter = 0
m1_counter = 0
mode = 0
class AuboI5Env(gym.Env):

    metadata = {
        "render_modes": ["human", "rgb_array"],
        "render_fps": None,
    }  # TODO add functionality to render_fps

    def __init__(self, render_mode=None):
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
        self._arm = AuboI5()
        self._block = Block()
        # ag95 gripper
        #self._gripper = AG95()
        self._gripper = G1Hand()
        # attach gripper to arm
        self._arm.attach_tool(self._gripper.mjcf_model, pos=[0, 0, 0], quat=[0, 0, 0, 1])

        # small box to be manipulated
        #self._box = Primitive(self._arena.mjcf_model, type="box", size=[0.03, 0.03, 0.03], pos=[0,0,0.02], rgba=[1, 0, 0, 1], friction=[1, 0.3, 0.0001])

        # attach arm to arena
        self._arena.attach(
            self._arm.mjcf_model, pos=[0,0,0]
        )

        # attach box to arena as free joint
        self._arena.attach_free(
            self._block.mjcf_model, pos=[0.5,0,0]
        )
       
        # generate model
        self._physics = mjcf.Physics.from_mjcf_model(self._arena.mjcf_model)

        # set up OSC controller
        self._controller = OperationalSpaceController(
            physics=self._physics,
            joints=self._arm.joints,
            eef_site=self._arm.eef_site,
            min_effort=-150.0,
            max_effort=150.0,
            kp=300,
            ko=300,
            kv=30,
            vmax_xyz=1.0,
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
            self._physics.bind(self._arm.joints).qpos = [0, 0, 1.5707, 0, 1.5707, 0]
            # put target in a reasonable starting position
            self._target.set_mocap_pose(self._physics, position=[0.5, 0, 0.04], quaternion=[0, 0, 0, 1])

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
    def step(self, action) -> tuple:
        # TODO use the action to control the arm
        global counter
        global m1_counter
        global mode
        counter += 1
        ee_pose, block_pose = self._get_obs()

        # get mocap target pose
        #target_pose = self._target.get_mocap_pose(self._physics)
        target_pose = block_pose
        dest_pose = np.asarray([0.5,0.2,0.2,0,0,0,1])

        # run OSC controller to move to target pose
        m = np.asarray([0,-1,-1,1,1,1,1])
        b = np.asarray([0,0,0,0.2,0.2,0.2,0.2])
        if np.linalg.norm(ee_pose[:2] - block_pose[:2]) < 0.1 and np.linalg.norm(ee_pose[2] - block_pose[2]) < 0.04:
            mode = 1
        if mode == 1:
            action = 0.5
            m1_counter += 1
        else:
            action = 0
        if m1_counter > 800:
            target_pose = dest_pose
        
        grip_target = action * m + b
        self.gripper_to(grip_target)
        
        if counter % 10 == 0:
            print("ee_pose: ", ee_pose, "\ntarget_pose: ", target_pose)
        
        # if(np.linalg.norm(self._get_obs()[0:3] - np.array([0.5,0,0.02]))) < 0.5:
        #     #arm_eef_pos = self._physics.bind(self._arm.eef_site).xpos
        #     #self._physics.bind(self._gripper.joint).qfrc_applied = 4
        #     #print("eef_site: ", joint_pos)
        #     #print("arm_eef_site: ", arm_eef_pos)
        #     self._controller.run(target_pose)
        #     #self._gripper_controller.run(2)
        # else:
        #     #self._physics.bind(self._gripper.joint).qfrc_applied = -3
        #     self._controller.run(target_pose)
        self._controller.run(target_pose)

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
