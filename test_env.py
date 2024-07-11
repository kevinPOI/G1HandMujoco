import gymnasium
import manipulator_mujoco

# create checkerboard floor arena
self._arena = StandardArena()

# create mocap target that OSC will try to follow
self._target = Target(self._arena.mjcf_model)

# ur5e arm
self._arm = Arm(
    xml_path=os.path.join(
        os.path.dirname(__file__),
        '../assets/robots/ur5e/ur5e.xml',
    ),
    eef_site_name='eef_site',
    attachment_site_name='attachment_site'
)

# attach arm to arena
self._arena.attach(
    self._arm.mjcf_model, pos=[0, 0, 0], quat=[0.7071068, 0, 0, -0.7071068]
)

# generate model
self._physics = mjcf.Physics.from_mjcf_model(self._arena.mjcf_model)

self._controller = OperationalSpaceController(
    physics=self._physics,
    joints=self._arm.joints,
    eef_site=self._arm.eef_site,
    min_effort=-150.0,
    max_effort=150.0,
    kp=200,
    ko=200,
    kv=50,
    vmax_xyz=1.0,
    vmax_abg=2.0,
)

target_pose = calculate_target_pose_for_OSC()  # [x, y, z, qx, qy, qz, qw]

# run OSC controller to move to target pose
self._controller.run(target_pose)

# step physics
self._physics.step()
