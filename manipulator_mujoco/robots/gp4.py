import os
from manipulator_mujoco.robots.arm import Arm

_GP4_XML = os.path.join(
    os.path.dirname(__file__),
    '../assets/robots/gp4/gp4.xml',
)

_JOINTS = (
    'joint_1',
    'joint_2',
    'joint_3',
    'joint_4',
    'joint_5'
)

_EEF_SITE = 'eef_site'

_ATTACHMENT_SITE = 'attachment_site'

class GP4(Arm):
    def __init__(self, name: str = None):
        super().__init__(_GP4_XML, _EEF_SITE, _ATTACHMENT_SITE, _JOINTS, name)