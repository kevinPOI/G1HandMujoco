import os
from manipulator_mujoco.robots.hand import Hand

_G1HAND_XML = os.path.join(
    os.path.dirname(__file__),
    '../assets/robots/g1_hand/g1_hand.xml',
)

_JOINT = (
    'right_zero_joint',
    'right_one_joint',
    'right_two_joint',
    'right_three_joint',
    'right_four_joint',
    'right_five_joint',
    'right_six_joint'
)

_ACTUATOR = (
    'right_zero_joint',
    'right_one_joint',
    'right_two_joint',
    'right_three_joint',
    'right_four_joint',
    'right_five_joint',
    'right_six_joint'
)

class G1Hand(Hand):
    def __init__(self, name: str = None):
        super().__init__(_G1HAND_XML, _JOINT, _ACTUATOR, name)