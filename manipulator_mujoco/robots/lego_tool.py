import os
from manipulator_mujoco.robots.suction_tool import SuctionTool

_LEGO_TOOL_XML = os.path.join(
    os.path.dirname(__file__),
    '../assets/robots/lego_tool/lego_tool.xml',
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

class LegoTool(SuctionTool):
    def __init__(self, name: str = None):
        super().__init__(_LEGO_TOOL_XML, name)