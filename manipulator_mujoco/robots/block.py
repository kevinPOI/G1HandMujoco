from dm_control import mjcf
import os
class Block():
    def __init__(self, name: str = None):
        xml_path = os.path.join(
            os.path.dirname(__file__),
            '../assets/robots/block.xml',
        )
        self._mjcf_root = mjcf.from_path(xml_path)
        if name:
            self._mjcf_root.model = name
        # Find MJCF elements that will be exposed as attributes.
        self._bodies = self.mjcf_model.find_all('body')

    # @property
    # # def joint(self):
    # #     """List of joint elements belonging to the arm."""
    # #     return self._joint
    # def joints(self):
    #     """List of joint elements belonging to the arm."""
    #     return self._joints

    # # @property
    # # def actuator(self):
    # #     """List of actuator elements belonging to the arm."""
    # #     return self._actuator
    # @property
    # def actuators(self):
    #     """List of actuator elements belonging to the arm."""
    #     return self._actuators

    @property
    def mjcf_model(self):
        """Returns the `mjcf.RootElement` object corresponding to this robot."""
        return self._mjcf_root
    # @property
    # def eef_site(self):
    #     """List of actuator elements belonging to the arm."""
    #     return self._eef_site