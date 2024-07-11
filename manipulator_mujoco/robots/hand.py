from dm_control import mjcf

class Hand():
    def __init__(self, xml_path, joint_names, actuator_names, name: str = None):
        self._mjcf_root = mjcf.from_path(xml_path)
        self._joints = []
        self._actuators = []
        if name:
            self._mjcf_root.model = name
        # Find MJCF elements that will be exposed as attributes.
        for name in joint_names:
            self._joints.append(self._mjcf_root.find('joint', name))
        for name in actuator_names:
            self._actuators.append(self._mjcf_root.find('actuator', name))
        self._bodies = self.mjcf_model.find_all('body')
        self._eef_site = self._mjcf_root.find('site', "eef_site")

    @property
    # def joint(self):
    #     """List of joint elements belonging to the arm."""
    #     return self._joint
    def joints(self):
        """List of joint elements belonging to the arm."""
        return self._joints

    # @property
    # def actuator(self):
    #     """List of actuator elements belonging to the arm."""
    #     return self._actuator
    @property
    def actuators(self):
        """List of actuator elements belonging to the arm."""
        return self._actuators

    @property
    def mjcf_model(self):
        """Returns the `mjcf.RootElement` object corresponding to this robot."""
        return self._mjcf_root
    @property
    def eef_site(self):
        """List of actuator elements belonging to the arm."""
        return self._eef_site