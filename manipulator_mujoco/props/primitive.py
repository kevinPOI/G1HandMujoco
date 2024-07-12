from dm_control import mjcf
import numpy as np

class Primitive(object):
    """
    A base class representing a primitive object in a simulation environment.
    """

    def __init__(self, _mjfc_root, **kwargs):
        """
        Initialize the Primitive object.

        Args:
            **kwargs: Additional keyword arguments for configuring the primitive.
        """
        self._mjcf_model = mjcf.RootElement()
        self._mjcf_root = _mjfc_root
        # Add a geometric element to the worldbody
        self._geom = self._mjcf_root.worldbody.add("body", name="box", mocap=False)
        self._geom.add("geom", **kwargs)

    @property
    def geom(self):
        """Returns the primitive's geom, e.g., to change color or friction."""
        return self._geom
    
    @property
    def mjcf_model(self):
        """Returns the primitive's mjcf model."""
        return self._mjcf_model
