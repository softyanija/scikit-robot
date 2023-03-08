from cached_property import cached_property
import numpy as np

from skrobot.coordinates import CascadedCoords
from skrobot.data import pr2hand_urdfpath
from skrobot.model import RobotModel

from .urdf import RobotModelFromURDF


class PR2Hand(RobotModelFromURDF):

    """PR2Hand Robot Model.

    """

    def __init__(self, use_tight_joint_limit=True):
        super(PR2Hand, self).__init__()

        # self.rarm_end_coords = CascadedCoords(
        #     parent=self.r_gripper_tool_frame,
        #     name='rarm_end_coords')
        # self.larm_end_coords = CascadedCoords(
        #     parent=self.l_gripper_tool_frame,
        #     name='larm_end_coords')
        # self.head_end_coords = CascadedCoords(
        #     pos=[0.08, 0.0, 0.13],
        #     parent=self.head_tilt_link,
        #     name='head_end_coords').rotate(np.pi / 2.0, 'y')
        # self.torso_end_coords = CascadedCoords(
        #     parent=self.torso_lift_link,
        #     name='head_end_coords')

        # limbs

    @cached_property
    def default_urdf_path(self):
        return pr2hand_urdfpath()

    def gripper_distance(self, dist=None, arm='arms'):
        """Change gripper angle function

        Parameters
        ----------
        dist : None or float
            gripper distance.
            If dist is None, return gripper distance.
            If flaot value is given, change joint angle.
        arm : str
            Specify target arm.  You can specify 'larm', 'rarm', 'arms'.

        Returns
        -------
        dist : float
            Result of gripper distance in meter.
        """
        if arm == 'larm':
            joints = [self.l_gripper_l_finger_joint]
        elif arm == 'rarm':
            joints = [self.r_gripper_l_finger_joint]
        elif arm == 'arms':
            joints = [self.r_gripper_l_finger_joint,
                      self.l_gripper_l_finger_joint]
        else:
            raise ValueError('Invalid arm arm argument. You can specify '
                             "'larm', 'rarm' or 'arms'.")

        def _dist(angle):
            return 0.0099 * (18.4586 * np.sin(angle) + np.cos(angle) - 1.0101)

        if dist is not None:
            # calculate joint_angle from approximated equation
            max_dist = _dist(joints[0].max_angle)
            dist = max(min(dist, max_dist), 0)
            d = dist / 2.0
            angle = 2 * np.arctan(
                (9137 - np.sqrt(2)
                 * np.sqrt(-5e9 * (d**2) - 5e7 * d + 41739897))
                / (5 * (20000 * d + 199)))
            for joint in joints:
                joint.joint_angle(angle)
        angle = joints[0].joint_angle()
        return _dist(angle)
