from StringIO import StringIO

import rospy
from geometry_msgs.msg import Pose

import pose_cov_ops as pco


class PoseCovOps(object):
    def __init__(self):
        a = 0
    def _to_cpp(self, msg):
        """Return a serialized string from a ROS message

        Parameters
        ----------
        - msg: a ROS message instance.
        """
        buf = StringIO()
        msg.serialize(buf)
        return buf.getvalue()

    def _from_cpp(self, str_msg, cls):
        """Return a ROS message from a serialized string

        Parameters
        ----------
        - str_msg: str, serialized message
        - cls: ROS message class, e.g. sensor_msgs.msg.LaserScan.
        """
        msg = cls()
        return msg.deserialize(str_msg)

    def composePose(self, a, b):
        """
        Return a geometry_msgs/Pose instance.

        Parameters
        ----------
        - a: a geometry_msgs/Pose instance.
        - b: a geometry_msgs/Pose instance.
        """
        print isinstance(a, Pose)
        print isinstance(b, Pose)
        
        if not isinstance(a, Pose):
            rospy.ROSException('Argument 1 is not a geometry_msgs/Pose')
        if not isinstance(b, Pose):
            rospy.ROSException('Argument 2 is not a geometry_msgs/Pose')
        str_a = self._to_cpp(a)
        str_b = self._to_cpp(b)
        str_sum = pco.composePose(str_a, str_b)
        return self._from_cpp(str_sum, Pose)