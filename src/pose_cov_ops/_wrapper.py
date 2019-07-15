from StringIO import StringIO

import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovariance
import pose_cov_ops as pco


class PoseCovOps(object):
    def __init__(self):
        pass

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
        Return a geometry_msgs/Pose or geometry_msgs/PoseWithCovariance  instance.

        Parameters
        ----------
        - a: a geometry_msgs/Pose or geometry_msgs/PoseWithCovariance  instance.
        - b: a geometry_msgs/Pose or geometry_msgs/PoseWithCovariance  instance.
        """
        if not (isinstance(a, Pose) or isinstance(a, PoseWithCovariance)):
            rospy.ROSException(
                'Argument 1 is neither a geometry_msgs/Pose nor a geometry_msgs/PoseWithCovariance')
        if not (isinstance(b, Pose) or isinstance(b, PoseWithCovariance)):
            rospy.ROSException(
                'Argument 2 is neither a geometry_msgs/Pose nor a geometry_msgs/PoseWithCovariance')

        str_a = self._to_cpp(a)
        str_b = self._to_cpp(b)

        if (isinstance(a, PoseWithCovariance) and isinstance(b, PoseWithCovariance)):
            str_sum = pco.composePosePCPC(str_a, str_b)
        elif (isinstance(a, PoseWithCovariance) and isinstance(b, Pose)):
            str_sum = pco.composePosePCP(str_a, str_b)
        elif (isinstance(a, Pose) and isinstance(b, PoseWithCovariance)):
            str_sum = pco.composePosePPC(str_a, str_b)
        elif (isinstance(a, Pose) and isinstance(b, Pose)):
            str_sum = pco.composePosePP(str_a, str_b)
            return self._from_cpp(str_sum, Pose)
        return self._from_cpp(str_sum, PoseWithCovariance)

    def inverseComposePose(self, a, b):
        """
        Return a geometry_msgs/Pose or geometry_msgs/PoseWithCovariance  instance.

        Parameters
        ----------
        - a: a geometry_msgs/Pose or geometry_msgs/PoseWithCovariance  instance.
        - b: a geometry_msgs/Pose or geometry_msgs/PoseWithCovariance  instance.
        """
        if not (isinstance(a, Pose) or isinstance(a, PoseWithCovariance)):
            rospy.ROSException(
                'Argument 1 is neither a geometry_msgs/Pose nor a geometry_msgs/PoseWithCovariance')
        if not (isinstance(b, Pose) or isinstance(b, PoseWithCovariance)):
            rospy.ROSException(
                'Argument 2 is neither a geometry_msgs/Pose nor a geometry_msgs/PoseWithCovariance')

        str_a = self._to_cpp(a)
        str_b = self._to_cpp(b)

        if (isinstance(a, PoseWithCovariance) and isinstance(b, PoseWithCovariance)):
            str_sum = pco.inverseComposePosePCPC(str_a, str_b)
        elif (isinstance(a, PoseWithCovariance) and isinstance(b, Pose)):
            str_sum = pco.inverseComposePosePCP(str_a, str_b)
        elif (isinstance(a, Pose) and isinstance(b, PoseWithCovariance)):
            str_sum = pco.inverseComposePosePPC(str_a, str_b)
        elif (isinstance(a, Pose) and isinstance(b, Pose)):
            str_sum = pco.inverseComposePosePP(str_a, str_b)
            return self._from_cpp(str_sum, Pose)
        return self._from_cpp(str_sum, PoseWithCovariance)
