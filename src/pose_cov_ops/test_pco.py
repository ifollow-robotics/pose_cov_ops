from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import Pose
from pose_cov_ops import PoseCovOps

PCO = PoseCovOps()

a = Pose()
a.orientation.w = 1.0 

b = PoseWithCovariance()
b.pose.orientation.w = 1.0

print 'Composition when two inputs are Pose'
print isinstance(PCO.composePose(a,a),Pose)


print 'Composition when two inputs are PoseWithCovariance and Pose'
print isinstance(PCO.composePose(b,a),PoseWithCovariance)


print 'Composition when inputs are Pose and PoseWithCovariance'
print isinstance(PCO.composePose(a,b),PoseWithCovariance)


print 'Composition when two inputs are PoseWithCovariance'
print isinstance(PCO.composePose(b,b),PoseWithCovariance)


print 'InverseComposition when two inputs are Pose'
print isinstance(PCO.inverseComposePose(a,a),Pose)


print 'InverseComposition when two inputs are PoseWithCovariance and Pose'
print isinstance(PCO.inverseComposePose(b,a),PoseWithCovariance)


print 'InverseComposition when inputs are Pose and PoseWithCovariance'
print isinstance(PCO.inverseComposePose(a,b),PoseWithCovariance)


print 'InverseComposition when two inputs are PoseWithCovariance'
print isinstance(PCO.inverseComposePose(b,b),PoseWithCovariance)
