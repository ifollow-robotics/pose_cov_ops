
 * Travis CI: [![Build Status](https://travis-ci.org/mrpt-ros-pkg/pose_cov_ops.svg?branch=master)](https://travis-ci.org/mrpt-ros-pkg/pose_cov_ops)
 * ROS build farm:
   * git master:
     * ROS Kinetic @ u16.04 Xenial [![Build Status](http://build.ros.org/job/Kdev__pose_cov_ops__ubuntu_xenial_amd64/badge/icon)](http://build.ros.org/job/Kdev__pose_cov_ops__ubuntu_xenial_amd64/)
     * ROS Melodic @ u18.04 Bionic [![Build Status](http://build.ros.org/job/Mdev__pose_cov_ops__ubuntu_bionic_amd64/badge/icon)](http://build.ros.org/job/Mdev__pose_cov_ops__ubuntu_bionic_amd64/)
   * Last released version:
     * ROS Kinetic @ u16.04 Xenial: [![Build Status](http://build.ros.org/job/Kbin_uX64__pose_cov_ops__ubuntu_xenial_amd64__binary/badge/icon)](http://build.ros.org/job/Kbin_uX64__pose_cov_ops__ubuntu_xenial_amd64__binary/)
     * ROS Melodic @ u18.04 Bionic: [![Build Status](http://build.ros.org/job/Mbin_uB64__pose_cov_ops__ubuntu_bionic_amd64__binary/badge/icon)](http://build.ros.org/job/Mbin_uB64__pose_cov_ops__ubuntu_bionic_amd64__binary/)


pose_cov_ops
============

MRPT C++ library-wrapper for SE(2) and SE(3) poses and points geometric operations with uncertainty.

Docs: http://wiki.ros.org/pose_cov_ops

This iFollow fork has python bindings implemented. 

```python
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import Pose
from pose_cov_ops import PoseCovOps
PCO = PoseCovOps()
# example inputs
a = Pose()
a.orientation.w = 1.0 
b = PoseWithCovariance()
b.pose.orientation.w = 1.0
```
Compositions irrespective of input with/without Covariances
```python
PCO.composePose(a,a)
PCO.composePose(a,b)
PCO.composePose(b,a)
PCO.composePose(b,b)
```
Inverse Compositions irrespective of input with/without Covariances
```python
PCO.inverseComposePose(a,a)
PCO.inverseComposePose(a,b)
PCO.inverseComposePose(b,a)
PCO.inverseComposePose(b,b)
```

