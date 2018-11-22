# opendlv-logic-cfsd18-sensation-slam

[![Build Status](https://travis-ci.org/cfsd/opendlv-logic-cfsd18-sensation-slam.svg?branch=master)](https://travis-ci.org/cfsd/opendlv-logic-cfsd18-sensation-slam)
## Overview
The architecture of SLAM system is supposed to summarize as front-end and back-end, in our systems front-end could be understood as the measurement from lidar, IMU and camera. The backend performs the inference on the data from the front-end,which is essentially a mathematical thing. The SLAM in our system is Least Squared SLAM which is also called GraphSLAM. This method uses graph which inlcudes edges and nodes to represet a unlinear optimization problem. The edges represent positions of poses and landmarks, while the edges represent the measurements or called constraints between the nodes. Graph optimization is currently mainstream optimization method in SLAM, which has higher efficiency, in particual when in visual SLAM scenario. However, since the lidar is used as the front-end of SLAM, some traditional filtering methods such as EKF,particle filter are supposed to assessed in the future design.

## Input and Output
＋Recieve
 - opendlv.proxy.GeodeticWgs84Reading
 - opendlv.proxy.GeodeticHeadingReading
 - opendlv.logic.sensation.Geolocation
 - opendlv.proxy.AngularVelocityReading
 - opendlv.logic.perception.ObjectType
 - opendlv.logic.perception.ObjectDirection
 - opendlv.logic.perception.ObjectDistance
 
 +Send
 - opendlv.logic.perception.ObjectType
 - opendlv.logic.perception.ObjectDirection
 - opendlv.logic.perception.ObjectDistance
