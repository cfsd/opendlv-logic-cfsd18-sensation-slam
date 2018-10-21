# opendlv-logic-cfsd18-sensation-slam

[![Build Status](https://travis-ci.org/cfsd/opendlv-logic-cfsd18-sensation-slam.svg?branch=master)](https://travis-ci.org/cfsd/opendlv-logic-cfsd18-sensation-slam)
## Overview
The architecture of slam system is supposed to summarize as front-end and back-end, in our systems front-end could be understood as the measurement from lidar, IMU and camera. The backend performs the inference on the data from the front-end,which is essentially a mathematical thing. The slam in our system is Least Squared SLAM which is also called GraphSLAM. This method uses graph which inlcudes edges and nodes to represet a unlinear optimization problem. The edges represent positions of poses and landmarks, while the edges represent the measurements or called constraints between the nodes. 
