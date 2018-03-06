/**
 * Copyright (C) 2017 Chalmers Revere
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
 * USA.
 */

#ifndef SLAM_HPP
#define SLAM_HPP

#include "g2o/core/sparse_optimizer.h"
#include<Eigen/Dense>
#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

#include "cone.hpp"


class Slam {


private:
 Slam(const Slam &) = delete;
 Slam(Slam &&)      = delete;
 Slam &operator=(const Slam &) = delete;
 Slam &operator=(Slam &&) = delete;
public:
  Slam();
  ~Slam() = default;
  void nextContainer(cluon::data::Envelope data);

 private:
  void setUp();
  void tearDown();
  bool CheckContainer(uint32_t objectId, cluon::data::TimeStamp timeStamp);
  bool isKeyframe(cluon::data::TimeStamp startTime);
  void performSLAM(Eigen::MatrixXd Cones);
  Eigen::MatrixXd conesToGlobal(Eigen::Vector3d pose, Eigen::MatrixXd Cones);
  Eigen::Vector3d coneToGlobal(Eigen::Vector3d pose, Eigen::MatrixXd Cone);
  Eigen::Vector3d Spherical2Cartesian(double azimuth, double zenimuth, double distance);
  void addConesToMap(Eigen::MatrixXd cones, Eigen::Vector3d pose);
  //bool newCone(Eigen::MatrixXd cone,int poseId);
  void sendData();



/*Member variables*/
  int32_t m_timeDiffMilliseconds = 110;
  cluon::data::TimeStamp m_lastTimeStamp;
  Eigen::MatrixXd m_coneCollector;
  uint32_t m_lastObjectId;
  std::mutex m_coneMutex;
  std::mutex m_sensorMutex;
  std::mutex m_mapMutex;
  Eigen::Vector3d m_odometryData;
  //opendlv::data::environment::WGS84Coordinate m_gpsReference;
  std::vector<Cone> m_map;
  double m_newConeThreshold= 3;
  cluon::data::TimeStamp m_keyframeTimeStamp;
  double m_timeBetweenKeyframes = 0.5;
  double m_coneMappingThreshold = 67;
  int m_currentConeIndex = 0;
  

    // Constants for degree transformation
  const double DEG2RAD = 0.017453292522222; // PI/180.0
  const double RAD2DEG = 57.295779513082325; // 1.0 / DEG2RAD;
};


#endif
