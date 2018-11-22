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

/*CVcones is an object containing the cones sent from detectcone
at the moment it just used as stopgap if the localization fails so that the car can get local information
in the second lap. If the localization improves and information can be sent from detectcone there is no need for this class*/

#ifndef CVCONES_HPP
#define CVCONES_HPP

#include <iostream>
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include "opendlv-standard-message-set.hpp"

#include "cluon-complete.hpp"
#include "cone.hpp"

typedef std::tuple<opendlv::logic::perception::ObjectDirection,opendlv::logic::perception::ObjectDistance,opendlv::logic::perception::ObjectType> ConePackage;

class CVCones{
  public:
    CVCones();
    ~CVCones() = default;
  
  void recieveCombinedMessage(cluon::data::TimeStamp currentFrameTime,std::map<int,ConePackage> currentFrame);
  void setCvCones(Eigen::MatrixXd cones);
  std::vector<Cone> getCvCones();
  void setTimeStamp(cluon::data::TimeStamp currentFrameTime);
  cluon::data::TimeStamp getTimeStamp();

  private:
  Eigen::Vector3d Spherical2Cartesian(double azimuth, double zenimuth, double distance);
  Eigen::Vector2d transformConeToCoG(double angle, double distance);

  std::vector<Cone> m_cones = {}; //Vector of cones received from detectcone
  cluon::data::TimeStamp m_lastTimeStamp = {}; //Timestamp of the received cones used for comparison
  std::mutex m_coneMutex; //Mutex for the access of the information in this class
  std::mutex m_stampMutex; //Mutex for altering timestamp
  // Constants for degree transformation
  const double DEG2RAD = 0.017453292522222; // PI/180.0
  const double RAD2DEG = 57.295779513082325; // 1.0 / DEG2RAD;
  const double PI = 3.14159265f;
};

#endif
