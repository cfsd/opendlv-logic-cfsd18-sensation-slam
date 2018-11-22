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

/*This is the class defining a cone object and its properties.
It is an attempt at object-oriented programming and is a good example of how a state based module
should be set up.*/

#ifndef CONE_HPP
#define CONE_HPP

#include <iostream>
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include "opendlv-standard-message-set.hpp"

class Cone{
  public:
    Cone(double x, double y,int type,int id);
    ~Cone() = default;
    
    opendlv::logic::perception::ObjectDirection getDirection(Eigen::Vector3d pose);
    opendlv::logic::perception::ObjectDistance getDistance(Eigen::Vector3d pose);
    double getOptX();
    double getOptY();
    void setOptX(double x);
    void setOptY(double y);
    double getMeanX();
    double getMeanY();
    double getX();
    double getY();
    int getType();
    int getId();
    
    void setMeanX(double x);
    void setMeanY(double y);
    void setType(int type);
    void setId(int id);
    void addObservation(Eigen::Vector3d localObservation,Eigen::Vector3d globalObservation,int i,int currConeId);
    Eigen::Vector2d getLocalConeObservation(int i);
    Eigen::Vector2d getGlobalConeObservation(int i);
    uint32_t getObservations();
    
    void calculateMean();
    Eigen::Vector2d getCovariance();
    std::vector<int> getConnectedPoses();
    void setOptimized();
    bool isOptimized();
    bool getLoopClosingState();
    void setValidState(bool state);
    bool isValid();

  private:
    double m_x;
    double m_y;
    int m_type; //Type of cone meaning the id of the color sent from detectcones CNN
    int m_id; //id of the cone meaning the order it was discovered
    const double RAD2DEG = 57.295779513082325; // 1.0 / DEG2RAD;
    std::vector<Eigen::Vector2d> m_observed = {}; //Observed global positions
    std::vector<Eigen::Vector2d> m_localObserved = {}; //Observed local positions
    double m_meanX = 0; //mean of all observations
    double m_meanY = 0;
    std::vector<int> m_connectedPoses = {}; //Connected pose Ids mainly used for plotting
    bool m_optimizedState = false; //If a cone has been part of an optimization
    bool m_looperCandidate = false; //IF a cone is a loop closer candidate
    double m_optX = 0; // THe g2o optimized position
    double m_optY = 0;
    bool m_validState = true; //If a cone is filtered from the map according to SLAM policy it is set to false and not used in the map
};

#endif
