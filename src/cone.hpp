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
    
    
    double getX();
    double getY();
    int getType();
    int getId();
    
    void setX(double x);
    void setY(double y);
    void setType(int type);
    void setId(int id);



  private:
    double m_x;
    double m_y;
    int m_type;
    int m_id;
    const double RAD2DEG = 57.295779513082325; // 1.0 / DEG2RAD;

};

#endif
