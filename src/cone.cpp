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

#include "cone.hpp"

Cone::Cone(double x, double y,int property,int id):
  m_x()
, m_y()
, m_property()
, m_id()
{
  m_x = x;
  m_y = y;
  m_property = property;
  m_id = id;
}


double Cone::getX(){
  return m_x;
}

double Cone::getY(){
  return m_y;
}

int Cone::getProperty(){
  return m_property;
}

int Cone::getId(){
  return m_id;
}

void Cone::setX(double x){
  m_x = x;
}

void Cone::setY(double y){
  m_y = y;
}

void Cone::setProperty(int property){
  m_property = property;
}

void Cone::setId(int id){
  m_id = id;
}
